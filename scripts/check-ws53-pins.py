#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Compare WS53 pin fixtures with compiled SDK enum/tables and generated PAC SVD.

No SDK code is executed on a device, and the C harness does not access MMIO.
Normal Cargo tests use the checked-in derived fixture, without requiring the SDK.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

PIN = "163d9150ab94a02d09a05f623351f9facee3025e"
SOURCES = (
    "src/drivers/chips/ws53/include/acore/platform_core.h",
    "src/drivers/chips/ws53/porting/pinctrl/pinctrl_porting.c",
    "src/drivers/drivers/hal/pinmux/ws53/hal_pinctrl_ws53.c",
    "src/drivers/chips/ws53/porting/include/pinctrl_porting.h",
)


def extract(text, pattern):
    match = re.search(pattern, text, re.S)
    if match is None:
        raise ValueError(f"SDK source shape changed: {pattern}")
    return match[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--svd", type=Path, required=True)
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--write-fixture", action="store_true")
    args = parser.parse_args()
    sdk = args.sdk.resolve()
    subprocess.run(["git", "-C", str(sdk), "diff", "--exit-code", PIN, "--", *SOURCES],
                   check=True, stdout=subprocess.DEVNULL)
    platform, port, hal, header = [(sdk / path).read_text() for path in SOURCES]
    pin_enum = extract(platform, r"typedef enum \{\s*S_MGPIO0\b.*?\} pin_t;")
    mode_enum = extract(port, r"typedef enum \{\s*PIO_MODE_0\b.*?\} pio_mode_t;")
    modes = extract(port, r"static uint8_t const g_pio_pins_avaliable_mode\[PIN_MAX_NUMBER\] = \{.*?\};")
    group = extract(hal, r"typedef struct \{\s*pin_t begin;.*?\} hal_pin_config_group_t;")
    tables = [extract(hal, rf"static hal_pin_config_group_t const g_pin_{kind}_map\[.*?\}};")
              for kind in ("mode", "ds", "ds2")]
    # Copy constant definitions and data declarations, never SDK MMIO helpers.
    defines = "\n".join(re.findall(r"^#define HAL_.*$", hal, re.M))
    names = [(kind, i) for kind, count in (("MGPIO", 23), ("AGPIO", 13), ("SGPIO", 3))
             for i in range(count)]
    aliases = [f'printf("{kind},{i},%u\\n", (unsigned)S_{kind}{i});' for kind, i in names]
    known_ids = "static const pin_t known_ids[] = {" + ",".join(
        f"S_{kind}{i}" for kind, i in names) + "};"
    pull_enum = extract(header, r"typedef enum \{\s*PIN_PULL_NONE\b.*?\} pin_pull_t;")
    field_names = {"MODE": "SEL", "DRIVE_LOW": "DS", "DRIVE_HIGH": "DS2",
                   "INPUT_ENABLE": "IE", "PULL": "PULL", "SCHMITT_ENABLE": "ST"}
    field_prints = [f'printf("FIELD,{name},%u,%u\\n", HAL_PIN_{macro}_START_BIT, HAL_PIN_{macro}_BITS_NUM);'
                    for name, macro in field_names.items()]
    program = "\n".join([
        "#include <stdint.h>\n#include <stdio.h>", "#define bit(x) (1U << (x))",
        "#define PIN_MAX_NUMBER 48", pin_enum, mode_enum, modes, defines, group, *tables, known_ids, pull_enum,
        '_Static_assert(PIN_PULL_NONE == 0 && PIN_PULL_UP == 1 && PIN_PULL_DOWN == 2, "pull encodings");',
        r'''
static const hal_pin_config_group_t *lookup(const hal_pin_config_group_t *g,
                                           unsigned count, unsigned pin) {
    for (unsigned i = 0; i < count; ++i) {
        if (pin >= (unsigned)g[i].begin && pin <= (unsigned)g[i].end) return &g[i];
    }
    return NULL;
}
static uint32_t address(const hal_pin_config_group_t *g, unsigned pin) {
    return g && g->per_num ? g->reg_addr + (pin - g->begin) / g->per_num * 4 : 0;
}
int main(void) {
    for (unsigned pin = 0; pin < 48; ++pin) {
        unsigned mask = g_pio_pins_avaliable_mode[pin];
        if (!mask && pin != S_AGPIO12) continue;
        /* Reject legacy duplicate mux slots that are not application pin_t
           values. Alias equality comes from the original compiled enum. */
        unsigned recognized = 0;
        for (unsigned i = 0; i < sizeof(known_ids) / sizeof(known_ids[0]); ++i)
            if ((unsigned)known_ids[i] == pin) recognized = 1;
        if (!recognized) continue;
        const hal_pin_config_group_t *m = lookup(g_pin_mode_map, HAL_PIN_MODE_GROUP_NUM, pin);
        const hal_pin_config_group_t *d = lookup(g_pin_ds_map, HAL_PIN_DS_GROUP_NUM, pin);
        const hal_pin_config_group_t *d2 = lookup(g_pin_ds2_map, HAL_PIN_DS2_GROUP_NUM, pin);
        if (!d || d->bits_num != 2 || !d->per_num) return 2;
        unsigned bits = d->bits_num + (d2 && d2->per_num ? d2->bits_num : 0);
        printf("PAD,%u,%u,%u,%u,%u\n", pin, mask, bits, address(m, pin), address(d, pin));
    }
''', *aliases, *field_prints, "return 0; }"])
    with tempfile.TemporaryDirectory(prefix="ws53-pin-sdk-") as work:
        binary = Path(work) / "pins"
        subprocess.run([args.cc, "-std=c11", "-Wall", "-Werror", "-x", "c", "-", "-o", str(binary)],
                       input=program, text=True, check=True)
        output = subprocess.check_output([str(binary)], text=True)
    rows = [line.split(",") for line in output.splitlines()]
    pads = [list(map(int, row[1:])) for row in rows if row[0] == "PAD"]
    alias_rows = [row for row in rows if row[0] in ("MGPIO", "AGPIO", "SGPIO")]
    fields = {row[1]: (int(row[2]), int(row[3])) for row in rows if row[0] == "FIELD"}
    # Keep the derived fixture stable; field checks are against the input SVD.
    output = "\n".join(line for line in output.splitlines() if not line.startswith("FIELD,")) + "\n"
    if len(pads) != 31 or len(alias_rows) != 39:
        raise ValueError("unexpected canonical/alias pin inventory")

    svd = ET.parse(args.svd)
    registers = {}
    for p in svd.findall("./peripherals/peripheral"):
        if not p.findtext("name").startswith("PINMUX_"):
            continue
        base = int(p.findtext("baseAddress"), 0)
        for r in p.findall("./registers/register"):
            registers[r.findtext("name")] = (base + int(r.findtext("addressOffset"), 0), r)
    checked = 0
    field_checks = 0
    for sdk_id, mask, bits, mux, pad in pads:
        if sdk_id < 23:
            name = f"MGPIO{sdk_id}"
        elif sdk_id < 45:
            name = f"AGPIO{sdk_id - 32}"
        else:
            name = f"MGPIO{(14, 15, 21)[sdk_id - 45]}"
        pad_address, reg = registers[name + "_PAD"]
        if pad_address != pad:
            raise ValueError(f"{name}: SDK/PAC pad address mismatch")
        drive_fields = [f for f in reg.findall("./fields/field") if f.findtext("name").startswith("DRIVE_")]
        if sum(int(f.findtext("bitWidth")) for f in drive_fields) != bits:
            raise ValueError(f"{name}: SDK/PAC drive width mismatch")
        for field in reg.findall("./fields/field"):
            got = (int(field.findtext("bitOffset")), int(field.findtext("bitWidth")))
            if got != fields[field.findtext("name")]:
                raise ValueError(f"{name}: SDK/PAC pad field mismatch")
            field_checks += 1
        if sdk_id == 44:
            if mux != 0 or mask != 0:
                raise ValueError("reset pad unexpectedly muxable")
        else:
            mux_address, mux_reg = registers[name + "_SEL"]
            if mux_address != mux or mux_reg.findtext("fields/field/bitWidth") != "3":
                raise ValueError(f"{name}: SDK/PAC mux mismatch")
            f = mux_reg.find("fields/field")
            if (int(f.findtext("bitOffset")), int(f.findtext("bitWidth"))) != fields["MODE"]:
                raise ValueError(f"{name}: SDK/PAC mode field mismatch")
            field_checks += 1
            checked += 1
    fixture = "# Derived facts, not SDK source. fbb_ws53 " + PIN + "\n"
    fixture += "# PAD,sdk_id,mode_mask,drive_bits,mux_address,pad_address; namespace,index,sdk_id\n" + output
    path = Path(__file__).resolve().parents[1] / "tests/fixtures/ws53-pins.csv"
    if args.write_fixture:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(fixture)
    elif path.read_text() != fixture:
        raise ValueError("pin fixture differs from compiled SDK facts")
    print(json.dumps({"sdk_commit": PIN, "source_sha256": {
        name: hashlib.sha256((sdk / name).read_bytes()).hexdigest() for name in SOURCES},
        "canonical_pads": len(pads), "chip_names": len(alias_rows),
        "pac_mux_checks": checked, "pac_pad_drive_checks": len(pads),
        "pac_field_checks": field_checks, "pull_encodings_checked": 3,
        "fixture_sha256": hashlib.sha256(fixture.encode()).hexdigest(),
        "hardware_access": False, "hardware_verified": False}, indent=2))


if __name__ == "__main__":
    main()
