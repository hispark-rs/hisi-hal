//! Shared byte-level I2C transaction planning for command-FIFO controllers.
use embedded_hal::i2c::Operation;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Step {
    pub op: usize,
    pub byte: usize,
    pub read: bool,
    pub restart: bool,
    pub stop: bool,
}

pub(crate) struct Planner {
    op: usize,
    byte: usize,
    last: Option<usize>,
    previous_read: Option<bool>,
}
impl Planner {
    pub fn new(operations: &[Operation<'_>]) -> Self {
        let last = operations.iter().rposition(|op| match op {
            Operation::Read(data) => !data.is_empty(),
            Operation::Write(data) => !data.is_empty(),
        });
        Self { op: 0, byte: 0, last, previous_read: None }
    }
    pub fn is_empty(&self) -> bool {
        self.last.is_none()
    }
    pub fn next(&mut self, operations: &[Operation<'_>]) -> Option<Step> {
        let last = self.last?;
        while self.op <= last {
            let (read, len) = match &operations[self.op] {
                Operation::Read(data) => (true, data.len()),
                Operation::Write(data) => (false, data.len()),
            };
            if self.byte == len {
                self.op += 1;
                self.byte = 0;
                continue;
            }
            let step = Step {
                op: self.op,
                byte: self.byte,
                read,
                restart: self.byte == 0 && self.previous_read.is_some_and(|previous| previous != read),
                stop: self.op == last && self.byte + 1 == len,
            };
            self.previous_read = Some(read);
            self.byte += 1;
            return Some(step);
        }
        None
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn skips_empty_operations_and_marks_only_direction_changes_and_last_byte() {
        let mut empty = [];
        let mut read = [0; 2];
        let ops =
            [Operation::Write(&[1, 2]), Operation::Read(&mut empty), Operation::Read(&mut read), Operation::Write(&[])];
        let mut planner = Planner::new(&ops);
        let mut steps = std::vec::Vec::new();
        while let Some(step) = planner.next(&ops) {
            steps.push(step);
        }
        assert_eq!(
            steps,
            [
                Step { op: 0, byte: 0, read: false, restart: false, stop: false },
                Step { op: 0, byte: 1, read: false, restart: false, stop: false },
                Step { op: 2, byte: 0, read: true, restart: true, stop: false },
                Step { op: 2, byte: 1, read: true, restart: false, stop: true },
            ]
        );
        assert!(!planner.is_empty());
        let mut no_data = [];
        let empty_ops = [Operation::Write(&[]), Operation::Read(&mut no_data)];
        let mut empty_planner = Planner::new(&empty_ops);
        assert!(empty_planner.is_empty());
        assert_eq!(empty_planner.next(&empty_ops), None);
    }
}
