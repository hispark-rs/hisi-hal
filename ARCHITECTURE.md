# hisi-hal 架构

HAL 独立开发、测试、版本和发布，父仓负责跨仓集成与实板验收。
驱动通过当前芯片 PAC 访问硬件；SoC 层提供实例、引脚、IRQ、时钟和请求路由。
相同 IP 的状态机、数据处理、错误和等待逻辑共用；真实寄存器行为差异使用必要后端。

HAL 保留 `embedded-hal`、`embedded-io` 和标准 async trait，以及与执行器无关的
Future/Waker/取消逻辑。HAL 不依赖 `hisi-rtos` 或 Embassy 队列、时间驱动、执行器。
`hisi-rtos → hisi-hal → PAC`；应用和平台初始化负责组装、时钟及资源交接。
硬件 IPC 属于 HAL，厂商协议、控制核服务、镜像布局和板卡接线属于上层组件。

接口契约以 API 文档为准；实验迁移见 MIGRATION.md；芯片行为和历史证据见 docs/。
