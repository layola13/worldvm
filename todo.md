# Zig 版 World VM v1（3D 内核先行）实施计划

## Summary
- 语言与平台：`Zig 0.14.1`，首版仅 `linux x64`。
- 交付形态：单一 CLI 可执行文件（无第三方依赖）。
- 范围：先实现 3D 核心（Entity16/Scene32/Tick/碰撞/重力/流体简化/LOD/ASCII 观测），不做自然语言 Parser 主链路。
- 外挂 LLM：仅做接口占位与本地验证链，不接真实外部 API。
- 首版资产：编译期内置样例实体（苹果/桌子/锤子/玻璃/水）。

## Implementation Changes
- **工程骨架与构建**
  - 建立 `build.zig` + `src/` + `scenarios/` 三层结构。
  - 构建模式固定：`Debug`、`ReleaseSafe`、`ReleaseSmall`；发布使用 `ReleaseSmall + strip`。
  - CLI 子命令固定为：`run`、`bench`、`dump`。

- **3D 数据模型（固定布局）**
  - `Entity16`：4KB 定长（含 `u64[64]` 拓扑、物理字段、颜色/材质、状态标记）。
  - `Scene32`：`u64[512]` 占位图 + 实例槽（建议 128）+ 焦点参数 + Tick 状态。
  - `Instance`：`entity_id + transform + motion_state + material_state + flags`。
  - 坐标语义固定：`16³` 实体局部坐标映射到 `32³` 场景局部坐标。

- **执行内核（确定性四步 Tick）**
  - 固定执行顺序：`Gather -> Speculate -> Resolve -> Commit`。
  - 固定算子集（v1）：`SPAWN_INSTANCE`、`DESPAWN_INSTANCE`、`MOVE_DELTA`、`ROTATE_LUT`、`TEST_COLLISION`、`APPLY_FORCE_FALL`、`FLOW_STEP`、`BREAK`、`COMMIT_DELTA`。
  - 碰撞固定两阶段：Broad-phase（AABB）+ Narrow-phase（位图按位与）。
  - 稳定态判定固定：无有效意图 + 无待处理冲突 + 无高优先级中断。

- **LOD 与空间调度**
  - 焦点区固定 `16³`，外围区 `32³ - 16³`。
  - 时间分层固定：焦点 `tick_rate=1`，外围默认 `tick_rate=8`。
  - 升降级规则固定：距离/速度/威胁触发唤醒；连续稳定 K Tick 后降级；加入滞回避免抖动。

- **ASCII 观测与调试**
  - `dump --view top|front|side|slice` 输出固定格式帧。
  - `run --scenario apple_table|hammer_glass|water_flow --ticks N`。
  - 帧日志包含：`tick_id`、`executed_ops`、`collision_pairs`、`delta_summary`、`reason_code`。

- **外挂 LLM 接口占位**
  - 定义 `ExternalLexiconAdapter` 接口（query/response schema）。
  - 定义本地 `Verifier`（schema/rule/conflict/source trust）流程。
  - v1 默认 adapter 为 stub（返回未接入），但全链路可编译可测试。

## Public Interfaces / Types
- `pub const Entity16`
- `pub const Instance`
- `pub const Scene32`
- `pub const TickEngine`
- `pub const Operator`（枚举）
- `pub const ExternalLexiconAdapter`
- `pub fn initScene(...)`
- `pub fn injectInstance(...)`
- `pub fn stepTick(...)`
- `pub fn runTicks(...)`
- `pub fn renderAscii(...)`
- `pub fn runScenario(...)`
- `pub fn benchScenario(...)`

## Test Plan & Acceptance
- **单元测试**
  - 位图索引映射（16³/32³）一致性。
  - 变换与旋转 LUT 正确性。
  - AABB + 位图碰撞判定正确性。
  - `FORCE_FALL` 与 `FLOW_STEP` 规则正确性。
- **集成测试**
  - `apple_table`：苹果下落并稳定在桌面。
  - `hammer_glass`：碰撞触发破碎状态与碎片生成。
  - `water_flow`：下落优先、侧向扩散次之。
  - LOD：外围低频更新且关键对象能被唤醒到焦点。
- **性能与构建验收**
  - 轻场景 Tick `p95 < 200us`。
  - 发布二进制（ReleaseSmall + strip）`< 5MB`。
  - Debug 全量编译 `< 15s`，增量编译 `< 3s`。
  - 运行时零外部依赖（除系统基础运行环境）。

## Assumptions / Defaults Locked
- 首版只做 Win x64；跨平台放到 v1.1。
- 首版不接自然语言主链路（Parser/IR 在后续阶段接入）。
- 首版资产内置，不做 ROM 文件加载器。
- 外挂 LLM 仅接口占位，不接网络 API。
- 单可执行交付优先，库化输出延后。
