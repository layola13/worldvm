# Physics 主入口统一计划

更新日期：2026-05-01

## 1. 背景

当前 Zig 测试矩阵已经覆盖全部 `src/*.zig`，可以开始进入 P1：统一 `tick_engine.zig`、`physics_world.zig`、`physics_kernel.zig` 的职责边界。

现状判断：

- `tick_engine.zig` 是事实上的权威运行入口：CLI、chapter tests、bench、vm_hook、rewind、mind sandbox 都主要通过它驱动。
- `physics_world.zig` 已经具备 world-level orchestrator 形态，但仍有兼容路径与部分重复 step 逻辑。
- `physics_kernel.zig` 是巨大底层能力集合，包含很多纯函数、约束求解、事件构造、排序、调试、回归测试。
- `contact_response.zig`、`collision_event.zig`、`break_response.zig` 已拆出一部分共享逻辑，应继续作为去重方向。

目标不是一次性大重构，而是在保持 `zig build test` 绿色的前提下逐步收敛。

## 2. 当前入口关系

### 2.1 当前事实入口

主要外部调用集中到 `tick_engine.zig`：

- `src/main.zig`
- `src/bench.zig`
- `src/vm_hook.zig`
- `src/rewind.zig`
- `src/mind.zig`
- `src/physics_tests.zig`
- `src/physics_test.zig`
- `src/chapter*_*.zig`
- `src/chapter_test_support.zig`

这说明外部 API 语义目前实际由 `TickEngine` 定义。

### 2.2 PhysicsWorld 当前角色

`physics_world.zig` 当前提供：

- `PhysicsWorld`：聚合 `Scene1024`、entities、joints、bus、pending event queues、broadphase pairs。
- `StepConfig` / `StepResult`：step 配置与结果结构。
- `preStep` / `broadPhase` / `solveConstraints` / `integrate` / `handleEvents` / `recordSnapshot` / `stepPhysicsConfigured`。
- `getQueryWorldView`。

它已经是合理的 world orchestrator 候选，但需要避免与 `tick_engine` 并行维护两套 motion/contact/event 逻辑。

### 2.3 PhysicsKernel 当前角色

`physics_kernel.zig` 当前承担：

- broadphase pair collection。
- contact manifold/classification/material/medium/body-type/condition。
- constraint row、priority order、iteration budget。
- continuous physics、sleep/wake、joint constraints。
- collision/sound/particle/deformation/break event enqueue/publish helper。
- break/wake/support/debris/settle 等大量底层工具。

它应该继续向“无状态或低状态纯能力层”收敛，避免成为另一个 world runtime。

## 3. 目标职责边界

### 3.1 `physics_kernel.zig`

定位：底层算法与可复用求解能力。

应该保留：

- 纯函数或接近纯函数的几何、接触、CCD、约束、排序、预算、分类逻辑。
- 对 `Scene1024` 的局部查询/求解 helper，但不持有长期 world 状态。
- pending event 的构造/发布 helper。
- sleep、wake、break、support 查询等底层 primitive。

应该避免：

- 拥有权威 tick 生命周期。
- 持有长期 runtime state。
- 直接决定 CLI/chapter/vm_hook 行为。

### 3.2 `physics_world.zig`

定位：物理世界 orchestrator。

目标职责：

- 持有 world-level step 上下文：scene、entities、joints、bus、pending queues、broadphase pairs。
- 明确暴露 pipeline：`preStep -> broadPhase -> detect/classify/solve -> integrate -> handleEvents -> recordSnapshot`。
- 提供 `StepConfig` 和 `StepResult`，作为未来统一 step 结果协议。
- 复用 `physics_kernel`，不复制 kernel 算法。

应该避免：

- 与 `tick_engine` 同时实现一套不一致的移动/碰撞/事件路径。
- 内嵌 chapter/CLI 特化逻辑。

### 3.3 `tick_engine.zig`

定位：权威 tick runtime 与兼容外部入口。

短期职责：

- 继续作为 CLI、chapter、vm_hook、rewind、mind 的稳定外部入口。
- 保留 intent pipeline：`gather -> speculate -> resolve -> commit`。
- 负责 trace、determinism flags、force field、bus sync、chapter 兼容。

中期目标：

- 将可下沉的 motion/contact/event 逻辑迁移到 `physics_world` / `physics_kernel`。
- `TickEngine` 调用 `PhysicsWorld.stepPhysicsConfigured`，再包装 trace/determinism/intent 兼容结果。
- 减少 `tick_engine` 与 `physics_world` 对同一行为的重复实现。

## 4. 分阶段重构计划

### Phase 1：只加桥，不改行为

目标：建立统一上下文，不触碰现有行为。

当前状态：桥接基础已完成，首批动态行为已对齐。已在 `tick_engine.zig` 侧新增桥接 helper 与测试专用 compat step，避免 `physics_world.zig` 反向 import `tick_engine.zig` 造成循环依赖。

任务：

1. 已完成：新增 `makePhysicsWorldView(engine)`，从 `TickEngine` 构造 `PhysicsWorld` 兼容视图。
2. 已完成：新增 `makePhysicsWorldStepConfig(engine, apply_continuous_physics)`，从 `TickEngine` 生成 `StepConfig`。
3. 已完成：新增 `stepViaPhysicsWorldCompat(engine, apply_continuous_physics)`，默认不启用，只在测试中对比。
4. 已完成：空世界下传统 `stepTickResult` 与 compat world step 的关键结果一致。
5. 已完成：新增只读桥接测试，验证 view/config 映射，不改变生产 step 路径。
6. 已完成：静态非空世界下传统 `stepTickResult` 与 compat world step 的关键结果一致。
7. 已完成：单个 lateral mover 对比测试已从差异记录升级为一致性测试。
8. 已完成：floor block 与 lateral wall block 的非空场景对比测试已加入并通过。
9. 已完成：ceiling block / upward sweep 的非空场景对比测试已加入并通过。
10. 已完成：stationary discrete fall 兼容逻辑已下沉为 `physics_kernel.planStationaryDiscreteFall` helper。
11. 已完成：blocked fall collision event 与 fragile lateral break 的 compat 对比测试已加入并通过。
12. 已完成：joint break event 的 compat 对比测试已加入并通过。
13. 已完成：compat path 已按 `stepTickResult` 顺序生成 fixed trace/contact telemetry 输出，blocked fall collision 场景已对齐 trace count 与 contact telemetry count。
14. 已完成：新增 compat 对比 helper，收敛重复的 StepResult/stability/instance kinematics 断言。
15. 已完成：空世界、静态非空、single lateral mover、floor block、lateral wall、upward ceiling 场景已对齐 snapshot/hash 输出。
16. 已完成：blocked fall collision、fragile lateral break、joint break event 场景已对齐 snapshot/hash 输出。
17. 已完成：blocked fall collision、fragile lateral break、joint break event 场景已对齐 trace/contact telemetry 明细内容。

已修复差异：

- `TickEngine PhysicsWorld compat step matches single lateral mover tick result`：compat path 现在允许 tick-engine authority 下的静止离散下落。
- `TickEngine PhysicsWorld compat step matches falling body blocked by floor`：compat path 对齐 blocked stationary fall 的当前位置保持语义。
- `TickEngine PhysicsWorld compat step matches lateral wall block`：compat config 在 `apply_continuous_physics = true` 时不再额外运行 pre-motion constraint，避免 broadphase pair count 与 `stepTickResult` 分叉。
- `TickEngine PhysicsWorld compat step matches upward ceiling block`：upward sweep / ceiling block 场景已与 `stepTickResult` 对齐。
- `TickEngine PhysicsWorld compat step matches blocked fall collision events`：collision event bus 消息数量、entity 分布与 `event_count` 已对齐。
- `TickEngine PhysicsWorld compat step matches fragile lateral break`：break-on-impact 的 broken state、速度清零与 `event_count` 已对齐。
- tick-engine authority 下 compat `changed` 语义已收敛为 moved/topology changed，不再把纯约束接触事件误报为 changed。
- `TickEngine PhysicsWorld compat step matches joint break event`：joint enabled 状态、pending joint break queue、bus 消息数量与 `event_count` 已对齐。
- compat helper 在 `apply_continuous_physics = false` 的 constraint-only 对比中保持 `changed = false`，匹配 `stepTickResult` 的离散 intent 语义。
- compat helper 现在延后 publish pending queues，先生成 trace/contact telemetry，再调用 `finishWorldStep`，顺序与 `stepTickResult` 一致。
- `PhysicsWorld.StepConfig.finish_world_step` 用于保留默认 `PhysicsWorld` 行为，同时允许 TickEngine compat path 接管 finish/publish 顺序。
- 基础无事件、简单 motion、collision event、break-on-impact、joint break 场景的 `last_tick_output.snapshot` 与 `last_tick_output.hash` 已通过 helper 对齐。
- collision/break/joint break 场景的 fixed trace event 与 contact telemetry 内容已通过 helper 对齐。

剩余风险：

- compat-only 的 stationary fall 仍由 `PhysicsWorld` 配置开关保护，默认 `PhysicsWorld` 行为不变；后续需要评估是否能成为正式 world stepping 语义。
- 多 tick 连续运行、trace async queue/history、以及更复杂多事件组合还没有桥接对比测试，仍不能直接把生产 `stepTickResult` 替换为 compat path。

验收：

- `zig build test-fast` 通过。
- `zig build test-full` 通过。
- 不改变 CLI 输出和 chapter tests。

### Phase 2：事件队列与 pending 发布去重

目标：把重复的 collision/sound/particle/deformation/break pending queue 路径统一。

任务：

1. 确认 `collision_event.zig` 为 pending queue 的唯一基础实现。
2. `physics_kernel` 只保留 enqueue/publish helper，内部直接委托 `collision_event`。
3. `tick_engine` 与 `physics_world` 都通过同一 helper 清理/发布 pending events。
4. 删除或收敛重复的 queue clear/publish wrapper。

验收：

- `src/collision_event.zig` tests 继续通过。
- `PhysicsWorld handleEvents` tests 继续通过。
- `TickEngine blocked fall broadcasts collision events for both participants` 继续通过。

### Phase 3：motion sweep 逻辑下沉

目标：统一 blocked fall、lateral block、ceiling block、diagonal slide。

任务：

1. 抽出 `MotionSweepPlan` / `MotionSweepResult`。
2. 将 `sweepMotionAlongAxis`、blocked contact handling、break-on-impact 组合成一个 kernel/world helper。
3. `tick_engine.processDynamicBodyMotion` 与 `physics_world.integrate` 逐步改用同一 helper。
4. 保持 `TickEngine` intent pipeline 的外部行为不变。

验收：

- `PhysicsWorld step blocks falling body on floor`。
- `PhysicsWorld step blocks lateral motion into wall`。
- `PhysicsWorld step blocks upward motion on ceiling`。
- `TickEngine stepTick does not tunnel through thin lateral wall`。
- `TickEngine stepTick blocks upward motion on ceiling`。

### Phase 4：StepResult 统一

目标：让 `TickEngine` 和 `PhysicsWorld` 返回同一类 step 结果。

任务：

1. 扩展或复用 `physics_world.StepResult`。
2. 将 `tick_engine.stepTickResult` 的核心字段映射到 `StepResult`。
3. 保留 trace-specific 输出，但不要让 trace 字段污染底层 world result。
4. 明确 `state_hash`、`determinism_flags` 的生成位置。

验收：

- `tick_engine` trace/determinism tests 继续通过。
- `physics_world` snapshot/hash tests 继续通过。

### Phase 5：切换权威实现

目标：让 `TickEngine` 外部入口保持不变，但内部物理 step 使用 `PhysicsWorld` orchestrator。

任务：

1. 在 feature flag 或 config 下启用 `PhysicsWorld` step。
2. 对 chapter tests 做逐章对比。
3. 移除旧 compat 路径中的重复代码。
4. 文档更新：`TickEngine` 是 runtime facade，`PhysicsWorld` 是 world orchestrator，`PhysicsKernel` 是 algorithm layer。

验收：

- `zig build test-full` 通过。
- CLI smoke 通过。
- vm_hook tests 通过。
- rewind/network determinism tests 通过。

## 5. 不建议立即做的事

- 不建议直接删除 `tick_engine` 里的旧 motion path。
- 不建议一次性把 `physics_world` 设为唯一入口。
- 不建议在没有对比测试前移动 broadphase/contact/event 代码。
- 不建议继续扩大 `physics_kernel.zig` 的长期状态职责。

## 6. 推荐下一步小任务

下一轮可以从最小安全任务开始：

1. 继续对比多 tick 连续运行、trace async queue/history 与复杂多事件组合。
2. 继续抽出通用场景构建 helper，减少 compat 对比测试的重复样板。
3. 评估 `finish_world_step` 是否应长期保留，或改为更明确的 TickEngine finish adapter。
4. 只有上述结果一致后，才考虑让 `TickEngine` 某个内部测试走 `PhysicsWorld` compat path。

## 7. 验证命令

```bash
zig build test-fast
zig build test-full
zig build check-matrix
python3 tools/zig_build_guard.py -- zig build test-fast
python3 tools/zig_build_guard.py --limit 1GiB -- zig build test-full
zig test src/physics_world.zig
zig test src/tick_engine.zig
zig test src/physics_kernel.zig
```
