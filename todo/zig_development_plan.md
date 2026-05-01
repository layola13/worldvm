# Zig 优先开发计划

更新日期：2026-05-01

## 1. 当前结论

本工程后续开发应继续以 Zig 内核为主线，Python 代码只作为低优先级参考、辅助脚本或历史兼容层。

当前 Zig 侧已经完成一次关键工程收敛：

- `zig build test` 已覆盖全部 `src/*.zig` 文件。
- 测试矩阵覆盖 `105/105` 个 Zig 源文件，无遗漏、无重复、无不存在条目。
- `zig build check-matrix` 已接入，用于阻止新增 Zig 文件漏进测试矩阵。
- `zig build test-fast` / `zig build test-full` 已接入，用于区分核心链路快速验证与全量提交前验证。
- 热缓存全量测试耗时约 `11s`；加入 `main.zig` / `vm_hook.zig` 后最终复测约 `28.5s`。
- CLI smoke 已通过：`run`、`dump`、`bench`、管道 BrokenPipe 场景均可运行。
- `zig-out` 已从 Git 索引移除并由 `.gitignore` 忽略，避免二进制 diff 混入源码提交。

这意味着当前可以把 `zig build test` 作为 Zig 主线健康检查入口。

## 2. 已稳定纳入的底层链路

### 2.1 体素与场景基础

覆盖文件：

- `src/address.zig`
- `src/entity16.zig`
- `src/scene32.zig`
- `src/scene1024.zig`
- `src/scenarios.zig`
- `src/sdf.zig`
- `src/renderer.zig`

当前保障：

- 地址编码/解码、page/local index 位布局。
- `Entity16` 4KB 固定布局、16³ voxel bit mapping、box/hollow/prototype 契约。
- `Scene32` occupancy 与 focus spherical radius。
- `Scene1024` global voxel addressing、instance capacity、rebuild occupancy、LRU page 语义。
- built-in scenario 的 prototype 与 instance placement 基础契约。
- SDF primitive / boolean operation 基础 signed distance 契约。
- ASCII renderer 的 top-view 与 instance 输出契约。

### 2.2 Query 与几何查询

覆盖文件：

- `src/query_types.zig`
- `src/query_world.zig`
- `src/query_raycast.zig`
- `src/query_overlap.zig`
- `src/query_sweep.zig`
- `src/query_penetration.zig`
- `src/query_debug.zig`
- `src/query_benchmark.zig`
- `src/query_regression.zig`
- `src/query.zig`
- `src/raycast.zig`

当前保障：

- Query contract version。
- half-open AABB / VoxelBox 语义。
- raycast/sweep/overlap/penetration 的 layer mask、metadata、classification、telemetry。
- negative direction raycast 边界回归。
- environment 与 dynamic instance 分层计数。
- query cache / scratch / batch / async job 基础行为。

### 2.3 物理核心与世界步进

覆盖文件：

- `src/physics.zig`
- `src/collision.zig`
- `src/ccd.zig`
- `src/contact_response.zig`
- `src/physics_kernel.zig`
- `src/physics_world.zig`
- `src/tick_engine.zig`
- `src/sleep_response.zig`
- `src/material_pairing.zig`
- `src/terrain.zig`
- `src/collision_event.zig`
- `src/break_response.zig`

当前保障：

- 重力、速度 clamp、entity local AABB、swept AABB、half-open AABB hit。
- contact manifold、contact classification、constraint row debug、contact response。
- CCD TOE、iterative TOI、thin wall、angular/rotating CCD、trigger CCD、precision/stability plan。
- `PhysicsWorld` broadphase、contact constraints、environment collision、event broadcast、KCC coordinator。
- `TickEngine` step、force fields、trace telemetry、determinism flags。
- sleep/wake/island 基础规则。
- terrain/material pairing 与 medium classification。
- pending collision/sound/particle/deformation/break/joint-break queue 的去重、过滤、发布契约。
- break impact、fragile break、debris spawning、幂等 break 行为。

### 2.4 高阶物理系统

覆盖文件：

- `src/kcc.zig`
- `src/joint.zig`
- `src/rewind.zig`
- `src/ragdoll.zig`
- `src/ballistics.zig`
- `src/destruction.zig`
- `src/soft_tissue.zig`
- `src/fluid.zig`
- `src/particle.zig`
- `src/softbody.zig`

当前保障：

- KCC move/jump/crouch/wall slide/predictive avoidance。
- joints、break/fatigue/temperature/stress/debug rows。
- rewind snapshot、branch、merge、GC、network packet。
- ragdoll、ballistics、destruction、soft tissue 的基础行为。
- fluid/particle/softbody 已纳入 Zig 测试矩阵。

### 2.5 车辆、AI、环境与接口

覆盖文件：

- `src/vehicle.zig`
- `src/tire.zig`
- `src/suspension.zig`
- `src/drivetrain.zig`
- `src/braking.zig`
- `src/aerodynamics.zig`
- `src/ai_traffic.zig`
- `src/network.zig`
- `src/crash_defense.zig`
- `src/sensors.zig`
- `src/weather.zig`
- `src/disasters.zig`
- `src/safety.zig`
- `src/planner.zig`
- `src/sports.zig`
- `src/biomechanics.zig`
- `src/mind.zig`
- `src/bus.zig`
- `src/vm_hook.zig`
- `src/main.zig`

当前保障：

- 车辆底盘链路：tire/suspension/drivetrain/braking/aero。
- AI traffic 与 vehicle 双向同步。
- crash defense、network、sensors、weather/disasters/safety/planner。
- TriWorld bus、AffectSystem、ShadowSandbox。
- VM hook exports 与 CLI smoke。

## 3. 近期开发原则

1. 优先保持 Zig 主线绿色：任何改动后先跑相关 `zig test src/<file>.zig`，再跑 `zig build test`。
2. 先改底层契约，再改高层行为：地址、体素、query、collision、CCD、contact、tick/world 是优先级最高的根。
3. 不再新增未纳入矩阵的 Zig 文件：新增 `src/*.zig` 时必须同步加入 `build.zig` 的 `test_files`。
4. Python 只做辅助：不要让 Zig 内核依赖 Python runtime。
5. 避免二进制产物入 diff：`zig-out/` 已被忽略，若未来新增 release 产物需单独走发布流程。
6. 注意 `.zig-cache`：曾膨胀到 `13G`，磁盘紧张时可清理 `.zig-cache`。

## 4. 下一阶段优先级

### P0：保持测试矩阵与工程卫生

目标：让 Zig 主线可持续开发。

任务：

- 已完成：`tools/check_zig_test_matrix.py` 与 `zig build check-matrix` 会验证所有 `src/*.zig` 都在矩阵中。
- 已完成：`zig build test-fast` / `zig build test-full` 分层已接入。
  - `test-fast`：底层核心链路，适合频繁运行。
  - `test-full`：完整 `src/*.zig` 矩阵，适合提交前运行。
- 已完成：`zig-out` 产物从 Git 索引移除，仅作为本地构建输出保留。
- 已完成：`tools/clean_zig_cache.py` 可安全清理 `.zig-cache` / `zig-cache`，必要时可带 `--include-zig-out`。

### P1：统一物理主入口

目标：明确 `tick_engine`、`physics_world`、`physics_kernel` 的职责边界。

详细计划：见 `todo/physics_entrypoint_unification_plan.md`。

建议方向：

- `physics_kernel.zig`：纯函数/底层求解能力。
- `physics_world.zig`：world-level orchestration 与 broadphase/contact pipeline。
- `tick_engine.zig`：tick intent、trace、event、integration 的权威入口。

待完成：

- 梳理重复逻辑，减少同一行为在 `tick_engine` 与 `physics_world` 的双实现。
- 给 contact pipeline 写一份 detect -> classify -> solve -> emit 的接口说明。
- 明确 `PhysicsWorld` 是 orchestrator 还是未来唯一 world step 入口。

### P2：Query 与 CCD 精度升级

目标：把目前能用的 cast/sweep 继续向真实体积查询推进。

重点：

- `sphereCast` / `boxCast` / `capsuleCast` 的采样与精确性。
- negative direction 与 voxel boundary 的一致性。
- dynamic instance broadphase pruning 的性能与正确性。
- rotating box / angular CCD 与 world step 的整合。

验收：

- 保持 query contract tests 通过。
- 新增复杂边界测试：斜向 sweep、薄墙、多体同时命中、初始重叠。

### P3：接触求解与堆叠稳定

目标：从“可以阻挡/去穿透”提升到“稳定多点接触”。

重点：

- contact manifold 质量。
- 多点接触稳定排序。
- normal/friction/restitution row 的一致求解。
- sleep island 与 resting stack 的长期稳定性。

验收：

- 多层 stack 长时间不抖动。
- 斜面/边缘/角落接触行为可解释。
- 事件广播不重复、不漏发。

### P4：高阶系统真实度

目标：在底层稳定后，提高高级系统保真度。

模块优先级：

1. `vehicle.zig` 与底盘模块：tire/suspension/drivetrain/braking/aero。
2. `kcc.zig`：step offset、slope limit、snap to ground、ceiling slide。
3. `fluid.zig` / `particle.zig` / `softbody.zig`：从示意/基础模型转向更可解释的物理近似。
4. `ragdoll.zig` / `soft_tissue.zig` / `biomechanics.zig`：统一 joint/contact/soft constraints。
5. `network.zig` / `rewind.zig`：回放确定性、snapshot 压缩、分支合并策略。

### P5：CLI 与调试体验

目标：让 Zig 内核更容易观察和调试。

建议：

- `dump --view top|front|side|slice`。
- `dump --query-stats`。
- `bench --json`。
- `trace --scenario ... --ticks ...`。
- `sim-check --domain physics|query|vehicle --case ...`。

## 5. 推荐日常命令

```bash
# 单文件快速验证
zig test src/query_world.zig
zig test src/physics_world.zig
zig test src/tick_engine.zig

# 全量 Zig 主线验证
zig build test

# 核心链路快速验证
zig build test-fast

# 带 Zig cache 保护的快速验证；默认 cache 超过 3GiB 才清理
python3 tools/zig_build_guard.py -- zig build test-fast

# 自定义 cache 阈值后运行全量验证
python3 tools/zig_build_guard.py --limit 1GiB -- zig build test-full

# 显式全量矩阵验证
zig build test-full

# 只检查 Zig 测试矩阵是否覆盖所有 src/*.zig
zig build check-matrix

# 查看 Zig 缓存可释放空间，不删除
python3 tools/clean_zig_cache.py --dry-run

# 只在 cache 超过阈值时计划清理
python3 tools/clean_zig_cache.py --dry-run --if-over 3GiB

# 清理 Zig 缓存；默认不删除 zig-out
python3 tools/clean_zig_cache.py

# 如需同时清理 zig-out 构建产物
python3 tools/clean_zig_cache.py --include-zig-out

# CLI smoke
zig build run -- --ticks 1
zig build run -- --ticks 0
zig build run -- dump --scenario bounce_test
zig build run -- bench --scenario apple_table
zig build run -- --ticks 0 | head -20

# 检查构建产物是否仍被 Git 跟踪；正常应无输出
git status --short zig-out
```

## 6. 提交分组建议

建议不要把所有改动压成一个难审的大提交。可按以下顺序拆分：

1. 测试矩阵全覆盖：`build.zig` 与新增底层契约测试。
2. Query / Scene / Entity / Address 回归测试。
3. Physics / Contact / CCD / World / Tick 回归测试。
4. 高阶模块接入矩阵：KCC、joint、rewind、ragdoll、vehicle、AI、network 等。
5. CLI BrokenPipe 与 smoke 相关修复。
6. 工程清理：`.gitignore`、删除 `src/vehicle.zig.bak`、清理生成物。
7. 新增模块或历史新增文件：`fluid.zig`、`particle.zig`、`softbody.zig` 如需要可单独审阅。

## 7. 当前风险

- `zig build test` 当前覆盖面很广，但某些文件相互 import 导致重复运行大量测试；如果后续耗时上升，需要分层测试。
- 若未来重新跟踪 `zig-out` 或其他 release 二进制，CLI smoke 后仍可能出现误 diff，应保持构建产物不进源码提交。
- `.zig-cache` 可能快速膨胀，占用磁盘。
- `physics.md` 目标远大于当前实现，高级物理系统大多仍是“有骨架/有基础测试”，不能误判为高保真实现完成。
- Python 与 Zig 两套实现可能语义漂移，后续应明确 Zig 为权威实现。

## 8. 最近验收状态

最近一次关键验收：

- `zig build test`：通过。
- `zig build test-fast`：通过。
- `zig build test-full`：通过。
- 全部 `src/*.zig`：已纳入 `build.zig` 测试矩阵。
- `zig build check-matrix`：通过，覆盖 `105/105` 个 Zig 源文件。
- `tools/clean_zig_cache.py --dry-run`：通过，可显示预计释放空间。
- CLI smoke：通过。
- `zig-out` tracked artifact：已从索引移除，后续构建不会产生二进制源码 diff。
