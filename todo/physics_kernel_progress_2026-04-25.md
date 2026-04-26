# Physics Kernel Bottom-Layer Progress

日期：2026-04-25
范围：`src/physics_kernel.zig`
主题：底层统一约束求解器、预测性接入、row 协议收口

## 当前结论

这轮工作已经把物理底层从“joint/contact/environment 各自一套求解路径”推进到了“共享 row 调度 + 共享 row state/equation + 共享 prepared channel + 共享 apply step + 部分共享 postprocess/finalize”。

当前状态可以描述为：

1. 已经有统一的 row solver 外壳。
2. contact 和 environment 大部分核心执行已经不再依赖旧的专用 apply 模板。
3. 预测性已经进入底层约束方程和优先级，而不是停留在外层模块。
4. 还没有到“通用 Jacobian / lambda 累积求解器”阶段。
5. 还没有形成真正统一的 `prepare -> plan -> solve -> postprocess -> finalize` 完整泛型协议，当前只是大部分关键层已经收口。
6. `ConstraintApplyPrimitive` 已经出现，说明底层不再只是 helper 并列，而是开始形成可承载通用 row primitive 的执行元数据壳。

## 已完成进度

### 1. 统一 row 调度已经成型

已存在统一约束行类型：

- `ConstraintRowKind`
  - `joint_anchor`
  - `joint_limit`
  - `joint_drive`
  - `contact_normal`
  - `contact_friction`
  - `environment`

已存在统一 row 结构和状态：

- `ConstraintRow`
- `ConstraintRowState`
- `ConstraintRowEquation`
- `ConstraintRowExecResult`

统一求解主循环已在：

- `solveConstraintBlock(...)`

这意味着当前 solver 已经不是按 subsystem 外层分发，而是按 row 统一迭代执行。

同时，row 构建层也开始出现真正的 builder 形态，而不再只是 append helper：

- `buildDirectionalRow(...)`
- `buildDirectionalNormalRow(...)`
- `buildDirectionalTangentRow(...)`
- `buildEnvironmentDirectionalRow(...)`

### 2. joint/contact/environment 都已接入统一 row 执行

joint 行执行：

- `solveJointAnchorRow(...)`
- `solveJointLimitRow(...)`
- `solveJointDriveRow(...)`

contact 行执行：

- `solveContactNormalRow(...)`
- `solveContactFrictionRow(...)`

environment 行执行：

- `solveEnvironmentRow(...)`

### 3. row state / warm-start / equation 协议已经统一

当前已统一复用：

- `constraintRowWarmImpulse(...)`
- `constraintRowSignedCorrection(...)`
- `constraintRowPlannedSignedCorrection(...)`
- `constraintRowSolveMagnitude(...)`
- `finalizeConstraintRowResult(...)`

这层很关键，因为它意味着不同 subsystem 已经共享：

- impulse retention
- bias 注入
- solve magnitude shaping
- residual 改善判断

### 4. contact 批处理旧路径已大幅收口

原来 `solveContactConstraintsForPairs(...)` 内部自己拼一整套：

- manifold 准备
- 材质响应
- 位置纠正
- 速度反弹
- 摩擦
- resting/wake

现在已经改成复用 shared prepared/helper 路径，核心收益：

1. contact 批处理路径和 row 路径不再各写一套 contact solve 主体。
2. 旧 contact 专用 apply helper 已经大幅减少。
3. contact 逻辑不再散落在 batch path / row path / row build path 三处重复维护。

### 5. environment 批处理旧路径也已开始收口

environment 现在已有：

- `PreparedEnvironmentConstraint`
- `prepareEnvironmentConstraint(...)`
- `applyEnvironmentResolvedMotion(...)`

这意味着：

1. environment 的 penetration 查询与 move 向量准备不再在 batch path / row path 各写一份。
2. environment 的“位移 + 表面响应”已抽成共享后处理 helper。

### 6. 预测性已经真正进入底层约束核

已引入：

- `const prediction = @import("prediction.zig");`

不是只做外层感知，而是直接进入 contact/environment 的底层求解输入。

当前预测接入点：

1. `prepareContactConstraintPair(...)`
   - 基于短时预测位置估算 future overlap
   - 把预测深度写入 `PreparedDirectionalConstraint`
   - 通过 row equation shaping 提前增大 bias / impulse limit

2. `measurePredictiveEnvironmentDepth(...)`
   - 估算 instance 短时未来环境穿透深度

3. `computeEnvironmentSolvePriorityMagnitude(...)`
   - 已把 predicted depth 纳入环境约束优先级

4. environment row 构建
   - 预测深度已进入 equation shaping

结论：

当前底层已经具备“预计下一小段时间会更糟的约束，优先求解、提前施压”的能力。

这和之前用户强调的“可预测、像人类一样先预估后几秒风险再可控处理”是同方向的，只是目前还是短时、保守、局部接入，不是完整前瞻控制系统。

### 7. prepared channel 已经开始统一

新增：

- `PreparedDirectionalConstraint`
- `DirectionalRowPlan`
- `PreparedJointLinearConstraint`
- `PreparedJointAngularConstraint`
- `PreparedJointAngularDriveConstraint`
- `PreparedJointDriveConstraint`

当前已经统一到这种表达的内容：

1. contact normal
2. contact tangent
3. environment normal
4. joint anchor linear
5. joint spring linear
6. joint slider limit linear
7. joint hinge limit angular
8. joint hinge drive angular target

统一字段包括：

- `dir_x`
- `dir_y`
- `dir_z`
- `depth`
- `predicted_depth`

这一步的意义非常大，因为它把“方向型约束”的输入数据形状统一了。

对 joint 来说，虽然还没有变成真正泛型 jacobian row，但至少已经不再是 solve 分支里完全裸写测量和计划逻辑。

并且 joint 这边也已经新增更明确的 channel 聚合层：

- `JointPreparedAnchorChannel`
- `JointPreparedLimitChannel`
- `JointPreparedDriveChannel`

它们把此前分散的：

- fixed / ball-socket / hinge / slider anchor linear prepared
- spring anchor linear prepared
- slider limit linear prepared
- hinge limit angular prepared
- slider drive linear prepared
- hinge drive angular prepared

开始收口到统一的 anchor / limit / drive channel 协议，而不再让 `solveJointAnchorRow(...)`、`solveJointLimitRow(...)`、`solveJointDriveRow(...)` 继续各自手写多套 prepared 消费分支。

### 8. shared apply step 已经成型

当前底层共享执行通道有：

- `ConstraintApplyPrimitive`
- `applySingleDisplacementRowStep(...)`
- `applyPairDirectionalDisplacementRowStep(...)`
- `applyPairVelocityImpulseRowStep(...)`
- `applyPairAngularVelocityRowStep(...)`
- `applyPairDirectionalConstraintRowStep(...)`
- `applyPairLinearVelocityDampingStep(...)`

已接入情况：

1. environment warm-start 位移
2. environment solve 位移
3. contact normal warm-start impulse
4. contact normal 位置纠正
5. contact normal 速度冲量
6. contact friction warm-start impulse
7. contact friction solve impulse
8. joint hinge angular velocity damping / motor bias
9. joint anchor / spring directional correction
10. joint slider axis correction
11. joint slider limit linear velocity damping
12. joint hinge angular position correction
13. contact normal velocity impulse
14. contact friction warm-start / solve impulse

也就是说，contact/environment 的大部分 solve-step 已经不再各写专用 apply 模板，而且 joint 的角向/线性底层 apply 也已经开始进入同一套 shared step 家族。

更关键的是：这些 step 现在已经开始通过 `ConstraintApplyPrimitive` 统一承载

- `body_mode`
- `channel`
- `equation`
- `bias_scale`
- `warm_impulse`
- `magnitude_slop`
- `clamp_non_negative`
- `axis_x`
- `axis_y`
- `axis_z`
- `pair_bodies`

当前 `channel` 已经实质覆盖：

- `linear_displacement`
- `linear_directional`
- `linear_velocity`
- `angular_position`
- `angular_velocity`

并且高层 wrapper 已经开始收口到 pair primitive 构造 helper，而不是在 joint/contact 多个调用点继续手工拼 `body_mode/channel/axis/pair_bodies`。

其中 `pair_bodies` 这一步很关键，因为 shared pair step 不再只靠调用点继续显式散传

- `inv_mass_a`
- `inv_mass_b`
- `ratio_a`
- `ratio_b`

而是已经开始把这些 pair body 元数据压进 primitive 本身，再由 shared apply step 优先消费。

进一步说，当前 shared pair step 的主干已经去掉了旧的 fallback mass 形参，`pair_bodies` 已经从“优先消费”推进到“唯一可信来源”。

这说明 Phase 4 所需的“通用 row primitive 外形”已经不再是纯规划，而是开始有真实落地的执行元数据载体。

### 9. contact 专用旧 helper 已清理一批

已删除旧 helper：

- `applyContactPositionCorrection(...)`
- `applyContactVelocityConstraint(...)`
- `applyContactFrictionConstraint(...)`
- `applyContactFrictionWarmStartImpulse(...)`

这说明当前不是“新增共享实现但旧实现还留着”，而是已经开始真正清理旧分叉路径。

### 10. postprocess / finalize 也开始收口

当前已新增并接入：

- `settleAndWakeContactPair(...)`
- `applyEnvironmentResolvedMotion(...)`
- `wakeSingleInstance(...)`
- `finalizePairRowResult(...)`
- `finalizeSingleRowResult(...)`

当前效果：

1. contact 的 `settle + wake` 不再在多个调用点重复写。
2. environment 的 `位移 + surface response` 不再在批处理和 row 路径各写一套。
3. pair / single 的 wake 协议外形已经对齐。
4. joint/contact/environment 的 finalize 开始通过 pair/single 两个共享壳收口，而不是各自直接拼 `wake + measure after + finalizeConstraintRowResult`。

## 当前底层架构图（简化）

### 已基本成型的链路

1. `prepare`
   - `prepareContactConstraintPair(...)`
   - `prepareEnvironmentConstraint(...)`

2. `plan`
   - `buildDirectionalRowPlan(...)`
   - `ConstraintRowEquation`

3. `solve`
   - `applySingleDisplacementRowStep(...)`
   - `applyPairDirectionalDisplacementRowStep(...)`
   - `applyPairVelocityImpulseRowStep(...)`

4. `postprocess`
   - `settleAndWakeContactPair(...)`
   - `applyEnvironmentResolvedMotion(...)`

5. `finalize`
   - `finalizeConstraintRowResult(...)`
   - `finalizeContactRowResult(...)`
   - `finalizeEnvironmentRowResult(...)`
   - `finalizeJointRowResult(...)`

### 仍然没有完全统一的部分

1. joint 的 plan/solve 仍然有较多专用逻辑。
2. friction row 仍是 contact 专有，不是一般化 tangent channel solver。
3. environment 和 contact 的 residual 计算接口还没有完全抽象成统一 directional residual helper。
4. single-body displacement 和 pair-body displacement 还没有提升到同一更高层 channel apply 接口。
5. 没有真正的 jacobian row 数据结构。
6. 没有统一 accumulated lambda / clamp per-row / tangent bundle 管理。

## 测试状态

本轮持续验证命令：

```powershell
zig test -j1 src/physics_kernel.zig
```

当前结果：

- `zig test src/physics_kernel.zig`
- `zig test src/tick_engine.zig`
- `zig test src/physics_world.zig`
- 当前三组都是 `200/200` 全通过

这说明当前底层收口在已有测试覆盖范围内是稳定的。

## 还没有完成的关键底层项

下面这些是“底层仍未完成”的核心，不是表层功能，而是结构性缺口。

### A. 还不是通用 Jacobian / lambda solver

当前虽然已经统一 row 调度和大量 row 通道，但仍然不是完整的通用约束求解器。

缺口表现：

1. 没有通用 jacobian 行表达。
2. 没有显式 lambda 累积与 clamp 规则统一管理。
3. 没有把法向、切向、角向、单体环境位移都抽成统一数学 row。
4. 当前更多还是“统一框架 + 多类专用 row helper”。

但这里也要更新一个重要现状：

1. 虽然还没有完整 jacobian row / accumulated lambda 管理，
2. 但 `ConstraintApplyPrimitive` 已经在 shared step 层落地，
3. 而且它已经具备 `body_mode / channel / axis / pair_bodies` 这些最基础的 row 元信息，
4. shared pair step 也已经开始优先从 primitive 读取 pair body 质量/分配数据，
5. 所以后续推进的起点不再是“从零设计 primitive”，而是“在已有 apply primitive 上继续补 body count / axis bundle / lambda / clamp policy”。

### B. joint 仍然最专用

joint 已经接入 row 框架，但底层仍保留一些专用求解逻辑：

- `applyJointAngularScalarCorrection(...)`
- `applyJointAxisScalarCorrection(...)`
- `applyJointDirectionalConstraint(...)`
- `applyJointAnchorDistanceConstraint(...)`

但这里也要继续更新现状判断：

1. joint 线性 prepared channel 已经落地，并已经进入 anchor / spring / slider limit。
2. joint 角向 prepared channel 也已经起步，并已经进入 hinge limit / hinge drive。
3. joint 的底层 angular / linear apply 已经显著收口到 shared step：
   - hinge angular velocity damping / motor bias
   - hinge angular position correction
   - anchor / spring directional correction
   - slider axis correction
   - slider limit velocity damping
4. 当前 joint 的主要剩余问题，已经进一步收缩为：
   - 仍保留 joint 语义层 solver wrapper，而不是直接 generic row primitive
   - prepared channel 还没有和更通用 jacobian row 数据完全对齐
   - finalize 虽然已开始 pair/single 收口，但 residual 量测仍按 subsystem 分开
5. 但相较上一阶段，joint 的 `anchor` / `limit` / `drive` 已经不再只是“prepared shape 部分统一、solver 消费仍分裂”的状态；现在三类行都已经开始通过共享 channel helper 进入统一消费路径，这使下一步继续把 motor target / angular-position / linear-directional 压进更通用 body-channel 协议时，阻力明显下降。

### C. friction 仍然是 contact 专属逻辑

虽然切向输入已经抽成 `PreparedDirectionalConstraint`，但目前还没有一个真正通用的 tangent row 计划与执行协议。

缺口包括：

1. friction residual/equation 仍是 contact 专用公式。
2. tangent 仍没有独立的统一 row builder helper。
3. 缺少更成熟的 friction cone / tangent bundle 管理。

### D. environment 仍然是“单体位移约束”

目前 environment 被纳入统一 row 框架，但本质仍是单体 depenetration + surface response。

还没完成的点：

1. 没有和 pair constraint 完全统一到更高层 body-channel 模型。
2. 没有显式的 environment tangent row。
3. 没有多接触点环境约束管理。

### E. 预测性仍然是短时、保守接入

当前预测性是：

1. 短时 horizon
2. bias / max impulse / priority shaping
3. 只接到 contact/environment

还没完成的是：

1. 预测性没有进入 joint 约束统一协议。
2. 没有完整的“预测计划 -> 约束目标 -> 求解结果”闭环。
3. 没有把“未来几秒风险窗口”转成多步约束控制。
4. 没有显式 predictive braking / predictive steering / predictive route-level constraint row。

### F. residual / equation / state 还没彻底抽成统一方向型工具层

目前已经开始有：

- `PreparedDirectionalConstraint`
- `DirectionalRowPlan`

但还没有完全统一出：

1. 通用 directional residual builder
2. 通用 tangent residual builder
3. 通用 single-body / pair-body row planner
4. 通用 row postprocess planner

### G. 旧接口虽清了很多，但仍有历史形态残留

虽然 contact 旧 apply helper 已删一批，但还存在一些历史形态：

1. contact / joint / environment 的 residual 量测仍分开函数，尚未进入更统一的 finalize planner。
2. constraint row build 仍然按 subsystem 代码块组织，而不是按 generic row-prep registry 组织。
3. joint 仍保留语义层 wrapper，而不是直接暴露到更通用的 row primitive。

## 下一步计划

下面是建议的执行顺序，按“底层优先、风险最小、收益最大”排序。

### Phase 1：继续把 row 构建协议抽干净

目标：让 `buildConstraintRowsForIsland(...)` 不再散写 contact/environment 的 directional residual/equation 拼装。

具体步骤：

1. 抽 `buildDirectionalNormalRow(...)`
   - 输入：`PreparedDirectionalConstraint`、state、index、kind、base equation
   - 输出：`ConstraintRow`

2. 抽 `buildDirectionalTangentRow(...)`
   - 先服务 contact friction
   - 后续可扩展 environment tangent / vehicle slip / KCC side resolve

3. 抽 single-body directional row builder
   - 服务 environment

4. 抽 pair-body directional row builder
   - 服务 contact normal / friction

预期收益：

1. row 构建阶段从“按 subsystem 拼装”变成“按 row channel 拼装”
2. 为真正 generic jacobian row 继续铺路

当前进度更新：

1. directional row builder 已经起步，不再只有 append helper。
2. `ConstraintRow` 的 priority/base_residual/equation 组合逻辑开始有统一 builder 入口。
3. contact normal / tangent 与 environment row 已经可以通过共享 directional builder 外形表达。
4. `buildConstraintRowsForIsland(...)` 已经开始直接消费 builder 返回值，而不是完全依赖 append helper。
5. contact/environment 的 row 构建入口也已独立成 helper，不再都堆在 island 主循环里。
6. joint 的 row 构建入口也已独立成 helper，当前三大类入口都已从 island 主循环中剥离。
7. island row build 已经开始通过 joint/contact/environment 三个 dispatch 壳调度，而不是直接在主循环里展开所有 builder 细节。
8. island row build 的 dispatch 壳已进一步收口为最小 registry 表，不再把 subsystem -> dispatch 映射散落在主流程里。
9. `ConstraintSubsystem` 已开始真实参与 island row build 顺序，当前会按 joint/contact/environment stress 动态决定 dispatch 次序，而不是固定顺序硬编码。
10. joint row build 已继续细分为 anchor / limit / drive 三个独立 builder，joint wrapper 本身开始从“大函数分支”退化为简单收口层。
11. row build 阶段已经新增共享 `state lookup + optional append` 收口，contact/environment/joint 三类 builder 不再各自重复写查 state 和追加 row 的模板代码。
12. `buildConstraintRow(...)` 已成为 row 构建的统一底层入口，joint 专用 row builder 已经退出，最终 `ConstraintRow` 产出路径开始真正同构。
13. 当前已经出现第一层 `ConstraintRowBuildSpec` 协议：specialized row-prep 可以先产出统一 spec，再由共享 row builder 结合 state 生成最终 `ConstraintRow`。
14. joint row-prep 已经开始通过最小 builder entry 表驱动，anchor / limit / drive 不再只能依赖手写顺序展开。
15. environment 也已经切到“先产出 spec，再统一 append”的路径，和 contact/joint 更接近相同协议。
16. contact row-prep 也已经开始通过最小 builder entry 表驱动，normal / friction 不再是单独写死的双行 append。
17. joint/contact/environment 三条路径的“遍历 builder entry -> append spec”模板也已经各自收口到 runner helper，不再直接散落在 dispatch 函数内部。
18. joint/contact/environment 三条 dispatch 路径的 item iteration 模板也已经抽离，当前 dispatch 壳开始只表达“枚举 island item -> 调用 item handler”。
19. dispatch 壳已经进一步退化成 subsystem runner 包装层，registry 指向的函数现在越来越接近“运行某类 subsystem row-prep”的统一语义，而不是内含细节逻辑。
20. island row build 的 registry 现在已经直接产出按 stress 排序的 dispatch entry 队列，主流程不再先得到 `ConstraintSubsystem` 再做第二次 subsystem -> dispatch 查表。
21. joint/contact/environment 三条 runner 已直接作为 registry dispatch 函数，旧的 `dispatch*RowsForIsland(...)` 包装层已经被移除。
22. registry 也已经从“挂一个 subsystem runner 函数”进一步收口为 item-driven 描述：当前 entry 直接携带 `item_count + dispatch_item + measure_stress`，统一 runner 只负责按 item 枚举。
23. joint/contact/environment 的 spec builder 入口现在也已经对齐成统一的 “context -> ?ConstraintRowBuildSpec” 形态，不再维持三套不同函数原型。
24. 共享 `appendConstraintRowSpecsFromEntries(...)` 已经接管三类 builder entry 表的遍历与 append 模板，builder 层的重复样板进一步减少。
25. island row dispatch registry 现在已经进一步提升为真正的 subsystem descriptor：entry 直接携带 `item_count + init_context + builder_entries + measure_stress`，通用 runner 不再需要知道 joint/contact/environment 的 builder 细节。
26. subsystem stress 测量也已经收口到同一份 `IslandRowBuildContext` 上，排序和执行开始真正围绕统一上下文，而不是继续散传多组 slice/指针参数。
27. builder 内部重复的 `RowSpecBuildContext` 联合分支也已经开始收口到 accessor helper，contact/joint/environment 不再各自反复手写相同的 `switch(ctx.*)` 模板。
28. subsystem context 初始化也已经收口到显式 factory 语义：`init_context` 现在直接返回 `?RowSpecBuildContext`，不再依赖 `bool + out param` 形式的可变输出协议。
29. registry 组装层的 item-count 样板也已被收掉，dispatch entry 不再单独挂 `countIsland*Items(...)`；当前统一由 `subsystem` 驱动 item 计数。
30. registry 组装层的 stress-measure 样板也已被收掉，dispatch entry 不再单独挂 `measure*SubsystemStress(...)`；当前统一由 `subsystem` 驱动 stress 测量。
31. registry 组装层的 context-factory 样板也已被收掉，dispatch entry 不再单独挂 `init*RowSpecContext(...)`；当前统一由 `subsystem` 驱动 context 构造。
32. registry 组装层的 builder-entry 选择样板也已被收掉，dispatch entry 不再单独挂 subsystem-specific builder table；当前统一由 `subsystem` 驱动 builder table 选择。
33. 当前 dispatch descriptor 已经收缩到最小核心形态：只剩 `subsystem`。
34. 在 descriptor 收口基本完成后，当前工作已经开始转向更深的 builder/protocol 层统一，而不是继续停留在 registry 外壳整理。
35. joint 的 anchor / limit / drive spec builder 已经开始通过共享 helper 统一“gate -> plan -> make spec”模板，builder 层重复逻辑进一步下降。
36. 通用 optional row-spec helper 也已经开始从 joint 扩展到 contact/environment，`kind/index/gate/plan->spec` 这层协议不再只服务 joint。
37. 当前剩余的是继续减少 subsystem 风格分支，让 contact/environment/joint 更像统一 row-prep registry 的不同实例，而不是三套外观相似的 helper 群。
38. contact normal / friction 的 spec builder 已经收口到同一个 `buildOptionalDirectionalRowSpec(...)`，旧的 normal/tangent optional helper 入口已退出调用路径。
39. environment spec builder 也已经切到 `buildOptionalEnvironmentRowSpec(...)`，environment 不再继续直接走通用 plan helper 的裸调用形态。
40. joint 的 anchor / limit / drive spec builder 已进一步压缩到单个 `buildJointRowSpecForJoint(...)` 模板，当前差异主要只剩 `kind + gate + plan_builder` 三个参数。
41. builder entry 表已经从“函数指针表”进一步升级成“descriptor 表”：contact/joint/environment 当前直接在 entry 上携带 `kind/mode/gate/plan_builder` 等元数据，再通过统一 `buildConstraintRowSpecFromEntry(...)` 分派，不再需要为每个 builder entry 保留一层薄 wrapper 函数。
42. environment builder entry 也已经从空标签升级成显式 `environment_plan` descriptor，当前 environment 路径和 joint/contact 一样，都通过 entry 自带的计划元数据进入统一 row-spec 分派，而不再依赖特殊 case 形态。
43. environment 的最后一个专用 row-spec helper 也已经被删除：当前 environment descriptor 上携带的 `kind` 不再只是占位字段，而是已经直接走通用 `buildOptionalConstraintRowSpecFromPlan(kind, index, enabled, row_plan)` 路径，environment 和 joint 在 “plan -> spec” 这层已经真正对齐。
44. contact directional descriptor 现在也已经开始显式承载 `enabled` 元数据，而不是在分派函数里硬编码 `true`。这一步虽然还没有把 contact 提升为完整的 directional-plan descriptor，但已经让 contact entry 和 joint/environment 一样，开始朝“entry 自带完整 row-spec 控制参数”靠拢。
45. contact 的 directional payload 选择逻辑也已经进一步内聚到 `ContactPreparedPair.directionalPayload(mode)`，`buildContactRowSpecForPair(...)` 不再自己手写 `normal/tangent -> direction/equation` 的双 `switch`。虽然 contact 仍保留专用 prepared shape，但其 row-spec 构造入口已经更接近“descriptor 选择 payload，prepared 自身提供数据”的形式。
46. contact 的 `direction + equation` 现在也已经显式提升为共享 `DirectionalRowPayload` 小结构，`buildOptionalDirectionalRowSpec(...)` 不再接收散开的 `direction/equation` 形参，而是直接消费 payload。这样 contact row builder 已进一步向“descriptor -> payload -> generic directional spec”形态收敛。
47. contact directional descriptor 现在还进一步显式承载 `payload_selector` 元数据，dispatch 分支不再自己决定如何从 `ContactPreparedPair` 取 normal/tangent payload，而是由 entry 自带 selector 决定。这样 contact 已开始接近和 joint/environment 相同的模式：descriptor 不只表达“要哪种 row”，也表达“如何从上下文取出 row 所需 payload”。
48. 三类 builder entry 的匿名 payload 也已经开始正规化为命名 entry 结构，并且 `environment/joint` 也都显式补上了 `enabled` 元数据。虽然这些 `enabled` 目前仍都是常量 `true`，但 entry 外形已经更统一，后续若要继续抽象为更通用 registry，就不需要再先回头补齐这些控制槽位。

### Phase 2：让 joint 也接近共享 channel

目标：joint 不再是统一框架里最专用的一块。

具体步骤：

1. 为 joint anchor / limit / drive 定义更统一的 prepared channel
2. 识别 joint 里哪些属于：
   - pair angular channel
   - pair linear directional channel
   - motor target channel

3. 把现有 joint helper 往共享 step 层压：
   - 线性部分优先
   - 角向部分次之

4. 尝试引入 `PreparedAngularConstraint`

当前进度更新：

1. 第 1 步已完成大半。
2. joint linear prepared channel 已经进入：
   - anchor
   - spring
   - slider limit
3. joint angular prepared channel 已经起步，已进入：
   - hinge limit
   - hinge drive
4. 当前真正剩余的是把这些 prepared channel 继续往共享 apply/body-channel 层压，而不是继续停留在 joint 专用 apply helper。
5. joint row build 也已经开始从 wrapper 内部分支拆成更细粒度的 anchor / limit / drive builder，这为下一步识别 linear/angular/motor target channel 提供了更清晰入口。
6. 相比前一阶段，这里的重点已经从“把 joint 接到 shared apply step”转成“减少 joint wrapper，推进更通用 row primitive”。
7. `solveJointLimitRow(...)` 现在已经开始通过统一的 `prepareJointLimitChannel(...) -> applyPreparedJointLimitChannel(...)` 路径消费 linear / angular prepared 数据，不再在 solver 内部保留两套完全分离的 prepared 处理模板。
8. `solveJointDriveRow(...)` 也已经开始通过统一的 `prepareJointDriveChannel(...) -> applyPreparedJointDriveChannel(...)` 路径消费 slider / hinge drive prepared 数据；solver 当前只保留 velocity bias 这一层语义差异，而位置校正消费协议已开始统一。
9. `solveJointAnchorRow(...)` 现在也已经开始通过统一的 `prepareJointAnchorChannel(...) -> applyPreparedJointAnchorChannel(...)` 路径消费普通 anchor 与 spring prepared 数据；solver 内只保留 hinge / slider 的额外速度阻尼后处理，而不再自己展开两套 anchor 位移修正模板。
10. joint 的尾部后处理现在也已经开始收口：`solveJointAnchorRow(...)` 的 hinge / slider limit damping 已通过统一 `JointAnchorPostprocess` 协议进入共享 helper；`solveJointDriveRow(...)` 的 angular / linear motor velocity bias 也已通过统一 helper 消费 `JointPreparedDriveChannel`，而不再在 solver 尾部保留两套手写 postprocess 分支。
11. joint wrapper 的执行态 bookkeeping 也已经开始收口：`anchor / limit / drive` 三类 solver 现在开始共享 `JointRowExecutionState`，用于统一管理 `changed` 与 `applied_impulse` 聚合，不再各自手写 `max(applied_impulse, ...)` 和布尔累积模板。
12. `JointPreparedDriveResult` 现在也已经开始从“裸返回 solve_step + velocity”向更明确的 result 协议推进：它已显式携带 `impulse_hint`，并提供统一 `applyTo(...)` 入口，让 `solveJointDriveRow(...)` 不再自己展开 drive impulse hint 与 solve step 的执行态写回模板。
13. `limit` 与 `anchor` 现在也已经补上与 `drive` 对齐的 result 应用语义：两者当前都会通过统一的 `JointPreparedSolveResult.applyTo(...)` 把 solve step 写回 `JointRowExecutionState`。这意味着 joint 三条主路径都已经进入“prepared -> result -> execution-state”的同构外形。
14. `anchor` 与 `drive` 的后处理计划也已经开始进入 result：`JointPreparedSolveResult` 当前已显式承载 `postprocess`，`anchor` 会把 `JointAnchorPostprocess` 作为 result 的一部分返回，`drive` 会把 velocity bias 计划作为 result 的一部分返回。这样 solver 已不再直接决定这些后处理动作，而是开始消费 result 自带的 postprocess plan。
15. `limit / anchor / drive` 三条 joint 主路径现在都已经走统一的 `result.applyAll(...)` 外形：同一入口会完成 `solve_step` 写回、`impulse_hint` 聚合，以及 `postprocess` 应用。即便 `limit` 当前的 `postprocess` 仍是 `.none`，solver 外观也已经与另外两条路径完全对齐。
16. joint runtime solve 侧现在也已经开始 descriptor 化：新增 `JointRuntimeRowDescriptor` 与 `jointRuntimeRowDescriptor(...)`，`solveConstraintBlock(...)` 对 joint row 的分发已开始通过 runtime descriptor 选择 solver，而不再只依赖一个更大的 `switch` 直接硬连三条 solver。这样 joint 已不只是 build side 有 descriptor，runtime side 也开始具备同类收口点。
17. joint runtime descriptor 现在还进一步接管了共同前置逻辑：`solveJointRuntimeRow(...)` 已统一处理 `joint_idx` 越界、`base_residual <= 0`、`row_enabled` gate、entity 索引检查、broken 状态检查，以及 `mass_data` 获取。三条 joint solver 当前已不再重复这些前置模板，而是只负责各自的 result 生成与 finalize。
18. 这意味着 Phase 2 的 solver-wrapper 收口已经接近本阶段收益上限；后续更值得做的是继续把 residual / postprocess /前置检查元数据压进 joint runtime descriptor，而不是继续在 wrapper 壳上做小样板清理。

预期收益：

1. joint/contact/environment 三大类在 row 层真正接近同构
2. 通用 jacobian row 的阻力显著下降

### Phase 3：把预测性从“增益”升级为“计划”

目标：预测不只影响 bias/priority，而是形成明确求解目标。

具体步骤：

1. 抽 `PredictiveRowPlan`
   - horizon
   - predicted target
   - urgency
   - allowed correction budget

2. contact/environment 先接入 predictive plan
3. joint drive 再接入 predictive plan
4. 把未来一小段时间的 overshoot / collision worsening 明确转成 solve target

预期收益：

1. 从“预测辅助求解”走向“预测驱动求解”
2. 更符合用户要求的“像人类一样预估未来几秒并可控处理”

### Phase 4：朝真正的 Jacobian row 迈进

目标：从当前共享框架升级为更正统的通用约束系统。

具体步骤：

1. 设计统一 row primitive
   - body count
   - linear/angular axes
   - effective mass
   - bias
   - impulse limits
   - accumulated lambda

2. 把现有 `ConstraintRowEquation` 升级为更丰富的 row data
3. 引入通用 accumulated lambda 管理
4. 逐步迁移 contact normal / friction / environment / joint anchor / limit / drive

这一步不要一次性硬切，应该按 row 类型渐进替换。

当前进度更新：

1. `ConstraintApplyPrimitive` 已经落地，成为 shared apply step 的执行元数据壳。
2. 当前 primitive 已承载：
   - `body_mode`
   - `channel`
   - `equation`
   - `bias_scale`
   - `warm_impulse`
   - `magnitude_slop`
   - `clamp_non_negative`
   - `axis_x/y/z`
   - `pair_bodies`
3. `pair_bodies` 已经开始接入：
   - contact normal position correction
   - contact normal velocity impulse
   - contact friction warm-start / solve impulse
   - joint anchor / spring directional correction
   - joint slider axis correction
   - joint slider limit linear velocity damping
   - joint hinge angular position correction
   - joint hinge angular velocity damping / motor bias
4. 对这些 shared pair step 来说，重复的 fallback `inv_mass_a/inv_mass_b/JointMassData` 形参已经被移除，primitive 成为唯一 body metadata 入口。
5. 高层 wrapper 也已经开始通过 pair primitive helper 生成：
   - joint mass -> primitive
   - contact inverse masses -> primitive
6. 它还没有覆盖：
   - body count
   - 多体/多轴 bundle 的统一表达
   - accumulated lambda
   - clamp policy registry
7. 因此下一阶段不应该再从空白开始设计 primitive，而应直接扩展这层已有结构，并继续把 row builder 也推进到类似的 helper/registry 形态。

### Phase 5：补底层测试

当前测试是够用的，但还缺面向新抽象层的测试。

需要补：

1. prepared directional channel 测试
2. directional row plan 测试
3. predictive gain / predictive row plan 测试
4. shared apply step 测试
5. postprocess/finalize 协议测试

特别要补的场景：

1. contact normal 和 friction 的 shared helper 行为一致性
2. environment row 和 batch path 走相同 helper 的一致性
3. 预测深度只增强、不反转求解方向

## 建议优先级

最推荐的下一个实际实施顺序：

1. 继续把 joint/contact/environment row builder 往统一 registry 入口压，优先减少 `buildConstraintRowsForIsland(...)` 周边的 subsystem 风格分支
2. 继续把 joint anchor / limit / drive 对齐到更明确的共享 linear / angular / motor target channel
3. 再把预测性从 bias/priority shaping 升级为显式 `PredictiveRowPlan`
4. 最后再推进更通用 jacobian row primitive

原因：

1. 风险最低
2. 与现有代码形状最贴近
3. 可以持续保持测试稳定
4. 不会过早重写整套 solver

## 当前不建议立即做的事

1. 不建议现在就重写成完整 LCP/MLCP 求解器
2. 不建议一次性推翻现有 joint 专用逻辑
3. 不建议现在转去章节测试或表层特性
4. 不建议现在宣称底层已经全部完成

## 当前状态一句话总结

底层已经从“多套专用约束求解”推进到“共享 row 框架 + 共享 prepared directional channel + joint prepared linear/angular channel + `ConstraintApplyPrimitive`（已开始携带 pair body 元数据，并覆盖 linear/angular velocity channel）+ 共享 apply step + pair/single finalize 壳”，而且预测性已经进入 contact/environment 的求解核心；但距离完整通用 Jacobian / lambda 约束系统，仍然还有 tangent row 泛化、predictive plan 升级、以及在现有 primitive 上继续补齐 body bundle/lambda/clamp policy 这几大步。
