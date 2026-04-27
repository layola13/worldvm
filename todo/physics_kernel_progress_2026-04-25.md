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
5. 还没有形成真正统一的 `prepare -> plan -> solve -> postprocess -> finalize` 完整泛型协议，当前只是大部分关键层已经收口；但 joint runtime 已经把 `prepare_channel -> prepared outcome resolution -> ready build -> finish` 这一段拉成共享协议。
6. `ConstraintApplyPrimitive` 已经出现，说明底层不再只是 helper 并列，而是开始形成可承载通用 row primitive 的执行元数据壳。

## AABB 语义保留更新（2026-04-26）

根据当前迭代要求，查询/角色碰撞侧明确保留 `AABB` 语义，不再把 `kcc` 顶碰解析混回 capsule 分支。

本轮已完成：

1. `kcc.resolveCollision(...)` 回到 `computePenetrationAABB(...)` 主路径，移除先前“顶碰依赖 capsule 渗透”的分叉执行。
2. `resolveCollision` 现在使用 AABB 渗透结果做小步迭代去穿透（最多 4 次），避免单次最小平移轴在复杂接触下留下残余重叠。
3. 在“上升过程发生重叠”场景中，显式钳制 `vel_y`，并保留切向速度；AABB 顶碰只补最小下压余量，不恢复 capsule 分支。
4. 顶层重叠触发条件改为“离散 AABB 采样或 AABB 渗透结果任一命中”，覆盖采样漏检但渗透可检出的边界场景。

验证结果：

- `zig test src/kcc.zig` 全部通过（235/235）
- `zig test src/physics_kernel.zig` 全部通过（235/235）
- `zig test src/tick_engine.zig` 全部通过（235/235）
- `zig test src/physics_world.zig` 全部通过（235/235）
- `kcc` 关键回归用例
  - `KCC resolveCollision slides along sloped ceiling instead of killing tangent motion`
  已恢复通过

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

最近一轮又进一步推进了一步：

- `JointPreparedOutcome(T)`
- `JointPreparedResolution(T)`
- `JointRuntimeRowPolicy`
- `prepareResolvedJointSolveResult(...)`
- `buildReadyJointSolveResultFromPreparedChannel(...)`

这意味着 joint 三类 row 已经不再各自维护 “prepare 失败如何收口 / inactive 如何收口 / stalled 如何收口 / ready 如何进入 apply” 的专用分支，而是统一走 descriptor policy。

同时，之前文档里提到的 row-specific ready builder 已经不再保留：

- `buildReadyJointLimitSolveResult(...)`
- `buildReadyJointAnchorSolveResult(...)`
- `buildReadyJointDriveSolveResult(...)`

这些中间层已经被删除，ready 路径统一收口到：

- `buildReadyJointSolveResultFromPreparedChannel(...)`

并且最新实现又把 joint 的部分后处理策略前移到了 prepared channel 本身：

- `JointPreparedAnchorChannel.postprocess`
- `JointPreparedDriveChannel.*.postprocess`

也就是说，`buildReadyJointSolveResultFromPreparedChannel(...)` 现在只负责按 channel 分发到 apply helper，而不再临时拼装 anchor/drive 的后处理策略。

同时，`joint_drive` 这层 ready/apply 内部又做了一次小幅收口：

- `jointDriveSignedCorrection(...)`
- `prepareJointDriveSolvePostprocess(...)`

这一步还没有把 `drive` 和 `limit` 真正合并成同一类 apply family，但已经把 `drive` 内部原本 angular / linear 分支里重复出现的：

- planned signed correction 模板
- velocity-bias postprocess 组装

压成共享 helper。这样下一步如果要继续把 joint ready apply 往更通用的 primitive/apply-family 层推进，入口会更干净。

再下一小步，`joint_limit` 与 `joint_drive` 已经开始共享更底层的 ready-result 构造 helper：

- `buildJointAngularPositionSolveResult(...)`
- `buildJointLinearAxisSolveResult(...)`

这意味着 `limit` 与 `drive` 虽然在 correction shaping 和 postprocess 上仍有差异，但它们已经不再各自手写：

- angular position solve result 的 primitive 构造
- linear axis solve result 的返回壳

剩余更明显的差异已经进一步收缩为：

- correction 的计算方式
- impulse hint 的注入
- postprocess 的注入

其中 correction 这层也已经开始被显式拉平：

- `jointLimitSignedCorrection(...)`
- `jointDriveSignedCorrection(...)`

这还不是最终的统一 shaping policy，但至少 joint 代码不再把 `constraintRowSignedCorrection(...)` / `constraintRowPlannedSignedCorrection(...)` 的参数模板直接散落在 `limit` / `drive` 的 apply 分支里。下一步如果要继续把 shaping 抽进 row policy，改造面会更集中。

最新一轮已经把这一步真正推进到了 descriptor policy：

- `JointSignedCorrectionPolicy`
- `JointRuntimeRowPolicy.signed_correction_policy`
- `jointRowSignedCorrection(...)`

当前状态变成：

1. `limit` / `drive` 的 correction shaping 参数来源，已经不再埋在 helper 常量里
2. `buildReadyJointSolveResultFromPreparedChannel(...)` 会通过 descriptor policy 把 shaping 规则下传给 ready/apply
3. `joint_limit` 与 `joint_drive` 当前的差异，已经更多体现在 policy 数据，而不是 apply 分支散写常量

这一步的意义是：joint runtime 现在不仅统一了 `inactive/stalled/ready/postprocess` 的收口，连 correction shaping 这层也开始从“代码分支语义”转向“descriptor policy 数据”。

最新一轮又把 joint 顶层 row 入口外形继续拉平到了和 `contact/environment` 更接近的 runtime protocol：

- `JointRuntimeSolveContext`
- `JointRuntimeSolveContextOutcome`
- `prepareJointRuntimeSolveContext(...)`
- `executePreparedJointRuntimeRow(...)`
- `executePreparedJointRuntimeOutcome(...)`

当前效果：

1. `solveJointRuntimeRow(...)` 不再自己展开：
   - descriptor 查找
   - joint 索引/残差检查
   - row enable 检查
   - instance/broken 检查
   - mass data 准备
   - generic execute 调度
2. joint row 入口现在也正式变成：
   - `prepare runtime context -> execute prepared runtime outcome`
   这和 `contact` / `environment` 的顶层形态已经一致
3. 原本只起过渡作用的 `solveJointGenericRow(...)` 已退出调用路径，joint 不再额外保留一层 generic wrapper
4. 到这一层为止，joint/contact/environment 三块在顶层 row runtime protocol 上已经基本对齐，后续如果继续提炼共用 dispatch，切入点会比之前更集中

之后又继续推进了一小步：

- `JointImpulseHintPolicy`
- `JointRuntimeRowPolicy.impulse_hint_policy`
- `jointImpulseHint(...)`

当前 `impulse_hint` 的来源也已经不再硬编码在 `applyPreparedJointDriveChannel(...)` 内部，而是开始通过 descriptor policy 数据下传。

这意味着 joint ready/apply 当前还保留的行为差异，已经更多集中在：

- 是否需要 postprocess
- postprocess 类型是什么

而不是继续散落在：

- correction 参数常量
- impulse hint 注入时机

随后又继续把 solve result 的元数据装配收口了一层：

- `prepareJointAnchorSolvePostprocess(...)`
- `withJointSolveResultMetadata(...)`

这一步的效果是：

1. `anchor` / `drive` 不再各自手工给 `JointPreparedSolveResult` 填 `impulse_hint` / `postprocess`
2. solve result 的“主 solve_step”和“附加元数据”开始显式分层
3. 之后如果继续把 postprocess 进一步 descriptor 化，改造入口会比之前更集中

再往前推进一小步之后，`drive_velocity_bias` 的 postprocess 载荷也被缩减了：

1. 不再额外保存一份 `desired_velocity`
2. 直接把 `JointPreparedDriveChannel` 作为 postprocess 载荷
3. `applyPreparedJointDriveVelocityBias(...)` 在消费时直接从 prepared drive channel 读取目标速度

这说明 `postprocess` 这层也开始从“额外拼一份派生数据”转向“尽量直接消费 prepared channel 本身”。

这一轮已经开始从 `joint` 转去下一个底层缺口：`environment`。

新增：

- `EnvironmentResolvedMotion`
- `EnvironmentSolveResult`
- `solvePreparedEnvironmentMotion(...)`
- `prepareEnvironmentResolvedMotion(...)`

当前效果：

1. `environment` 的 row 路径不再自己手工把 solve impulse 再次转成 move 向量并立刻消费
2. `environment` 的批处理路径也开始复用同一类 resolved motion 结果壳
3. `environment` 现在开始具备和 `joint` 类似的“prepared -> solve result -> apply/finalize”收口趋势

这一步还没有把 `environment` 完全 descriptor 化，但已经把原来分散在批处理/row 路径里的 motion-result 装配拉到共享层。

接着又开始切入 `contact` 的组合执行壳。

新增：

- `ContactSolveAccumulator`

当前效果：

1. `solvePreparedContactNormalSteps(...)` 不再手工维护 `changed/applied_impulse` 聚合
2. `solvePreparedContactFrictionSteps(...)` 也不再手工维护同一套聚合逻辑
3. `contact` 的 warm-start 与 solve-step 聚合开始具备共享执行壳

这一层虽然还没有像 `joint` 一样继续推到 descriptor/outcome/policy，但已经把 normal/friction 两条组合路径里的“重复 bookkeeping”收掉了。

紧接着，`contact` 批处理路径也开始有共享 pair 结果壳：

- `ContactPairSolveResult`

当前效果：

1. batch path 里不再直接散写 `normal_step.changed or friction_step.changed`
2. normal/friction 的 pair 级组合结果开始具备共享外壳
3. 后续如果要把 `contact` 再往 `ready result / finish` 收口，已经有更稳定的 pair 级承载点

随后又继续把 `contact` 的 pair 级执行入口往前收口了一层：

- `ContactSolveRequest`
- `ContactPairSolveRequest`
- `ContactExecutionContext`
- `ContactExecutionOutcome`
- `prepareContactExecutionContext(...)`
- `solvePreparedContactPair(...)`

当前效果：

1. batch path 和 row path 不再各自重复写 contact pair 的 `instance/entity/prepared` 前置检查
2. normal / friction 的求解选择开始通过 pair request 显式表达，而不是在多个调用点手工拼 solve 顺序
3. `solveContactConstraintsForPairs(...)` 与 `solveContactRuntimeRow(...)` 已经进入同一条 `prepare -> pair request -> pair result -> finalize/wake` 主链
4. 这让 contact 更接近 joint 当前已经具备的 “execution context + outcome + ready result” 收口方向，后续继续做 pair-level finalize / policy 时阻力更小
5. contact 的 settle/wake 与 row finalize 也已经开始直接消费 `ContactExecutionContext`，batch/row 尾部不再继续散传 `inv_mass/normal_y/inst_a/inst_b` 这类 pair 元数据
6. `contact` row 路径也已经开始具备最小 runtime descriptor 形态：当前通过 `ContactRuntimeRowDescriptor` + `solveContactRuntimeRow(...)` 统一 normal/friction 的 row 执行，dispatch 不再需要知道两套 row solver 细节
7. `contact` row 的 channel 请求构造与 solve-step 选择也已经开始进入共享 helper：当前通过 `buildContactRowSolveRequest(...)` 和 `solvePreparedContactRuntimeRow(...)` 把 `descriptor -> request -> prepared outcome` 这层再往前收口，`solveContactRuntimeRow(...)` 本体分支密度已明显下降
8. `contact` row 现在也开始有最小 `outcome -> finish` 壳：当前通过 `ContactPreparedPairOutcome` 和 `finishContactSolveResultOutcome(...)` 承接 solve 结果，`solveContactRuntimeRow(...)` 已进一步接近和 `joint` 同形的 `prepare -> outcome -> finish` 主骨架
9. `contact` batch path 也开始接入同类 finish 语义：当前通过 `ContactPreparedPairOutcome`、`solvePreparedContactPlan(...)`、`finishContactBatchSolveOutcome(...)` 收口 `pair_result.changed + settle/wake`，batch 不再继续裸写 solve/settle 组合尾部
10. row / batch 两条路径的 `ready` 载荷现在也已经统一回 `ContactPairSolveResult`，不再分裂成两套只差单步/双步外形的 outcome 载荷；这让 `contact` 的 prepared result protocol 更接近单一壳
11. row / batch 两条路径的 request 构造现在也开始统一进 `ContactPreparedPairPlan`：当前通过 `buildContactRowSolvePlan(...)`、`buildContactBatchSolvePlan(...)`、`solvePreparedContactPlan(...)` 收口 `request + requires_tangent` 这一层，`contact` 已开始具备更完整的 `plan -> outcome -> finish` 协议外形
12. island row dispatch 现在也已经直接命中 `solveContactRuntimeRow(...)`，旧的 `solveContactNormalRow(...)` / `solveContactFrictionRow(...)` 薄包装层已退出调用路径；这让 `contact` 更接近真正的 runtime dispatch，而不是保留多余的别名入口
13. batch 侧只服务单调用点的 `solvePreparedContactBatch(...)` 包装层也已退出调用路径，当前 batch 直接走共享 `buildContactBatchSolvePlan(...) -> solvePreparedContactPlan(...) -> finishContactBatchSolveOutcome(...)` 主链
14. `contact` row descriptor 现在还进一步显式携带 `build_plan` 与 `select_step` 元数据，`solvePreparedContactRuntimeRow(...)` 和 `finishContactSolveResultOutcome(...)` 已不再自己决定 normal/friction 的 row 计划和 step 选择；这让 channel 差异开始真正压回 descriptor
15. 旧的过渡 helper `buildContactRowSolveRequest(...)` 与通用 `buildContactRowSolvePlan(...)` 已退出调用路径，说明 `contact` row 的 descriptor 外移不再只是增加一层包装，而是已经开始实质删减中间分派层
16. `contact` row 现在也开始具备更明确的 runtime solve context：当前通过 `ContactRuntimeSolveContext` 和 `prepareContactRuntimeSolveContext(...)` 把 `kind/descriptor/contact execution/before residual` 收成统一上下文，`solveContactRuntimeRow(...)` 不再继续散传这些运行态元数据，外形进一步向 `joint` 靠拢
17. `contact` batch 路径现在也开始具备对应的最小 runtime 壳：当前通过 `ContactBatchSolveDescriptor`、`ContactBatchSolveContext`、`prepareContactBatchSolveContext(...)`、`solvePreparedContactBatch(...)` 把 batch 侧也收成 `prepare context -> descriptor build plan -> prepared outcome -> finish` 外形，`solveContactConstraintsForPairs(...)` 不再继续裸串 `prepare/build_plan/solve/finish`
18. batch 的 settle 策略也已开始回收到 descriptor 元数据：`finishContactBatchSolveOutcome(...)` 不再默认总是 settle，而是消费 `ContactBatchSolveDescriptor.settle_after_solve`，这样 batch 与 row 都开始通过 descriptor 决定尾部行为，而不是在调用点硬编码
19. 这一步虽然还没有把 batch/row 真合成单一入口，但 contact 两条路径现在已经共享：
    - `ContactExecutionContext`
    - `ContactPreparedPairPlan`
    - `ContactPreparedPairOutcome`
    - prepared solve / finish 壳
    差异已进一步收缩到 row/batch 各自的 descriptor/context，而不是继续散落在调用点
20. 入口调用点也进一步变薄：当前通过 `executePreparedContactRuntimeRow(...)` 与 `executePreparedContactBatch(...)`，`solveContactRuntimeRow(...)` 和 `solveContactConstraintsForPairs(...)` 已不再自己拼 `solvePrepared... + finish...` 组合，contact 开始形成更清晰的 `prepare context -> execute prepared` 双段结构
21. row/batch 的 context 封装前置模板也继续收口：当前通过 `mapPreparedContactRuntimeSolveContext(...)` 与 `mapPreparedContactBatchSolveContext(...)` 统一消费 `prepareContactExecutionContext(...)` 的 outcome，`prepareContactRuntimeSolveContext(...)` / `prepareContactBatchSolveContext(...)` 不再各自重复写同形的 `inactive/stalled/ready` 包装分支

随后开始把同类思路推到 `environment`：

- `EnvironmentExecutionContext`
- `EnvironmentExecutionOutcome`
- `prepareEnvironmentExecutionContext(...)`

当前效果：

1. `solveEnvironmentConstraintsForIndices(...)` 与 `solveEnvironmentRow(...)` 不再各自重复写：
   - instance 边界检查
   - `broken/entity_id/mass/static` 检查
   - `prepareEnvironmentConstraint(...)`
   - `previous_vel_*` 捕获
2. `environment` 的 row/batch 两条路径第一次开始共享同一个执行上下文壳，虽然还没有像 `contact` 那样继续推进到 descriptor/context/outcome/finish，但至少 prepare 阶段已经不再是两套分离模板
3. 这说明下一阶段把 `environment` 继续拉到 `prepare -> execute prepared -> finalize` 形态的改造面已经变得更集中

随后 `environment` 的尾部执行也继续收口了一层：

- `buildPreparedEnvironmentBatchSolveResult(...)`
- `executePreparedEnvironmentResolvedMotion(...)`

当前效果：

1. batch 路径不再自己手拼 `EnvironmentSolveResult{ changed/applied_impulse/resolved_motion }`
2. row / batch 不再各自重复写 `applyResolvedMotion(...) + wakeInstance(...)`
3. `environment` 现在已经开始接近 contact 当前的节奏：先共享 execution context，再共享 prepared result / execute 尾部，后续继续推进 finalize/outcome 会更直接

再下一步，`environment` row 的执行主链也已经开始显式收口到：

- `executePreparedEnvironmentRow(...)`

当前效果：

1. `solveEnvironmentRow(...)` 已不再自己展开 warm-start / solve / execute / finalize 细节
2. `environment` row 入口现在也开始接近 `prepare context -> execute prepared` 双段结构
3. 这意味着如果后续要继续给 `environment` 增加 runtime context/outcome/descriptor，改造入口会明显比之前更窄

这之后又补上了最小 row runtime 壳：

- `EnvironmentRuntimeSolveContext`
- `EnvironmentRuntimeSolveContextOutcome`
- `prepareEnvironmentRuntimeSolveContext(...)`

当前效果：

1. `solveEnvironmentRow(...)` 不再直接处理 `base_residual <= 0` 与 execution-context 包装逻辑
2. `environment` row 现在在入口外形上已经更接近 `contact` 的 `prepare runtime context -> execute prepared`
3. 这还不是完整 descriptor/policy/runtime 协议，但已经把 row 入口需要关心的运行态元数据进一步收成单一上下文

最后又补了一层结束路径薄壳：

- `executePreparedEnvironmentBatch(...)`
- `finalizePreparedEnvironmentRowResult(...)`

当前效果：

1. batch 路径不再在调用点直接组合 `buildPreparedEnvironmentBatchSolveResult(...) + executePreparedEnvironmentResolvedMotion(...)`
2. row 路径也不再在 `executePreparedEnvironmentRow(...)` 里直接拼 `finalizeEnvironmentRowResult(...)` 的上下文字段
3. `environment` 的 row/batch 两条路径现在在结构上已经更完整地对齐到：
   - prepare context
   - execute prepared
   - finalize/return

顺手还清掉了 `environment` batch 迭代里的一个无效重复：

1. `solveEnvironmentConstraintsForIndices(...)` 不再先调用一次全局 `buildEnvironmentPriorityOrder(...)` 再立刻调用 `buildEnvironmentPriorityOrderForIndices(...)`
2. 当前批处理循环只保留 subset 侧排序结果，因为前一个全局排序结果原本只用于判零，没有其他消费点
3. 这不会改变求解语义，但减少了一次每轮无意义的环境优先级扫描

最后再补了 batch 侧最小 runtime 壳：

- `EnvironmentBatchSolveContext`
- `EnvironmentBatchSolveContextOutcome`
- `prepareEnvironmentBatchSolveContext(...)`

当前效果：

1. `environment` batch 路径也开始从 `prepare execution context -> execute prepared` 提升为 `prepare batch runtime context -> execute prepared`
2. row / batch 两条路径的入口外形现在更一致，后续如果完全停止打磨 `environment`，这一层已经足够整齐

最后一小步又把 `environment` 的 runtime/batch context 包装模板收掉了：

- `mapPreparedEnvironmentRuntimeSolveContext(...)`
- `mapPreparedEnvironmentBatchSolveContext(...)`

当前效果：

1. `prepareEnvironmentRuntimeSolveContext(...)` 与 `prepareEnvironmentBatchSolveContext(...)` 不再各自重复写 `inactive/stalled/ready` 映射模板
2. `environment` 这一块现在在 prepare/context/execute/finalize 的结构一致性上，已经接近这阶段值得投入的上限

最后把 batch execute 入口也对齐了一下：

1. `executePreparedEnvironmentBatch(...)` 现在直接消费 `EnvironmentBatchSolveContext`
2. `solveEnvironmentConstraintsForIndices(...)` 不再在调用点手动解包 `ctx.execution`
3. 这样 `environment` batch 入口在表面结构上已经更接近 `contact` 的 `prepare batch context -> execute batch`

同时，这一轮也顺手确认了两层旧 finalize 包装已经没有保留价值并移除：

- `finalizeContactRowResult(...)`
- `finalizeEnvironmentRowResult(...)`

当前效果：

1. `contact` 已稳定通过 `finalizePreparedContactRowResult(...)` + shared solve-step finalize helper 收口，不再回退到旧式 `changed/applied_impulse` 包装
2. `environment` 已稳定通过 `finalizePreparedEnvironmentRowResult(...)` + `finalizeSingleSolveStepResult(...)` 收口
3. 这说明这两块的 finalize 协议已经正式以 solve-step 语义为主，不再保留旧 row-result 包装层作为并行路径

再下一步，三块 runtime/batch outcome 上重复的 `inactive/stalled/ready` 解包模板也开始收口：

- `executePreparedRuntimeOutcome(...)`
- `executePreparedBatchOutcome(...)`

当前效果：

1. `joint` / `contact` / `environment` 的 row runtime outcome 都不再各自重复写同形的：
   - inactive -> `inactiveConstraintRowResult()`
   - stalled -> `stalledConstraintRowResult(base_residual)`
   - ready -> execute prepared row
2. `contact` / `environment` 的 batch outcome 也不再各自重复写：
   - inactive/stalled -> `false`
   - ready -> execute prepared batch
3. 这一步只统一了控制流 dispatch，没有触碰任何 solver 数学、prepared plan、apply step 或 finalize 细节
4. 到这里为止，三块子系统在顶层协议上的相似部分已经更多体现为共享 dispatch helper，而不是“看起来相似、实际上各自保留一份 switch”

随后又继续把 `contact/environment` 两块的 execution-outcome 映射模板合并到了同一层：

- `mapPreparedExecutionOutcome(...)`

当前效果：

1. `mapContactExecutionOutcome(...)` 与 `mapEnvironmentExecutionOutcome(...)` 已退出调用路径
2. `contact` 与 `environment` 在：
   - execution outcome -> runtime context
   - execution outcome -> batch context
   这两类包装上不再各自保留一份同形 mapper
3. 这一步继续只处理 prepare/context 协议外壳，没有改动 prepare 检查逻辑本身
4. 到这里为止，prepare 阶段真正还没统一的差异，已经更多是各自的领域检查与 context 载荷，而不是纯模板式 `inactive/stalled/ready` 映射代码

再往前收口一小步之后，这四个只剩单调用点的 prepare-context 包装层也已经退出：

- `mapPreparedContactRuntimeSolveContext(...)`
- `mapPreparedContactBatchSolveContext(...)`
- `mapPreparedEnvironmentRuntimeSolveContext(...)`
- `mapPreparedEnvironmentBatchSolveContext(...)`

当前效果：

1. `prepareContactRuntimeSolveContext(...)`
2. `prepareContactBatchSolveContext(...)`
3. `prepareEnvironmentRuntimeSolveContext(...)`
4. `prepareEnvironmentBatchSolveContext(...)`

现在都直接命中共享 `mapPreparedExecutionOutcome(...)`

这意味着 prepare/context 这层已经不再保留“共享 mapper 外面再包一层薄函数”的过渡结构，调用链更短，协议形态也更直接。

再下一小步，`contact` 的 prepared-pair finish 分发也开始共用同一层 mapper：

- `mapPreparedContactPairOutcome(...)`

当前效果：

1. `finishContactSolveResultOutcome(...)` 不再单独维护 `ready/stalled` switch
2. `finishContactBatchSolveOutcome(...)` 也不再单独维护同形 switch
3. row / batch 的差异进一步收缩到各自的 ready-handler，而不是继续各自保留 outcome 分发模板
4. 这一步仍然没有触碰 `contact` 的 solve plan、settle 策略或 finalize 语义，只把 finish 阶段的控制流壳继续拉平

最后又顺手清掉了 `joint` finish 侧两个只服务单调用点的过渡层：

- `finishJointSolveResultOutcome(...)`
- `finishUnreadyJointSolveResultOutcome(...)`

当前效果：

1. `executePreparedJointRuntimeRow(...)` 现在直接对 `JointSolveResultOutcome` 做最终收口
2. `joint` 不再保留“runtime row -> finish wrapper -> unready wrapper -> final result”这条多余链路
3. `ready/no_change/finalize_exec_state/stalled` 四类 outcome 仍然全部保留，行为没有变化，只是落点更直接
4. 到这里为止，`joint` 的 finish 层更接近 `contact/environment` 当前的“尽量少保留只做一次分发的薄壳”风格

最后又顺手把 `joint` 剩下两个只服务单调用点、且不再承载独立协议语义的 helper 内联掉了：

- `buildJointRuntimeSolveContext(...)`
- `executePreparedJointSolveResult(...)`

当前效果：

1. `prepareJointRuntimeSolveContext(...)` 直接构造 `JointRuntimeSolveContext`
2. `executePreparedJointRuntimeRow(...)` 的 `ready` 分支直接完成：
   - apply prepared result
   - finalize execution state
3. 这一步没有新增任何抽象，只是把最后一层明显多余的单调用点 helper 消掉
4. 到这里为止，这一段协议收口工作已经接近“该停就停”的边界，继续抽象的收益开始明显下降

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
- 当前三组都是 `204/204` 全通过

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
   - `applyPreparedJointLimitChannel(...)`
   - `applyPreparedJointAnchorChannel(...)`
   - `applyPreparedJointDriveChannel(...)`
   这三条 ready-apply helper 仍然各自保留语义差异，还没有继续压到更通用的 primitive/apply-family 元数据层
   - prepared channel 还没有和更通用 jacobian row 数据完全对齐
   - finalize 虽然已开始 pair/single 收口，但 residual 量测仍按 subsystem 分开
5. 但相较上一阶段，joint 的 `anchor` / `limit` / `drive` 已经不再只是“prepared shape 部分统一、solver 消费仍分裂”的状态；现在三类行都已经开始通过共享 channel helper 进入统一消费路径，且部分 postprocess 已经前移到 prepared channel，使下一步继续把 motor target / angular-position / linear-directional 压进更通用 body-channel 协议时，阻力明显下降。

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
19. joint runtime descriptor 现在已经不只承载 `residual_scale`，还开始承载运行时执行策略：新增 `warm_impulse_scale` 与 `unavailable_policy`，anchor / limit / drive 三类 row 的 warm-start 强度和 “prepared 缺失时如何收口” 都已经转成 descriptor 元数据，而不是继续散落在各自 solver wrapper 里硬编码。
20. 为配合这一步，joint runtime context 也已经显式持有 descriptor 本身，后续 solver 内部不再需要重新判断当前 row 的 runtime 策略；同一上下文现在可以直接驱动 warm-start 初始化、prepared 缺失分支，以及 finalize 收口。
21. joint solver wrapper 这边也已经新增共享 runtime helper：`initJointExecutionState(...)`、`finalizeJointExecutionState(...)`、`handleUnavailablePreparedJointRow(...)`。这样 `anchor / limit / drive` 三条路径当前都已经走统一的 execution-state 初始化与 prepare-miss 收口模板，而不是分别手写：
   - `warm_impulse = constraintRowWarmImpulse(...)`
   - `JointRowExecutionState.init(...)`
   - `prepared == null` 时各自 decide `no_change / stalled / finalize current state`
22. 当前三类 joint row 的 unavailable policy 已经被清晰固化为：
   - `anchor -> finalize_exec_state`
   - `limit -> finalize_no_change`
   - `drive -> stalled`
   这一步虽然还没有把 prepare outcome 提升成更通用协议，但已经先把“语义差异属于 runtime policy，而不是 solver 杂散分支”这个边界立住了。
23. joint runtime 现在还已经补上了显式的 prepared outcome 外壳：新增泛型 `JointPreparedOutcome(T)`。当前 `anchor / limit / drive` 三条 solver 已经不再直接依赖 `prepare...(...) orelse return ...` 这种散乱分支，而是统一先把 prepared channel 提升成显式 outcome，再由 runtime policy 与 finish 层决定如何收口。
24. 这一步最近又继续推进了一层：`JointPreparedOutcome` 已经从早期的抽象二态推进成显式三态协议：
   - `ready`
   - `inactive`
   - `stalled_prepare`
25. 当前语义已经明确区分：
   - `inactive`：该 row 对当前 joint 类型/状态逻辑上不适用
   - `stalled_prepare`：该 row 本应适用，但 prepare 阶段拿不到必要数据，无法形成 solve payload
   - `ready`：prepared channel 已经可被 solve 侧消费
25. 在这之后，joint solver wrapper 的主体也已经继续退化：新增 `JointSolveResultOutcome`、`prepareJointLimitSolveResult(...)`、`prepareJointAnchorSolveResult(...)`、`prepareJointDriveSolveResult(...)`，以及统一的 `finishJointSolveResultOutcome(...)` / `solveReadyJointResult(...)`。这意味着 `solveJointAnchorRow(...)` / `solveJointLimitRow(...)` / `solveJointDriveRow(...)` 当前已经不再自己负责：
   - prepared channel 提取后的 apply 调用
   - solve result 写回 execution-state
   - ready / unavailable 两态收口
   而是开始退化为：
   - 取当前 runtime context
   - 初始化 execution-state
   - 选择对应的 `prepare...SolveResult(...)`
   - 交给统一 `finish...Outcome(...)` 完成收尾
26. 这一步的直接结果是：joint 三条主路径现在在 runtime solve 壳上已经高度同构，彼此真正剩下的核心差异，主要只剩：
   - 使用哪种 prepared channel
   - anchor 是否需要 warm impulse 参与 directional constraint
   - 各自的 joint 语义 prepare 逻辑
   也就是说，wrapper 层已经接近“仅承载语义差异”，不再承担 protocol 级重复模板。
27. joint postprocess policy 也已经开始进入 runtime descriptor：当前新增 `JointAnchorPostprocessPolicy`，并由 `JointRuntimeRowDescriptor.anchor_postprocess_policy` 控制 anchor row 是否按 joint 类型生成后处理计划。`prepareJointAnchorSolveResult(...)` 已不再直接硬连 `prepareJointAnchorPostprocess(joint_def)`，而是转为通过 `prepareJointDescriptorAnchorPostprocess(ctx)` 读取 descriptor policy。
28. drive 的 velocity bias 后处理现在也已经进入同一层 descriptor policy：新增 `JointDrivePostprocessPolicy`，并由 `JointRuntimeRowDescriptor.drive_postprocess_policy` 控制 drive row 是否生成 velocity-bias postprocess。`applyPreparedJointDriveChannel(...)` 已不再默认内部硬编码 `.drive_velocity_bias`，而是改为显式接收 `JointPreparedPostprocess`，再由 `prepareJointDriveSolveResult(...)` 通过 `prepareJointDescriptorDrivePostprocess(ctx, prepared, desired_velocity)` 注入。
29. 在此基础上，joint runtime descriptor 现在又进一步收口成显式 policy 结构：新增 `JointRuntimeRowPolicy`，把此前分散在 descriptor 顶层的：
   - `residual_scale`
   - `warm_impulse_scale`
   - `unavailable_policy`
   - `anchor_postprocess_policy`
   - `drive_postprocess_policy`
   全部压入单个 `policy` 字段。当前所有读取点也都已经切到 `descriptor.policy.*`。
30. 这一步的价值不是纯命名整理，而是明确了 “descriptor 负责 row 身份与 solver 入口，policy 负责 runtime 行为策略” 这条边界。后续如果继续加入：
   - prepare outcome policy
   - finalize route policy
   - predictive bias policy
   就不需要再污染 descriptor 顶层字段布局。
31. joint runtime outcome 现在也已经从二态进一步升级成显式多态协议：`JointSolveResultOutcome` 当前不再只是 `available / unavailable`，而是已经明确区分：
   - `ready`
   - `no_change`
   - `finalize_exec_state`
   - `stalled`
   与此同时，旧的 `handleUnavailablePreparedJointRow(...)` 外部收口 helper 已被移除，`prepareJointLimitSolveResult(...)` / `prepareJointAnchorSolveResult(...)` / `prepareJointDriveSolveResult(...)` 在 prepared 缺失时会直接产出明确 outcome，再由统一 `finishJointSolveResultOutcome(...)` 消费。
32. 这一步的意义是：joint runtime 现在真正具备了“prepare 阶段直接表达 solver 下一步该怎么收尾”的协议能力，而不是先返回模糊的 `unavailable`，再在外层重新查 policy 决定怎么结束。也就是说，policy -> outcome -> finish 这条链已经闭合。
33. `JointPreparedOutcome` 现在也已经不再只是“语义对齐”的壳，而是开始承载真实的 runtime 区分：channel prepare 层当前已经能直接表达
   - prepared 阶段表达“该 row 是否不适用”
   - prepared 阶段表达“该 row 是否适用但 prepare 卡住”
   - solve 阶段表达“拿到 prepared 后最终应该如何结束本 row”
   当前 `stalled_prepare` 也已经不再固定映射到 `JointSolveResultOutcome.stalled`，而是会继续通过 `descriptor.policy.stalled_prepare_outcome` 收口；`inactive` 与 `stalled_prepare` 两条非 ready 路径都已经进入 runtime policy，而不会再混成同一个 unavailable 分支。
34. joint runtime solver wrapper 这层现在也已经基本被 descriptor 吃掉：`JointRuntimeRowDescriptor` 当前不再持有每类 row 各自的 `solver`，而是已经进一步收口为直接持有：
   - `prepare_channel`
   - `row_enabled`
   - `policy`
   `solveJointLimitRow(...)` / `solveJointAnchorRow(...)` / `solveJointDriveRow(...)` 三个专用 runtime solver 已被收口成单一的 `solveJointGenericRow(...)`。当前 generic solver 只负责：
   - execution-state 初始化
   - 调用 descriptor 的 `prepare_channel(...)`
   - 把结果统一交给 `finishJointSolveResultOutcome(...)`
35. 这意味着 joint runtime solve 壳已经进一步接近“纯协议执行器”，三类 joint row 真正剩下的差异已经主要被压缩到 descriptor entry 上的：
   - `policy`
   - `prepare_channel`
   - `row_enabled`
   而不是再散落在三套 solver wrapper 内部。
36. prepared 协议现在也已经真正下沉到 channel 层本身：`prepareJointLimitChannel(...)`、`prepareJointAnchorChannel(...)`、`prepareJointDriveChannel(...)` 当前不再返回裸 `?channel`，而是直接返回 `JointPreparedOutcome(...)`。这意味着 prepared 阶段的 protocol 已经不再依赖外层的 optional-wrapper helper，channel prepare 自身就能明确表达 `ready / inactive / stalled_prepare`。
37. 这一步的意义是：joint 的“语义 prepare”与“prepared protocol”开始真正贴合，而不是两层分离。当前已经有一部分本应支持的 row 在 prepare helper 返回空时会上抛为 `stalled_prepare`，而明确不支持的 joint 类型才继续返回 `inactive`。
38. 在这一步之后，`JointRuntimeRowPolicy` 已经同时显式承载：
   - `inactive_outcome`
   - `stalled_prepare_outcome`
   这意味着 descriptor 现在已经能分别控制“不适用”和“prepare 卡住”两类非 ready 路径的收口路线，而不是只管理其中一类。
39. 当前新增的定向测试也已经把这层 policy 行为锁住：
   - `prepareJointDriveChannel distinguishes inactive from stalled_prepare`
   - `prepareJointDriveSolveResult maps stalled_prepare to stalled outcome`
   - `stalled prepared joint outcome follows descriptor policy`
40. 在这之后，`prepareJointLimitSolveResult(...)` / `prepareJointAnchorSolveResult(...)` / `prepareJointDriveSolveResult(...)` 里原本重复的 prepared-outcome 分支模板也已经收口到共享 helper：新增 `JointPreparedResolution(T)` 与 `resolveJointPreparedChannel(...)`。当前三条路径已经不再各自手写：
   - `ready => prepared`
   - `inactive => policy outcome`
   - `stalled_prepare => policy outcome`
   而是统一先做 prepared-resolution，再只保留各自的 prepared -> solve-result 转换逻辑。
41. 这一步的意义不只是减少样板，而是把 joint runtime 的 prepared 协议进一步拆成了稳定的两层：
   - channel prepare 层负责产出 `JointPreparedOutcome(...)`
   - resolution 层负责把 prepare outcome 解释成 `ready prepared` 或明确 `JointSolveResultOutcome`
   后续如果继续加入 telemetry 或更细的 prepare-failure 分类，就不需要再同时改三条 `prepareJoint*SolveResult(...)`。
42. 当前这层共享 resolution 也已经补上定向测试：
   - `resolveJointPreparedChannel maps inactive and stalled via descriptor policy`
43. 在这之后，`prepareJointLimitSolveResult(...)` / `prepareJointAnchorSolveResult(...)` / `prepareJointDriveSolveResult(...)` 又进一步退化成统一模板调用：新增 `prepareResolvedJointSolveResult(...)` 与每类 row 自己的 `buildReadyJoint*SolveResult(...)`。当前三条路径已经不再自己负责：
   - 调 resolution helper
   - 包装 `.{ .ready = ... }`
   - 选择 `return outcome` 还是继续 ready build
   而是统一通过 “prepared outcome + ready builder” 模板执行。
44. 这一步的意义是：joint `prepare_solve_result` 这一层现在也已经正式拆成两段稳定协议：
   - 通用模板负责 `prepare outcome -> solve outcome`
   - row-specific builder 只负责 `prepared payload -> JointPreparedSolveResult`
   这样下一步如果要继续把 row-specific 差异压进 descriptor 或 metadata，改动面已经显著缩小。
45. 当前又进一步推进了一层：旧的三条 `prepareJoint*SolveResult(...)` wrapper 已退出主路径，descriptor 现在直接提供 `prepare_channel(...)`，而共享 `buildReadyJointSolveResultFromPreparedChannel(...)` 会统一消费 `JointPreparedChannel`。这意味着 runtime 主路径已经不再依赖 row-specific solve-result wrapper，descriptor 本体也进一步缩回最小执行元数据。
46. 因此，下一阶段 joint runtime 真正值得继续推进的点，已经进一步聚焦到：
   - 是否继续把 finalize / predictive / outcome metadata 压进 `JointRuntimeRowPolicy`
   - 是否继续减少 `buildReadyJointAnchorSolveResult(...)` / `buildReadyJointLimitSolveResult(...)` / `buildReadyJointDriveSolveResult(...)` 中仅剩的 row-specific 语义分支
   - 是否开始把 prepared stall 的原因细分成可观测 telemetry，而不是只保留一个总的 `stalled_prepare`

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
2. 继续把 contact 从当前的 pair execution shell 往 pair-level finalize / residual / policy 收口，减少 subsystem 专有 finish 分支
3. 再把 joint anchor / limit / drive 对齐到更明确的共享 linear / angular / motor target channel
4. 然后把预测性从 bias/priority shaping 升级为显式 `PredictiveRowPlan`
5. 最后再推进更通用 jacobian row primitive

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

## 2026-04-26 增量更新

### joint unavailable outcome 再收口

本轮增加了一个小但明确的协议清理：

1. 删除了 joint runtime 里仅做转发的 `preparedJointSolveResultOutcome(...)`
2. `resolveJointPreparedChannel(...)` 现在直接落到 `jointPreparedUnavailableOutcome(...)`
3. 对应测试不再构造无意义的 runtime ctx / joint / instance，只验证 policy 到 outcome 的单点映射

这一步没有改变求解行为，但继续减少了 joint runtime 的假依赖和薄包装层。

收益是：

1. joint unavailable policy 映射彻底收敛到单点
2. joint prepare-resolution 协议更接近 contact/environment 当前的“直接 outcome 映射”风格
3. 下一步如果要继续抽 shared row finish / unavailable protocol，joint 侧噪音会更少

### joint descriptor prepare 再去重

joint descriptor prepare 之前还保留了三段重复的 outcome 映射壳：

1. `prepareJointDescriptorLimitChannel(...)`
2. `prepareJointDescriptorAnchorChannel(...)`
3. `prepareJointDescriptorDriveChannel(...)`

它们的共同形状都是：

1. 调底层 `prepareJoint*Channel(...)`
2. `ready` 时做一层 descriptor 级包装或 postprocess 注入
3. `inactive/stalled_prepare` 原样透传

这一轮把这段模式收成了共享 helper：

1. `MapJointPreparedOutcomeFn(...)`
2. `mapJointPreparedOutcome(...)`

然后每类 channel 只保留自己的 ready 映射函数：

1. `mapJointDescriptorLimitChannel(...)`
2. `mapJointDescriptorAnchorChannel(...)`
3. `mapJointDescriptorDriveChannel(...)`

这一步同样不改物理行为，只继续把 joint descriptor prepare 往“共享 outcome 协议 + 最小 ready 变换”靠拢。

### joint ready finish 再压平

joint finish 路径里之前还留着一个很薄的 ready wrapper：

1. `solveReadyJointResult(...)`

它只做两件事：

1. `solve_result.applyAll(...)`
2. `finalizeJointExecutionState(...)`

这一轮已经把这层直接内联进：

1. `finishJointSolveResultOutcome(...)`

收益不大，但方向明确：

1. joint finish 协议更接近 contact/environment 当前的“ready outcome -> execute/finalize”形状
2. 后续如果要继续比较三类 subsystem 的 row finish 协议，joint 侧中间壳更少

### contact settle-after-solve 分支收口

contact finish 路径里 row/batch 之前各自保留了一份相同的：

1. `if (descriptor.settle_after_solve) settleContactExecutionContext(...)`

这一轮把它们收进：

1. `finishSettledContactPairChanged(...)`

现在：

1. row 路径继续先选 `solve_step`
2. batch 路径继续先取 `pair_result.changed()`
3. settle/wake 是否执行由同一个 helper 收口

这仍然不是大的协议重构，但继续减少了 contact finish 内的重复控制流，为后续比较 joint/contact/environment 的 finalize 形状提供了更干净的基线。

### shared solve-step finalize 壳开始出现

这一轮继续往 finalize 层补一小步共享协议：

1. 新增 `finalizePairSolveStepResult(...)`
2. 新增 `finalizeSingleSolveStepResult(...)`

它们的职责很单一：

1. 接收已经测得的 `measure_after`
2. 接收 `ConstraintSolveStep`
3. 统一把 `solve_step.changed / solve_step.applied_impulse` 落到 pair/single finalize 壳

当前接入点：

1. `finalizePreparedContactRowResult(...)`
2. `finalizePreparedEnvironmentRowResult(...)`

意义在于：

1. subsystem row finalize 开始从“拆字段后再 finalize”转向“保留 solve_step 语义直到最后一层”
2. 这为后续继续比较 joint/contact/environment 的 execute/finalize 协议，提供了更一致的末端形状

### joint finalize 也开始对齐 solve-step 语义

在上一轮 contact/environment 已经开始直接走 shared `solve_step -> finalize` 壳之后，这一轮把 joint 也往同一个方向推了一步：

1. `JointRowExecutionState` 新增：
   - `solveStep()`
2. `finalizeJointExecutionState(...)` 现在不再直接拆：
   - `changed`
   - `applied_impulse`
   而是先投影到 `ConstraintSolveStep`
3. joint 旧的中间壳：
   - `finalizeJointRowResult(...)`
   已经删除

这意味着 joint/contact/environment 三类 row finalize 的末端表达，开始逐步收敛到同一套 `ConstraintSolveStep` 语义，而不是继续各自保留 subsystem 私有的 changed/impulse 传递方式。

### contact/environment execution outcome mapper 再收口

contact 和 environment 之前各自都保留了两段同形 mapper：

1. runtime solve context mapper
2. batch solve context mapper

共同模式都是：

1. `inactive -> inactive`
2. `stalled -> stalled`
3. `ready -> 做一层 ready context 包装`

这一轮把它们分别收成 subsystem 内的共享 helper：

1. `mapContactExecutionOutcome(...)`
2. `mapEnvironmentExecutionOutcome(...)`

并配套引入各自的 ready builder：

1. contact
   - `buildContactRuntimeSolveContext(...)`
   - `buildContactBatchSolveContext(...)`
2. environment
   - `buildEnvironmentRuntimeSolveContext(...)`
   - `buildEnvironmentBatchSolveContext(...)`

这样做的收益是：

1. runtime/batch context mapper 不再各自保留重复三态 switch
2. ready 分支差异被限制在最小的 context build 函数里
3. 后续如果还要继续抽“prepared execution outcome -> runtime/batch context”协议，接缝会更清晰

### contact descriptor-plan solve wrapper 再压平

contact 之前还保留了两层很薄的 solve wrapper：

1. `solvePreparedContactRuntimeRow(...)`
2. `solvePreparedContactBatch(...)`

它们本质上只做一件事：

1. `descriptor.build_plan(...)`
2. 然后把 plan 交给 `solvePreparedContactPlan(...)`

这一轮已经把这两层删除，并让：

1. `executePreparedContactRuntimeRow(...)`
2. `executePreparedContactBatch(...)`

直接调用 shared `solvePreparedContactPlan(...)`。

这一步的价值仍然主要在协议层：

1. contact row/batch 的 solve 主体继续向单点 `solvePreparedContactPlan(...)` 收拢
2. row/batch 差异进一步缩减为“如何构造 plan”，而不是再保留中间 solve wrapper

### joint descriptor-level prepare wrapper 再删除一层

joint 这里之前还保留了一个单用途薄包装：

1. `prepareJointSolveResultFromDescriptor(...)`

它本质上只是把：

1. `ctx.descriptor.prepare_channel(ctx)`
2. `buildReadyJointSolveResultFromPreparedChannel`

接到 shared `prepareResolvedJointSolveResult(...)` 上。

这一轮已经把这层删除，让：

1. joint 真实执行路径
2. 对应测试路径

都直接调用 `prepareResolvedJointSolveResult(...)`。

这样 joint 的 prepare protocol 又减少了一层中间壳，更接近 contact 当前“直接用 shared plan/solve helper”的形状。

### joint finish 的 unready outcome 再收口

joint finish 这边之前虽然 ready 分支已经很薄，但：

1. `no_change`
2. `finalize_exec_state`
3. `stalled`

这三类 unready outcome 仍然直接摊在 `finishJointSolveResultOutcome(...)` 里。

这一轮把它们收进：

1. `finishUnreadyJointSolveResultOutcome(...)`

结果是：

1. `finishJointSolveResultOutcome(...)` 更明显地变成
   - ready execute/finalize
   - unready finalize dispatch
2. joint finish protocol 结构更接近 contact 当前“ready path / stalled-or-other path”分层

同时，对应测试名称也已和真实 shared helper 对齐：

1. `prepareResolvedJointSolveResult maps stalled_prepare to stalled outcome`

避免继续保留过时 wrapper 名称干扰后续收口。

### joint ready path 对齐到 execute helper 形状

为了继续和 contact/environment 的执行路径对齐，这一轮把 joint 的 `.ready` 分支也显式提成了：

1. `executePreparedJointSolveResult(...)`

职责保持不变：

1. `solve_result.applyAll(...)`
2. `finalizeJointExecutionState(...)`

但协议层价值比较明确：

1. `finishJointSolveResultOutcome(...)` 现在更接近
   - ready -> execute helper
   - unready -> finalize dispatch
2. joint/contact/environment 三类 runtime row 在命名和结构上进一步靠拢到同一套 `executePrepared...` / `finish...` 语义

### contact/environment 高层 outcome dispatch 再收口

在 runtime/batch context mapper 已经统一之后，这一轮继续把更外层的 outcome dispatch 从调用点挪走：

1. contact
   - `executePreparedContactRuntimeOutcome(...)`
   - `executePreparedContactBatchOutcome(...)`
2. environment
   - `executePreparedEnvironmentRuntimeOutcome(...)`
   - `executePreparedEnvironmentBatchOutcome(...)`

这样：

1. `solveContactRuntimeRow(...)`
2. `solveEnvironmentRow(...)`
3. contact/environment 的 batch loop

都不再内联：

1. `inactive`
2. `stalled`
3. `ready`

这类三态 switch，而是把它们继续收进 execute-outcome helper。

收益是：

1. 高层 row/loop 更专注于编排
2. outcome dispatch 不再散落在 batch loop 和 runtime row 入口
3. contact/environment 和 joint 当前的“prepare -> execute helper -> finish helper”分层更接近

### directional residual 基元开始共享

在协议壳已经基本收口之后，这一轮开始转去一个更高价值的底层缺口：共享 directional residual 数学基元。

新增：

1. `DirectionalConstraintChannel`
2. `DirectionalConstraintMetrics`

当前效果：

1. `ContactConstraintMetrics` 与 `EnvironmentConstraintMetrics` 现在共享同一底层 metrics 结构
2. `contact_normal` / `contact_friction` 的 residual 计算不再散写公式，而是统一走：
   - `metrics.residual(.normal)`
   - `metrics.residual(.tangent)`
3. `environment` 的 stress 计算也开始复用同一组 directional residual 数学基元
4. 这一步还没有把 `environment` 的最终优先级策略完全改成和 `contact` 同一入口，因为它还叠加了 predictive gain；但至少共同的方向型速度/残差数学已经不再重复维护

### displacement channel apply 开始共享更高层入口

继续沿着底层缺口推进，这一轮把 `single-body displacement` 和 `pair-body displacement` 提升到了同一层 directional apply 入口：

新增：

1. `DirectionalDisplacementBodies`
2. `applyDirectionalDisplacementPrimitive(...)`

当前效果：

1. `applySingleDisplacementRowStep(...)` 不再自己维护位移 apply 逻辑，而是转到共享 directional displacement primitive
2. `applyPairDirectionalDisplacementRowStep(...)` 也改为复用同一入口
3. `single` 和 `pair` 当前仍保留各自的 body 载荷差异，但共同的：
   - solve magnitude
   - clamp / slop 入口
   - displacement apply 返回壳
   已经开始在更高一层统一
4. 这一步还没有把所有 displacement/channel 都压成完全同一接口，但已经把文档里提到的“single-body displacement 和 pair-body displacement 仍分两套 apply 接口”往前推进了一步

### directional velocity 分解开始共享

为了继续往 contact friction / tangent 的更通用 directional channel 方向推进，这一轮先把更底层的方向速度分解数学抽成共享基元：

新增：

1. `DirectionalVelocityComponents`
2. `measureDirectionalVelocityComponents(...)`

当前效果：

1. `contact` metrics 不再自己手工拆：
   - relative normal speed
   - tangent vector
   - tangential speed
2. `contact` prepare path 里的 tangent 方向与 normal speed 也改为复用同一分解结果
3. `environment` metrics 同样开始复用这层方向速度分解
4. 这一步还没有把 friction solver 本身泛化成完整 tangent channel protocol，但已经把 contact/environment 共用的“方向速度拆分数学”从多个调用点里抽了出来

### friction tangent frame 开始共享

继续沿着 contact friction 的低层重复点推进，这一轮把“方向速度分解之后的切线帧提取”收敛成共享 helper。

新增：

1. `DirectionalTangentFrame`
2. `buildDirectionalTangentFrame(...)`

当前效果：

1. `prepareContactConstraintPair(...)` 不再自己手工处理：
   - tangent length
   - tangent normalize
   - tangential speed
   而是改为复用统一的 tangent frame 提取结果
2. `applyContactFrictionRowStep(...)` 也改为在 solve 阶段复用同一套 directional velocity -> tangent frame 基元，不再重复维护一份局部切线归一化逻辑
3. 这一步没有改 contact friction 的约束方程、clamp 策略和 impulse 上限，只是把准备阶段与求解阶段共同依赖的切线方向抽取逻辑统一到底层 helper
4. 这样后续如果继续推进 tangent channel 共享，可以直接建立在：
   - `measureDirectionalVelocityComponents(...)`
   - `buildDirectionalTangentFrame(...)`
   这两层基元之上，而不用再回头收拾 prepare/solve 两侧各自一套的切线数学

### contact friction velocity apply 入口继续收口

这一轮继续沿着 contact friction 的低层路径收口，但仍然只处理 apply 层，不改求解公式。

当前调整：

1. 新增统一的摩擦 velocity apply helper：
   - `applyContactFrictionRowStep(...)`
2. 原先负责实时摩擦解算的入口改成：
   - `solveContactFrictionRowStep(...)`
3. `applyContactFrictionWarmStartRowStep(...)` 与普通 friction solve 现在都复用同一个 tangent velocity primitive apply 入口

当前效果：

1. friction warm start 与 friction solve 不再各自构造一遍：
   - tangent direction velocity primitive
   - pair velocity impulse apply
2. 两条路径当前只保留各自的 impulse 计算差异：
   - 普通 friction solve 负责从当前切向速度推导阻尼冲量
   - warm start 负责从累计冲量和切向速度符号恢复预热冲量
3. 这样 contact friction 路径的分层开始更清晰：
   - directional velocity/tangent frame
   - impulse derivation
   - shared tangent velocity apply
4. 这一步仍然没有调整 friction equation、max impulse、warm retention 或 residual 规则，行为保持不变

### friction tangent residual 基元开始收口

这一轮继续沿着 contact friction 的 build 层收口，但仍然没有动 friction equation 和 predictive 策略。

新增：

1. `buildDirectionalTangentResidual(...)`

当前效果：

1. `buildContactFrictionRowPlan(...)` 不再自己手工写 `direction.depth * 0.125`
2. `buildDirectionalTangentRowSpec(...)` 也改为复用同一 tangent residual helper
3. contact friction 的 prepare/build 与 row-spec build 至少开始共用同一条 tangent residual 基元，避免后续继续出现多处散写同一残差公式
4. 这一步仍然没有把 friction row plan 强行接到 predictive directional row plan；当前保留原有 friction residual 语义，只先把共享基元收口

### directional row spec / plan 的 residual 入口继续统一

这一轮继续推进 build 层收口，重点不是改 predictive 策略，而是把 normal/tangent 的 residual row 组装入口统一到底层 helper。

新增：

1. `buildDirectionalNormalResidual(...)`
2. `buildDirectionalResidualRowPlan(...)`

当前效果：

1. `buildDirectionalNormalRowSpec(...)` 不再自己内联 `@max(direction.predicted_depth, equation.bias * 1.25)`
2. `buildDirectionalTangentRowSpec(...)` 不再自己手工拼 residual/equation 结构，而是走共享 `buildDirectionalResidualRowPlan(...)`
3. `buildContactFrictionRowPlan(...)` 也改为复用同一个 residual-row helper
4. 这样现在至少有三条 build 路径开始共用同一层 directional residual row 入口：
   - normal row spec
   - tangent row spec
   - contact friction row plan
5. 这一步仍然保留 contact normal 的 predictive directional row plan，以及 friction 当前非 predictive 的 residual 语义；只是让 row-plan / row-spec 的组装入口不再散写

### row spec 的薄包装继续裁掉

这一轮继续收口 build 层的薄包装 helper，目标是让 directional row spec 和 row plan spec 更直接落到共享入口。

当前调整：

1. 删除两层只差 mode 的薄包装：
   - `buildDirectionalNormalRowSpec(...)`
   - `buildDirectionalTangentRowSpec(...)`
2. 新增统一入口：
   - `buildDirectionalModeRowSpec(...)`
3. `buildOptionalConstraintRowSpecFromPlan(...)` 现在直接复用：
   - `buildDirectionalRowSpec(...)`

当前效果：

1. `buildOptionalDirectionalRowSpec(...)` 不再维护 normal/tangent 的 switch 分发到两个几乎等价的 helper
2. directional row spec 当前统一变成：
   - payload -> mode -> `buildDirectionalModeRowSpec(...)` -> `buildDirectionalRowSpec(...)`
3. optional row-plan spec 也不再自己手工调用 `makeConstraintRowBuildSpec(...)`，而是直接落到共享 row-spec 入口
4. 这一步仍然只是在消除 build 层薄包装，没有修改 priority/residual/equation 行为

### joint row spec 的单次转发 wrapper 已删除

这一轮继续按“只删单次转发型 wrapper”的原则推进，目标是减少 row-spec build callsite 上没有独立价值的中间层。

当前调整：

1. 删除 `buildJointRowSpecFromPlan(...)`
2. `buildJointRowSpecForJoint(...)` 现在直接调用：
   - `buildOptionalConstraintRowSpecFromPlan(...)`

当前效果：

1. joint row-spec build 不再经过一层只负责传递 `joint_idx` 和 `row_plan` 的薄包装
2. joint/contact/environment 当前在 row-spec build 层更接近统一：
   - joint 直接落到 shared optional row-plan spec helper
   - contact 直接落到 shared directional optional row-spec helper
   - environment 直接落到 shared optional row-plan spec helper
3. 这一步仍然没有改 joint row enable 判定、row plan 生成、priority 或 residual 语义

### environment penetration probe 开始共享

这一轮从命名层收口转回到底层 prepare/build 数学共享，先处理 `environment` 路径里重复最重的一段：实例校验 + AABB + penetration query。

新增：

1. `EnvironmentPenetrationProbe`
2. `probeEnvironmentPenetration(...)`

当前效果：

1. `measureEnvironmentConstraintMetrics(...)` 不再自己重复：
   - instance/entity 有效性检查
   - world AABB 计算
   - `query_penetration.computePenetrationAABB(...)`
2. `prepareEnvironmentConstraint(...)` 也改为复用同一个 probe helper
3. 现在 `environment` 的 metrics 路径和 prepare 路径至少共用同一份 penetration probe 入口，后续如果继续收 `predictive depth` 或 move/resolution 派生逻辑，会更容易建立在这层 probe 之上
4. 这一步没有改 penetration filter、move quantize、predictive depth 或 environment equation 行为，只是去掉重复查询前半段

### contact constraint probe 开始共享

这一轮把同样的 probe 共享思路推进到 `contact` 路径，处理 `measureContactConstraintMetrics(...)` 和 `prepareContactConstraintPair(...)` 前半段重复较重的问题。

新增：

1. `ContactConstraintProbe`
2. `probeContactConstraint(...)`

当前效果：

1. `measureContactConstraintMetrics(...)` 不再自己重复：
   - entity 有效性检查
   - world AABB 计算
   - `collision.buildAABBContactManifold(...)`
   - directional velocity 分解
2. `prepareContactConstraintPair(...)` 也改为复用同一个 probe helper
3. 现在 `contact` 的 metrics 路径和 prepare 路径开始共用：
   - AABB overlap probe
   - contact manifold
   - relative directional velocity components
4. 这一步没有改材质配对、预测重叠深度、normal/tangent row plan、摩擦方程或 restitution 行为，只是去掉共同的查询/测量前半段重复

### contact future AABB overlap 深度开始共享

这一轮继续沿着 `contact` prepare 的低层数学共享推进，把未来重叠深度那段长公式收成单一 helper。

新增：

1. `computePredictedAABBOverlapDepth(...)`

当前效果：

1. `prepareContactConstraintPair(...)` 不再自己维护一整段：
   - predicted min/max AABB 边界展开
   - predicted overlap xyz 计算
   - predicted penetration depth 取最小重叠
2. contact 预测重叠深度现在落到单一 helper，后续如果 environment 或别的预测接触路径也需要 future AABB overlap，可以直接复用同一基元
3. 这一步没有改 prediction dt、AABB 外扩方式、normal predicted depth 语义或 friction/tangent 逻辑，只是把长公式收口

### instance -> linear state 预测输入开始共享

这一轮继续沿 predictive 路径推进，但选了一个更底层、更稳的共享点：`scene32.Instance` 到 `prediction.LinearState` 的转换。

新增：

1. `prediction.linearStateFromInstance(...)`

当前效果：

1. `prepareContactConstraintPair(...)` 里两处 `predictLinearState(...)` 不再各自手工拼 `LinearState`
2. `measurePredictiveEnvironmentDepth(...)` 也改为复用同一个 helper
3. contact predictive 和 environment predictive 现在至少共用同一层预测输入基元，减少位置/速度字段展开的重复维护
4. 这一步没有改 prediction dt、未来 AABB 展开、penetration query 或任何 solver 行为，只是统一预测输入构造

### environment AABB penetration query 本体继续共享

这一轮继续沿 `environment` predictive 路径推进，把 penetration query 本体再抽了一层，让当前 probe 与未来 probe 共用同一入口。

新增：

1. `queryEnvironmentPenetrationForAABB(...)`

当前效果：

1. `probeEnvironmentPenetration(...)` 不再自己维护：
   - world view 组装
   - `query_penetration.computePenetrationAABB(...)`
   - penetration 结果过滤
2. `measurePredictiveEnvironmentDepth(...)` 也改为复用同一个 AABB penetration query helper，只是传入预测后的 `dx/dy/dz` 偏移
3. 现在 environment 的当前 probe 与未来 probe 至少共享同一层 query 本体，后续如果还要收 offset 计算或 future query policy，会更容易继续推进
4. 这一步没有改 penetration filter、预测位移量化或 predictive gain 语义，只是收口重复 query 本体

### effective mass 倒数基元开始共享

这一轮回到 equation/build 路径，先处理 contact/environment/joint 都在重复的最基础一层：inverse mass 到 effective mass 的倒数逻辑。

新增：

1. `effectiveMassFromInverseMass(...)`
2. `effectiveMassFromPairInverseMasses(...)`

当前效果：

1. `buildContactNormalEquation(...)` 不再自己计算 `1 / (inv_mass_a + inv_mass_b)`
2. `buildContactFrictionEquation(...)` 也改为复用同一 pair effective mass helper
3. `buildEnvironmentEquation(...)` 改为复用 single effective mass helper
4. `buildJointEquation(...)` 与 `buildJointDriveEquation(...)` 也开始共用同一 pair effective mass helper
5. 这一步没有改 bias/max impulse 公式或任何 solver 行为，只是把 contact/environment/joint 共用的倒数基元统一到底层 helper

### contact/environment normal equation 骨架开始共享

这一轮继续沿 equation/build 路径收口，但只处理 contact normal 与 environment normal 之间真正重复的一层：penetration bias + closing-speed bias + max impulse shaping 的通用骨架。

新增：

1. `buildNormalConstraintEquation(...)`

当前效果：

1. `buildContactNormalEquation(...)` 改为只负责提供 pair effective mass 和本行的四个 scale
2. `buildEnvironmentEquation(...)` 改为只负责提供 single-body effective mass 和本行的四个 scale
3. contact/environment 的 normal equation 不再各自重复：
   - `penetration_depth * scale`
   - `max(0, -relative_normal_speed) * scale`
   - `max_impulse` 下限与深度/速度混合
4. 这一步没有把 friction equation 强行并入同一个 helper，也没有改 predictive depth、row plan 或 solver 行为；只把真正共享的 normal equation 骨架压到底层 helper
5. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### predictive gain 开始升级成最小 policy metadata

这一轮不再只是命名几个 scale helper，而是把 predictive gain 的输入参数第一次收成了一个显式 metadata 壳。

新增：

1. `PredictiveConstraintPolicy`
2. `makePredictiveConstraintGainWithPolicy(...)`
3. `environmentPredictiveConstraintPolicy()`
4. `contactNormalPredictiveConstraintPolicy()`

当前效果：

1. `buildDirectionalRowPlan(...)` 不再接收三段散传的：
   - `residual_scale`
   - `bias_scale`
   - `impulse_scale`
   而是改为接收一个 `PredictiveConstraintPolicy`
2. `buildContactNormalRowPlan(...)` 与 `buildEnvironmentDirectionalRowPlan(...)` 都改成传 policy，而不是继续散传 3 个常量
3. `environmentPredictiveConstraintGain(...)` 也已经改为复用同一个 policy 入口

这一步的意义：

1. predictive gain 终于从“共享数学函数 + 若干散写常量”提升成了一个最小可传递的 metadata 结构
2. 这为下一阶段把 predictive row metadata 继续扩成：
   - horizon
   - urgency
   - correction budget
   - telemetry
   留下了真实的结构化入口
3. 当前仍然保持克制，没有引入完整 `PredictiveRowPlan`，也没有改变 solver 或策略数值，只是先把最稳定的一层参数包起来
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### PreparedDirectionalConstraint 也开始区分原始预测深度与策略化 hint

上一轮已经把 `DirectionalRowPlan` 的最终 residual 与 predictive hint 拆开，这一轮继续把同样的语义边界前推到 prepared directional payload 本身。

新增：

1. `PreparedDirectionalConstraint.predictive_residual_hint`

当前效果：

1. `PreparedDirectionalConstraint.predicted_depth` 恢复只表示“原始预测深度/原始预测量”
2. `PreparedDirectionalConstraint.predictive_residual_hint` 单独表示“经过 predictive policy shaping 后的 residual hint”
3. `prepareContactConstraintPair(...)` 不再把 `normal_row_plan.predictive_residual_hint` 覆盖写回 `predicted_depth`
   而是：
   - `predicted_depth = normal_direction.predicted_depth`
   - `predictive_residual_hint = normal_row_plan.predictive_residual_hint`
4. `buildDirectionalNormalResidual(...)` 现在优先消费 `predictive_residual_hint`
   如果该值未提供，再回退到 `predicted_depth`

这一步的意义：

1. predictive 数据流现在已经真正拆成三层：
   - 原始预测量
   - policy 后的 predictive hint
   - 最终 row residual
2. contact 不再需要复用 `predicted_depth` 字段承载两种不同语义
3. 后续如果要继续做 predictive telemetry / debug / row plan，可直接沿这三层展开，而不是再追查“某个字段当前到底装的是原始深度还是策略化 residual”
4. 这一步没有改变 solver 行为，只是把 predictive metadata 的边界再拉清一层
5. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### row plan 开始区分最终 residual 与 predictive hint

这一轮继续往 `predictive metadata` 走了一小步，但重点不是再加新 policy，而是修正一个已经开始显现的语义混用。

新增：

1. `DirectionalRowPlan.predictive_residual_hint`

当前效果：

1. `DirectionalRowPlan.residual` 继续表示“最终用于 row 调度/优先级的 residual”
2. `DirectionalRowPlan.predictive_residual_hint` 单独保留 predictive gain 产出的原始 residual hint
3. `buildDirectionalRowPlan(...)` 现在会同时写入：
   - `residual = max(predictive_hint, equation.bias * 1.25)`
   - `predictive_residual_hint = predictive_hint`
4. `prepareContactConstraintPair(...)` 不再把 `normal_row_plan.residual` 回写到 `PreparedDirectionalConstraint.predicted_depth`
   而是改为回写 `normal_row_plan.predictive_residual_hint`

这一步的意义：

1. predictive 语义开始和最终调度残差解耦，不再把两者混成一个字段
2. 这让后续如果要继续扩：
   - predictive telemetry
   - row debug 可视化
   - 更细的 priority/source attribution
   会更容易，因为“预测提示值”和“最终调度值”已经分家
3. 这一步没有改变 contact/environment/joint 的求解行为，只是把 plan metadata 的语义边界拉清
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### contact normal predictive scaling 也开始显式命名

在把 `environment` 的 predictive scaling 收到单点入口之后，这一轮没有强行做“对称式大抽象”，只把 `contact normal` 那组目前仍然只出现一处的策略参数显式命名。

新增：

1. `contactNormalPredictiveResidualScale()`
2. `contactNormalPredictiveBiasScale()`
3. `contactNormalPredictiveImpulseScale()`

当前效果：

1. `buildContactNormalRowPlan(...)` 不再直接散写：
   - residual `0.75`
   - bias `0.15`
   - impulse `0.5`
2. 这一步的目标不是立刻给 `contact` 也补完整 `PredictiveConstraintGain` policy 壳，而是先把这组 row-plan 策略从裸数字里拉出来，避免后续扩到 priority / telemetry / config 时继续追硬编码
3. 这样做的好处是保持当前抽象层级克制：`environment` 因为已有双路径共用，值得收成完整 policy 入口；`contact` 目前只有单路径使用，所以只先命名，不做过度结构化
4. 这一步没有改变 contact normal 的 predictive gain 数值或 solver 行为，只是把策略参数显式化
5. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### 统一预测 horizon 常量入口

这一轮继续沿 predictive 路径做底层收口，但仍然不引入完整 `PredictiveRowPlan`，只先统一短时预测 horizon 的入口。

新增：

1. `kernelPredictionDt()`

当前效果：

1. joint spring 预测延伸
2. joint drive 预测 braking/step 规划
3. contact 预测 future overlap
4. environment 预测 future penetration

这些路径里原本散落的 `1.0 / 60.0` 已统一改为 `kernelPredictionDt()`。

这一步的意义：

1. predictive 接入点开始共享同一 horizon 来源，后续如果要把固定 horizon 升级成 policy / config / row-plan metadata，改动面会明显更集中
2. 当前还没有改变任何预测公式、brake 判定或 overlap/depth 语义，只是去掉硬编码常量散布
3. 这一步是从“预测性已接入”继续走向“预测性有统一底层入口”的小前置动作
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### environment predictive scaling policy 开始集中

这一轮第一次把一组真正的 predictive policy 常量，从“散落的调用点参数”下沉成了底层显式入口。

新增：

1. `environmentPredictiveResidualScale()`
2. `environmentPredictiveBiasScale()`
3. `environmentPredictiveImpulseScale()`
4. `environmentPredictiveConstraintGain(...)`

当前效果：

1. `buildEnvironmentDirectionalRowPlan(...)`
2. `computeEnvironmentSolvePriorityMagnitude(...)`

这两条路径原本分别散写的同一组 predictive scaling：

- residual `0.75`
- bias `0.2`
- impulse `0.5`

现在统一从同一组 helper 入口读取。

这一步的意义：

1. predictive 终于不只是共享数学 helper，也开始共享“哪组策略参数属于哪个子系统”的 metadata 入口
2. row-plan 路径与 priority 路径不会再因为并行散写常量而发生静默漂移
3. 这是后续把 fixed predictive gain 升级成更明确 `PredictiveRowPlan` / policy 对象的直接前置动作
4. 这一步没有改变 environment 的 predictive gain 数值或 solver 行为，只是把策略参数集中
5. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### 预测输入基元继续收口到实例级 helper

这一轮继续沿 predictive 输入层做小步共享，把 `scene32.Instance -> prediction.LinearState -> predictLinearState(...)` 这条链再压了一层。

新增：

1. `predictKernelInstanceState(...)`

当前效果：

1. contact 预测 future overlap
2. environment 预测 future penetration

这两条路径里原本重复的：

- `prediction.linearStateFromInstance(inst)`
- `prediction.predictLinearState(..., dt)`

现在统一改为 `predictKernelInstanceState(inst, dt)`。

这一步的意义：

1. predictive 输入基元开始从“统一 horizon 常量”继续推进到“统一实例预测入口”
2. 后续如果要给 predictive row plan 增加统一的量化/裁剪/telemetry，改动面会继续缩小
3. 这一步没有改变 contact/environment 的预测位姿、predicted depth 或 overlap 算法，只是把输入转换模板统一到底层 helper
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### joint 零方程返回开始共享

这一轮继续在 equation/build 层做很小的收口，目标不是泛化 joint equation，而是去掉几处完全相同的“提前失效直接返回零方程”模板。

新增：

1. `zeroConstraintEquation()`

当前效果：

1. `buildJointEquation(...)` 在：
   - priority 不活跃
   - joint 索引越界
   - entity 映射越界
   这几种情况下统一返回 `zeroConstraintEquation()`
2. `buildJointDriveEquation(...)` 在：
   - priority 不活跃
   - joint 索引越界
   - motor 未启用或速度无效
   - entity 映射越界
   这些情况下也统一返回 `zeroConstraintEquation()`
3. 这一步没有改变 joint anchor/limit/drive 的 bias 或 max impulse 公式，只是把重复的零方程壳统一到底层 helper，减少后续继续收口 joint equation 时的样板噪音
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### joint drive predictive 空计划壳开始共享

这一轮开始轻微触碰 predictive 入口，但仍然只做行为保持的模板收口，没有提前引入完整 `PredictiveRowPlan`。

新增：

1. `idleKernelJointDrivePlan()`

当前效果：

1. `computeKernelJointDrivePlan(...)` 在两类“预测后应进入刹停/空驱动”路径上不再重复返回：
   - `signed_step = 0.0`
   - `desired_velocity = 0.0`
2. 当前被收口的两类情况是：
   - 已到目标，只剩残余速度，需要 braking
   - 预测位置已到目标，当前不该再继续推进
3. 这一步没有改变 predictive brake 判定、target clamp、speed/torque 限幅或 desired velocity 公式，只是把 joint drive predictive 路径里的空计划返回壳统一到 helper
4. 这也给下一阶段从“predictive gain”走向“predictive plan”留了一个更干净的入口，因为现在至少 `joint drive` 已经显式有了 idle-plan 语义壳
5. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### joint descriptor channel 映射模板继续收口

这一轮没有继续扩 `JointRuntimeRowPolicy` 字段，而是先把 descriptor prepare 层里已经明显重复的一段模板再压掉一层。

新增：

1. `prepareMappedJointDescriptorChannel(...)`

当前效果：

1. `prepareJointDescriptorLimitChannel(...)`
2. `prepareJointDescriptorAnchorChannel(...)`
3. `prepareJointDescriptorDriveChannel(...)`

这三条路径不再各自直接调用 `mapJointPreparedOutcome(...)`，而是统一走 `prepareMappedJointDescriptorChannel(...)`。

这一步的意义：

1. joint descriptor prepare 层现在更明确分成两段：
   - row-specific `prepareJoint*Channel(...)`
   - row-specific `mapJointDescriptor*Channel(...)`
   - shared descriptor-level outcome 映射模板
2. 后续如果继续给 descriptor prepare 层加入 telemetry、policy gate 或 prepare-stage tracing，不需要同时改三条 descriptor wrapper
3. 这一步没有改变 `inactive / stalled_prepare / ready` 语义，也没有改变 anchor/drive postprocess 注入方式，只是继续压模板噪音
4. 改动后已重新通过：
   - `zig test src/physics_kernel.zig`
   - `zig test src/tick_engine.zig`
   - `zig test src/physics_world.zig`

### directional prepared payload 也开始有共享构造入口

这一轮继续做小步收口，没有再扩新的 predictive abstraction，而是把 `PreparedDirectionalConstraint` 的常见构造路径收成了两个最小 helper。

新增：

1. `buildPreparedDirectionalConstraint(...)`
2. `withPredictiveResidualHint(...)`

当前效果：

1. `prepareContactConstraintPair(...)` 不再手工内联 normal/tangent 的 `PreparedDirectionalConstraint` 构造。
2. `prepareContactConstraintPair(...)` 回填 normal predictive hint 时，改为统一走 `withPredictiveResidualHint(...)`，显式保持：
   - `predicted_depth` 仍然是原始预测量
   - `predictive_residual_hint` 单独承载 policy-shaped hint
3. `prepareEnvironmentConstraint(...)` 也改为复用 `buildPreparedDirectionalConstraint(...)`，不再单独再写一份 normal payload 初始化。

这一步的意义：

1. contact / environment 的 directional prepared payload 现在开始共享同一组底层构造入口。
2. 原始预测量与后置 hint 的字段边界继续被硬化，减少后续再把 `predicted_depth` 当成 residual 使用的机会。
3. 这一步仍然是低风险重构，没有引入新的 solver 协议或新的 predictive row abstraction。

### environment priority 也开始复用 directional predictive gain helper

这一轮继续保持小步收口，把 `environment` 的 priority 计算也切回和 directional row plan 同一条 predictive gain 语义。

新增：

1. `makeDirectionalPredictiveConstraintGain(...)`
2. `computeDirectionalPredictiveSolvePriorityMagnitude(...)`

当前效果：

1. `buildDirectionalRowPlan(...)` 不再自己直接从 `direction.predicted_depth / direction.depth / policy` 组装 predictive gain，而是统一走 `makeDirectionalPredictiveConstraintGain(...)`。
2. `computeEnvironmentSolvePriorityMagnitude(...)` 不再单独再写一份：
   - 取 `predicted_depth`
   - 调 `environmentPredictiveConstraintGain(...)`
   - 再取 `residual_hint`
   的流程。
3. `environment` 的 row-plan 路径和 solve-priority 路径现在共享同一个 directional predictive gain helper 入口。

这一步的意义：

1. environment 不再在“构建 row plan”和“计算 priority”两处各自维护一份 predictive gain 输入语义。
2. predictive gain 现在开始同时服务：
   - row residual / bias / impulse shaping
   - solve priority 估计
3. 这一步依然没有引入更大的 `PredictiveRowPlan`，只是把已经重复出现的 directional predictive gain 组装再下沉一层。

### DirectionalRowPlan 也开始统一走单一构造出口

这一轮没有再扩新协议，只把 `DirectionalRowPlan` 的字面量构造统一收到一个最小 helper。

新增：

1. `makeDirectionalRowPlan(...)`

当前效果：

1. `buildDirectionalRowPlan(...)` 改为统一走 `makeDirectionalRowPlan(...)`。
2. `buildDirectionalResidualRowPlan(...)` 也改为统一走 `makeDirectionalRowPlan(...)`，显式把非 predictive 路径的 `predictive_residual_hint` 固定为 `0.0`。
3. `buildJointRowPlan(...)` 与 `zeroDirectionalRowPlan()` 不再各自手写 `DirectionalRowPlan` 字面量，而是复用同一个构造出口。

这一步的意义：

1. `DirectionalRowPlan` 的构造语义开始真正固定到一个 helper，而不是 predictive / non-predictive / joint / zero 几条路径各写一份。
2. 后续如果需要继续给 row plan 增加 metadata 字段，只需要沿一个出口扩，不需要回头扫多处字面量。
3. 这一步仍然是低风险底层收口，没有改变 residual、equation 或 predictive hint 的数值行为。

### plan -> spec 也开始统一走单一出口

这一轮继续压缩 `ConstraintRowBuildSpec` 这一层的重复包装，不改 solver 行为，只收口 `DirectionalRowPlan -> ConstraintRowBuildSpec` 的转换入口。

新增：

1. `makeConstraintRowBuildSpecFromPlan(...)`

当前效果：

1. `buildDirectionalRowSpec(...)` 改为复用 `makeConstraintRowBuildSpecFromPlan(...)`。
2. `buildOptionalConstraintRowSpecFromPlan(...)` 不再单独重复取 `row_plan.residual / row_plan.equation`，而是复用同一个 `plan -> spec` helper。
3. `ConstraintRowBuildSpec` 这一层现在开始有明确的“原始字段构造”和“从 row plan 投影构造”两个固定出口。

这一步的意义：

1. `row plan -> row spec` 的语义不再散落在 optional / non-optional 两条路径中各写一份。
2. 后续如果 `ConstraintRowBuildSpec` 再增加元数据字段，可以沿 `makeConstraintRowBuildSpecFromPlan(...)` 一次性扩散。
3. 这一步仍然没有改变 residual、equation 或 row enable 语义。

### row spec 入口层开始对齐 environment helper

这一轮继续做 row spec 入口层的轻量收口，没有引入新的通用 builder，只把 `environment` 这支从 `buildConstraintRowSpecFromEntry(...)` 里提出成独立 helper。

新增：

1. `buildEnvironmentRowSpecForInstance(...)`

当前效果：

1. `buildConstraintRowSpecFromEntry(...)` 里的 `environment_plan` 分支不再自己内联：
   - 取 environment context
   - 调 plan builder
   - 再调 optional spec builder
2. `environment` 现在和 `joint` 一样，有自己对应的 row-spec 入口 helper。
3. `contact / environment / joint` 三类 row-spec 入口开始更对齐：
   - `buildContactRowSpecForPair(...)`
   - `buildEnvironmentRowSpecForInstance(...)`
   - `buildJointRowSpecForJoint(...)`

这一步的意义：

1. `buildConstraintRowSpecFromEntry(...)` 继续变薄，逐步只保留 entry dispatch，不再混杂具体上下文装配。
2. 后续如果要继续统一 row-spec 入口层，可以在这三个 helper 之间做更小粒度的收口，而不是继续往大 switch 里堆逻辑。
3. 这一步仍然没有改变 row enable、plan builder 或 spec 构造语义。

### optional row spec 包装也开始走统一出口

这一轮继续收口 row-spec 入口层，但只动最小的 optional 包装，不改 builder 分层。

新增：

1. `makeOptionalConstraintRowSpec(...)`

当前效果：

1. `buildOptionalConstraintRowSpecFromPlan(...)` 不再自己直接写 `if (!enabled) return null`，而是统一走 `makeOptionalConstraintRowSpec(...)`。
2. `buildOptionalDirectionalRowSpec(...)` 也改为复用同一个 optional helper。
3. `enabled -> ?ConstraintRowBuildSpec` 这层语义现在开始共享单一出口。

这一步的意义：

1. optional row-spec 包装不再在 plan 路径和 directional 路径各写一份。
2. 后续如果要给“禁用 row”补更多 tracing / debug 行为，可以先沿同一个 optional helper 扩，而不是回头扫多处 early-return。
3. 这一步仍然没有改变 row enable 判断本身，也没有改变 row spec 的字段值。

### directional mode -> row spec 投影也开始固定到单一出口

这一轮继续压缩 `row spec` 这一层的重复投影逻辑，把 `direction + equation + mode -> ConstraintRowBuildSpec` 固定到一个 helper。

新增：

1. `makeConstraintRowBuildSpecFromDirectionalMode(...)`

当前效果：

1. `buildDirectionalModeRowSpec(...)` 改为复用 `makeConstraintRowBuildSpecFromDirectionalMode(...)`。
2. `buildOptionalDirectionalRowSpec(...)` 不再直接重复调用 `buildDirectionalModeRowSpec(...)` 的同组参数投影，而是走同一个 directional-mode helper。
3. `directional payload -> row spec` 的投影入口继续收口。

这一步的意义：

1. `DirectionalRowPayload` 相关的 spec 投影不再在 optional / non-optional 两条链上各保留一份参数展开。
2. 后续如果 `DirectionalRowSpecMode` 再扩或要加 trace 字段，有明确的 directional-mode spec 出口可挂。
3. 这一步仍然没有改变 residual、equation 或 enable 语义。

### joint prepare ready-channel 包装也开始共享 helper

这一轮从 `row/build/spec` 稍微转开一点，收口的是 `joint prepare` 路径里仍然重复的 ready-channel 包装。

新增：

1. `jointAxisVectorFromPreparedConstraint(...)`
2. `readyJointPreparedAngularChannel(...)`
3. `readyJointPreparedLinearChannel(...)`

当前效果：

1. `prepareJointLimitChannel(...)` 不再自己手工拼：
   - hinge angular ready channel
   - slider linear ready channel
2. `prepareJointDriveChannel(...)` 也改为复用同一组 angular/linear ready helper。
3. slider limit 的 `prepared.dir_* -> axis vec3` 投影统一改为 `jointAxisVectorFromPreparedConstraint(...)`。

这一步的意义：

1. joint prepare 层的 ready-channel 包装开始有固定出口，不再在 limit/drive 各写一份近似相同的 union 字面量。
2. 这为后续如果要继续收 joint prepare / descriptor / postprocess 三层之间的协议边界，提供了更稳定的底层构造点。
3. 这一步仍然没有改变 inactive/stalled policy，也没有改变任何 joint 求解数值。

### anchor prepare ready 包装也开始收口

这一轮继续沿 `joint prepare` 做小步统一，把 `anchor` 路径里仍然重复的 ready 包装也收成了单点 helper。

新增：

1. `readyJointPreparedAnchorChannel(...)`

当前效果：

1. `prepareJointAnchorChannel(...)` 的 `spring` 分支不再手工拼：
   - `dir_* -> axis`
   - `constraint = prepared`
   - `damping_scale = ...`
2. `prepareJointAnchorChannel(...)` 的 `fixed/ball_socket/hinge/slider` 分支也改为复用同一个 helper。
3. `jointAxisVectorFromPreparedLinearConstraint(...)` 现在同时服务：
   - limit slider
   - anchor spring
   - anchor fixed/ball_socket/hinge/slider

这一步的意义：

1. `joint prepare` 三条主线里，`anchor` 也开始拥有和 `limit/drive` 一样的底层 ready 构造出口。
2. 这让 `prepare -> descriptor -> postprocess` 之间的边界更清晰：prepare 层负责得到标准 channel，descriptor 层再做 postprocess 注入。
3. 这一步仍然没有改变 damping 数值、inactive/stalled 语义或求解行为。

### drive postprocess 注入也开始对齐 anchor 的更新模式

这一轮继续沿 `joint descriptor / postprocess` 做一小步收口，没有新加抽象层，只是把 `drive` 的 postprocess 注入方式改成和 `anchor` 同类的“复制后局部更新”。

当前效果：

1. `withPreparedJointDrivePostprocess(...)` 不再按 `angular/linear` 两个分支手工重建整个 union。
2. 现在改为：
   - `var result = prepared`
   - 针对活动分支只更新 `.postprocess`
   - 返回 `result`
3. `withPreparedJointAnchorPostprocess(...)` 与 `withPreparedJointDrivePostprocess(...)` 现在都采用“复制后注入 metadata”这一类模式。

这一步的意义：

1. joint descriptor postprocess 注入层开始更一致，不再出现一边是局部更新、一边是完整重建的风格分裂。
2. 后续如果要继续给 prepared channel 注入更多 metadata，这种写法更稳，也更不容易漏字段。
3. 这一步没有改变任何 postprocess policy、solver 数值或 channel 语义。

### runtime/batch context prepare 入口继续共享

这一轮没有再碰 solver 数值，而是把 `contact/environment` 两条链里仍然重复的一层
`prepared execution outcome -> runtime/batch solve context`
收成了共享 helper。

新增：

1. `preparePreparedRuntimeSolveContext(...)`
2. `preparePreparedBatchSolveContext(...)`

当前效果：

1. `prepareContactRuntimeSolveContext(...)` 不再自己重复写：
   - `base_residual <= 0 -> inactive`
   - `mapPreparedExecutionOutcome(...)`
2. `prepareEnvironmentRuntimeSolveContext(...)` 也改为走同一个 shared runtime prepare helper。
3. `prepareContactBatchSolveContext(...)` 与 `prepareEnvironmentBatchSolveContext(...)` 现在都统一走 shared batch prepare helper。
4. `mapPreparedExecutionOutcome(...)` 继续保留为更底层的三态映射原语，而 runtime/batch 是否要额外处理 `base_residual` 的差异被提升到单独 helper 层。

这一步的意义：

1. `prepared execution outcome` 的 prepare 协议又前进了一层，不再只有“映射 ready 载荷”是共享的，连 runtime/batch 外层入口也开始共享。
2. `contact/environment` 在 `prepare execution -> prepare runtime/batch context -> execute prepared outcome` 这条链上的外形更一致，后续如果继续收 `executePrepared...Outcome(...)` 外壳，切口会更窄。
3. 这一步仍然没有改变 inactive/stalled/ready 语义、row residual gating 或任何求解行为。

### batch outcome 的纯转发层也进一步缩掉

这一轮继续只收最薄的一层，把 `contact/environment` batch 路径里只为适配
`void` context 的纯转发 wrapper 去掉了。

新增：

1. `executePreparedBatchOutcomeDirect(...)`

当前效果：

1. `executePreparedContactBatchOutcome(...)` 不再需要：
   - `runPreparedContactBatch(...)`
   - 伪造 `void` context
2. `executePreparedEnvironmentBatchOutcome(...)` 也不再需要：
   - `runPreparedEnvironmentBatch(...)`
   - 伪造 `void` context
3. `executePreparedBatchOutcome(...)` 继续保留给需要显式 context 的 batch 路径；而 contact/environment 这种“ready -> bool”直达型 batch，开始走更薄的 direct helper。

这一步的意义：

1. batch outcome dispatch 现在开始区分两种真实形态：
   - 需要外部上下文的通用 batch dispatch
   - 不需要上下文的 direct batch dispatch
2. `contact/environment` 的 batch 主链又少了一层纯转发噪音，代码更接近真正的执行协议，而不是 adapter 套 adapter。
3. 这一步仍然没有改变 batch inactive/stalled fallback、settle/wake 策略或求解结果。

### contact finish outcome 入口也开始共享

这一轮继续沿 `contact` 的 row/batch finish 外壳做一小步，把两条 finish 入口共同依赖的
`mapPreparedContactPairOutcome(...)`
再上提一层，变成明确的 `contact finish outcome` helper。

新增：

1. `finishPreparedContactOutcome(...)`

当前效果：

1. `finishContactSolveResultOutcome(...)` 不再直接调用底层 `mapPreparedContactPairOutcome(...)`。
2. `finishContactBatchSolveOutcome(...)` 也改为走同一个 `finishPreparedContactOutcome(...)`。
3. row/batch 仍然各自保留自己的 ready 处理函数：
   - `finishPreparedContactSolveResultReady(...)`
   - `finishPreparedContactBatchSolveReady(...)`
   但两条路径的 outcome 映射入口现在已经统一。

这一步的意义：

1. `contact` 的 finish 链开始有了更明确的协议分层：
   - prepared pair outcome 映射
   - row/batch ready finish
   - settle/wake / finalize
2. 后续如果 `contact` finish 还要增加 trace / telemetry / policy 字段，改动点会先收敛到这层 helper，而不是 row/batch 两边各自再碰底层 map。
3. 这一步仍然没有改变 tangent 需求、stalled fallback、row finalize 或 batch changed 判定。

### predictive metadata 开始进入 row build spec / row 壳

这一轮不再继续做表层 wrapper 收口，而是补了一个更实质的底层缺口：
`DirectionalRowPlan` 里的 predictive metadata 以前在进入 `ConstraintRowBuildSpec` 时会丢失；
现在它开始成为 row build 层的显式字段。

新增/调整：

1. `ConstraintRowBuildSpec.predictive_residual_hint`
2. `ConstraintRow.predictive_residual_hint`
3. `buildConstraintRow(...)`
4. `makeConstraintRowBuildSpec(...)`
5. `makeConstraintRowBuildSpecFromPlan(...)`
6. `buildConstraintRowFromSpec(...)`

当前效果：

1. `DirectionalRowPlan.predictive_residual_hint` 不再只停留在 plan 层。
2. 通过 `makeConstraintRowBuildSpecFromPlan(...)` 生成的 row spec 会显式保留 predictive hint。
3. build 成 `ConstraintRow` 后，这个 metadata 也仍然存在。
4. 当前这份 metadata 还没有接入 priority 计算、solver numerics 或 iteration 停止条件，属于“先打通承载链路、后续再消费”的状态。

新增测试：

1. `makeConstraintRowBuildSpecFromPlan preserves predictive residual hint`

这一步的意义：

1. row plan 层的预测信息第一次开始拥有稳定的下游承载点，不再需要后续系统反查 directional payload 才能知道“这行是否带 predictive hint”。
2. 这为下一阶段的 trace / telemetry / 预测调度策略升级打了基础，因为 build spec / row 已经能直接携带这份 metadata。
3. 这一步刻意没有改变调度优先级和求解行为，避免把“metadata 挂载”与“行为升级”混成一次提交。

### row metadata 进一步从“单字段”升级成独立壳

在上一轮把 predictive hint 接到 `ConstraintRowBuildSpec/ConstraintRow` 之后，这一轮继续把它
从散落字段提升成统一 metadata 结构，避免后续 trace / telemetry / priority shaping 再次把字段分散到多处。

新增/调整：

1. `ConstraintRowMetadata`
2. `ConstraintRow.metadata`
3. `DirectionalRowPlan.metadata`
4. `ConstraintRowBuildSpec.metadata`
5. `makeConstraintRowMetadata(...)`

同步更新：

1. `makeDirectionalRowPlan(...)`
2. `buildConstraintRow(...)`
3. `makeConstraintRowBuildSpec(...)`
4. `makeConstraintRowBuildSpecFromPlan(...)`
5. `buildConstraintRowFromSpec(...)`
6. contact prepared normal 路径对 predictive hint 的读取方式
7. 对应单测改为校验 `spec.metadata.predictive_residual_hint`

当前效果：

1. predictive metadata 现在不再作为 `ConstraintRow` / `ConstraintRowBuildSpec` / `DirectionalRowPlan` 上的散字段存在。
2. 三层都开始共享同一个 `ConstraintRowMetadata` 承载壳。
3. 这使后续如果还要补：
   - trace id
   - source/provenance
   - predictive policy tags
   - debug counters
   不需要再次改一轮大面积字段签名。

这一步的意义：

1. row 承载层开始从“先塞一个字段进去”升级为“明确预留 metadata 扩展位”。
2. 这让后续行为升级可以更聚焦在 metadata 的消费逻辑，而不是继续反复重构载体。
3. 这一步依然没有改变 row priority、solver numerics、iteration 逻辑或 runtime dispatch。

### row metadata 现在已经有最小 debug 导出出口

在 metadata 承载层已经稳定之后，这一轮补了一个非常保守的观察出口，让外部不需要读取内部 `ConstraintRow`
结构，也能把 row 的关键调度信息和 predictive metadata 拿出来。

新增：

1. `ConstraintRowDebugSnapshot`
2. `writeConstraintRowDebugSnapshots(...)`

新增测试：

1. `writeConstraintRowDebugSnapshots exports row metadata`

当前效果：

1. `ConstraintRow` 现在可以被安全复制成只读 debug snapshot。
2. snapshot 当前包含：
   - `kind`
   - `index`
   - `priority`
   - `base_residual`
   - `metadata`
   - `equation`
3. 这意味着 `predictive_residual_hint` 已经不只是“内核里能传递”，而是“外部也能稳定观察”。

这一步的意义：

1. 之后无论要做约束阶段 trace、调试 dump、测试探针，还是更精细的迭代可视化，都有了现成的只读导出壳。
2. 这让后续“开始消费 metadata”之前，先具备了可观测性，不必把行为升级和观测接口绑在一起。
3. 这一步仍然没有改变求解、调度和停止条件；只是新增导出辅助层。

### 约束块级别的 row debug 采样也已经打通

在单条 `ConstraintRow -> ConstraintRowDebugSnapshot` 导出已经可用之后，这一轮继续把同样的观测能力提升到
“整块约束调度前的 row 集合”。

新增：

1. `collectConstraintRowDebugSnapshots(...)`

新增测试：

1. `collectConstraintRowDebugSnapshots includes environment row`

当前效果：

1. 现在可以直接输入：
   - `Scene1024`
   - `entities`
   - `joints`
   - `broadphase_pairs`
2. helper 会复用真实的：
   - island 划分
   - island stress 排序
   - row build
   - row priority 排序
3. 最终导出 `ConstraintRowDebugSnapshot` 列表，但不会执行 solve loop，也不会修改世界状态。

这一步的意义：

1. 约束阶段已经拥有“构建真实 row 集合并只读观察”的最小采样能力。
2. 后续如果要做：
   - 调试工具
   - 行优先级可视化
   - predictive metadata 观察
   - 回归测试里直接检查 row 组成
   都不需要再借助真正的 solve 过程侧面观察。
3. 这一步依然没有让 metadata 进入行为层，只是把观测从“单行”提升到了“stage/block 级 row 集合”。

### row debug 采样现在已经覆盖 environment / contact / joint 三类组成

在 `collectConstraintRowDebugSnapshots(...)` 打通之后，这一轮继续补了三类 row 组成的直接测试，
让后续如果要调整 priority shaping 或 row 构造策略时，有更明确的回归护栏。

新增测试：

1. `collectConstraintRowDebugSnapshots includes environment row`
2. `collectConstraintRowDebugSnapshots includes contact normal and friction rows`
3. `collectConstraintRowDebugSnapshots includes joint anchor limit and drive rows`

当前效果：

1. `environment` 场景现在能直接验证会不会生成 environment row。
2. `contact` 场景现在能直接验证会不会同时生成：
   - `contact_normal`
   - `contact_friction`
3. `joint` 场景现在能直接验证 slider + motor 配置下会不会生成：
   - `joint_anchor`
   - `joint_limit`
   - `joint_drive`

这一步的意义：

1. 观测层不再只是“能把 row 导出来”，而是已经开始覆盖真实 row family 组成。
2. 这为下一步把 `predictive_residual_hint` 逐步接入 row priority shaping 提供了重要护栏，因为一旦 row 构造条件变化，测试会立刻暴露。
3. 这一步仍然没有改变 solver、priority 算法或行为层逻辑；它只是在观测与测试层把边界补扎实。

### predictive metadata 第一次进入行为层：priority floor

在 row 观测与测试护栏补齐之后，这一轮终于让 `ConstraintRowMetadata` 开始进入行为层，但范围仍然刻意压得很小：
只接到 row priority 的 floor，不碰 solver 数值、warm start 或 iteration stop 逻辑。

新增/调整：

1. `measureConstraintRowMetadataPriorityFloor(...)`
2. `measureConstraintRowCachedPriority(...)` 现在先消费 `ConstraintRowMetadata`
3. `buildConstraintRow(...)` 继续沿同一路径把 metadata 传入 priority 计算

新增测试：

1. `measureConstraintRowCachedPriority respects predictive metadata floor`

当前效果：

1. 对没有 row cache 的新行来说，如果 `metadata.predictive_residual_hint` 高于 `base_residual`，priority 会被抬到这条 metadata floor。
2. 对已有 cache 的行来说，最终 priority 仍是：
   - metadata floor 处理后的 base priority
   - 与 cached residual/impulse priority
   两者取高。
3. 当前这一改动主要影响 contact/environment 这类已有 predictive hint 的 row；joint 仍然基本保持原样，因为 metadata 目前为 0。

这一步的意义：

1. 这标志着 `predictive_residual_hint` 不再只是“可传递、可导出、可测试”，而是第一次成为实际调度输入。
2. 因为接入点只是 priority floor，风险明显低于直接修改 equation/bias/impulse/solver 顺序。
3. 现有 contact/environment/joint/world 测试全部保持通过，说明这一步目前仍处于保守可控范围。

### finalize solve-step 展开也开始有共享 helper

这一轮转到 `finalize pair/single row result` 这条链，收口的是 `ConstraintSolveStep -> changed/applied_impulse` 的重复展开。

新增：

1. `finalizeRowResultFromSolveStep(...)`
2. `finalizePairRowResultWithContext(...)`
3. `finalizeSingleRowResultWithContext(...)`

当前效果：

1. `finalizePairSolveStepResult(...)` 不再自己手工展开：
   - `solve_step.changed`
   - `solve_step.applied_impulse`
2. `finalizeSingleSolveStepResult(...)` 也改为复用同一个 solve-step finalize helper。
3. pair/single 两条 finalize 路径现在共享同一层 `solve_step` 展开入口，而 wake/measure 语义仍保留在各自的 row-result helper 中。

这一步的意义：

1. `ConstraintSolveStep` 的结果展开不再在 pair/single 两处各写一份。
2. finalize 链条的分层更清晰：
   - `solve_step` 展开
   - pair/single wake 与 finalize
   - 统一 `finalizeConstraintRowResult(...)`
3. 这一步仍然没有改变 residual_after、impulse 或 wake 语义。

### solve-step metadata 覆盖也开始有共享 helper

这一轮继续沿 finalize 链做一小步，把 `ConstraintSolveStep` 的 `applied_impulse` 覆盖收成了一个单点 helper。

新增：

1. `withSolveStepAppliedImpulse(...)`

当前效果：

1. `finalizePreparedEnvironmentRowResult(...)` 不再手工重建匿名 `ConstraintSolveStep`。
2. 现在改为：
   - 保留 `solve_result.solve_step.changed`
   - 通过 `withSolveStepAppliedImpulse(...)` 覆盖最终 `applied_impulse`
3. environment finalize 路径重新回到统一的 solve-step 处理风格上。

这一步的意义：

1. `ConstraintSolveStep` 的 metadata 覆盖不再散落为匿名字面量。
2. 后续如果要继续补 step telemetry / trace 字段，这种 helper 能减少局部重建时漏字段的风险。
3. 这一步仍然没有改变 environment 的 impulse 聚合策略或最终 residual 行为。

### contact stress 排序现在也开始消费 predictive overlap

这一轮把 `contact` 子系统的 stress/order 入口接上了和 row plan 同方向的短时预测信息，
但仍然保持在很保守的范围内：只影响“当前已经形成接触的 pair”优先级，不扩展到非重叠预测接触。

新增/调整：

1. `computeContactSolvePriorityMagnitude(...)`
   - 在现有 `probeContactConstraint(...)` 成功后
   - 复用 `kernelPredictionDt()`
   - 复用 `predictKernelInstanceState(...)`
   - 复用 `computePredictedAABBOverlapDepth(...)`
   - 再通过 `computeDirectionalPredictiveSolvePriorityMagnitude(...)` 计算 contact stress
2. `buildContactPriorityOrder(...)`
   - 不改排序协议本身
   - 但它读取到的 `magnitude` 已开始包含 predictive overlap 影响

新增/调整测试：

1. `buildContactPriorityOrder prioritizes predictive overlap over weaker current overlap`
   - 现在明确构造成“两个 pair 当前都已重叠”
   - 其中一个 pair 未来 overlap 会明显加深
   - 验证 priority order 会先处理未来更糟的那一组

当前效果：

1. `contact` 的 subsystem stress 不再只看当前 penetration/normal/tangent stress。
2. 对“已经处于接触中的 pair”，如果预测步内 overlap 会继续恶化，priority 会更高。
3. `contact` 的 island/order 路径现在和此前已经接入 predictive 的 environment/row priority floor 更一致。

当前边界：

1. `probeContactConstraint(...)` 仍然要求当前 AABB 已重叠。
2. 这意味着“当前未接触、但预测步内会发生接触”的 broadphase pair 仍不会进入当前 contact priority order。
3. 如果后续要支持这类 pair，需要单独引入“非重叠预测接触 probe”或更前置的 predictive contact candidate 语义；这不是这一步的范围。

这一步的意义：

1. predictive metadata 不再只停留在 row build 和 row priority floor，已经开始进入 contact 子系统的顶层 stress/order 入口。
2. 接触求解调度现在更接近“先处理接下来会更糟的已接触约束”，而不是只看当前帧的静态穿透。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步仍然处于可控的保守接入范围。

### predictive gain 现在开始进入最小 `PredictiveRowPlan` 壳

这一轮开始正式朝 Phase 3 的“predictive 从增益升级为计划”迈出第一小步，
但仍然保持非常保守：先只把现有的 predictive gain 输入/输出统一收成显式 plan 载体，不改变 solver 数值与行为。

新增：

1. `PredictiveRowPlan`
   - `horizon_dt`
   - `current_depth`
   - `predicted_depth`
   - `urgency`
   - `allowed_correction_budget`
   - `gain`
2. `makePredictiveRowPlan(...)`
3. `makeDirectionalPredictiveRowPlan(...)`

调整：

1. `computeDirectionalPredictiveSolvePriorityMagnitude(...)`
   - 不再直接从 `direction + policy` 取 `gain.residual_hint`
   - 改为统一先构造 `PredictiveRowPlan`
   - 再读取 `plan.urgency`
2. `buildDirectionalRowPlan(...)`
   - 不再直接拼 `gain -> equation/residual`
   - 改为统一先构造 `PredictiveRowPlan`
   - 再消费 `plan.gain` 与 `plan.urgency`

新增测试：

1. `makeDirectionalPredictiveRowPlan preserves predictive gain semantics`
   - 验证 plan 壳没有改变既有 predictive gain 数值
   - 当前 `urgency == gain.residual_hint`
   - 当前 `allowed_correction_budget == gain.impulse_delta`
   - `horizon_dt` 统一来自 `kernelPredictionDt()`

当前效果：

1. contact/environment 现有的 predictive priority 与 predictive row-plan 不再只是共享 gain helper，也开始共享同一个最小 plan 载体。
2. predictive 路径现在第一次显式拥有：
   - horizon
   - 当前深度
   - 预测深度
   - urgency
   - correction budget
3. 当前行为保持不变，仍然只是把旧语义压进新壳中。

当前边界：

1. 这个 `PredictiveRowPlan` 还不是完整的求解计划协议。
2. 目前仍未显式携带：
   - predicted target pose / target velocity
   - row-level allowed axis
   - overshoot clamp policy
   - solver-consumed target residual
3. joint drive 还没有正式切到这层 plan 壳，当前只是 contact/environment 这条 directional predictive 路径先打通。

这一步的意义：

1. Phase 3 不再停留在文档规划，已经有了第一层真实代码载体。
2. 后续如果继续把 predictive 做成更明确的 solve target，已经不需要再从 `gain helper` 直接硬切，而可以沿 `PredictiveRowPlan` 扩字段与消费点。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步仍属于低风险协议收口。

### joint drive predictive 入口也开始对齐 `PredictiveRowPlan`

在 contact/environment 已经先接入最小 `PredictiveRowPlan` 之后，这一轮把 joint drive 的 predictive 误差入口也对齐到同一层 plan helper，
但仍然严格保持行为不变：不改 brake 判定、target clamp、desired velocity 或 step 限幅公式。

新增：

1. `jointDrivePredictiveConstraintPolicy()`
2. `makeJointDrivePredictiveRowPlan(...)`

调整：

1. `computeKernelJointDrivePlan(...)`
   - 预测误差不再只靠局部 `@abs(predicted_error)` / `@abs(position_error)` 散写
   - 改为统一先构造 `PredictiveRowPlan`
   - 再用：
     - `plan.current_depth`
     - `plan.predicted_depth`
     表达当前误差幅度与预测误差幅度
2. `use_predictive_brake` 仍保持原语义
   - 仍然要求跨过目标或预测误差比当前误差更小
3. `control_error` 的符号与数值仍保持原行为
   - 只是误差幅值改为从 plan 中读取

新增测试：

1. `makeJointDrivePredictiveRowPlan preserves error magnitudes for brake decisions`
   - 验证 joint drive 的 plan 壳会把带符号误差收成：
     - `current_depth = abs(position_error)`
     - `predicted_depth = abs(predicted_error)`
   - 当前 `urgency` 与 `allowed_correction_budget` 也只是最小保守映射，不直接改变 solver 行为

当前效果：

1. `joint drive` 不再是 predictive plan 体系之外的独立入口。
2. contact/environment/joint drive 三条 predictive 路径现在都已经至少共享同一类 plan 壳。
3. 当前 joint drive 仍然只是“用 plan 收口误差输入”，还没有把 plan 直接升级成真正的 motor solve target。

当前边界：

1. `jointDrivePredictiveConstraintPolicy()` 目前只是最小保守 policy，用来承载误差幅值，不参与 joint motor solver numerics。
2. `PredictiveRowPlan.gain` 在 joint drive 这里当前主要是占位性承载，还没有像 contact/environment 那样直接进入 row equation shaping。
3. 这一步仍未引入：
   - predicted target velocity profile
   - braking budget policy
   - overshoot clamp policy
   - motor-specific predictive target row

这一步的意义：

1. Phase 3 的 predictive plan 不再只覆盖 directional contact/environment，也开始触碰 joint drive。
2. 后续如果要把 joint motor 从“预测辅助刹车”继续推进到“显式预测驱动目标”，现在已经有了统一 plan 承载点。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步仍然处于低风险协议统一范围。

### contact priority 现在也开始纳入非重叠 predictive candidate

这一轮继续沿 contact predictive 路径补之前明确留下的边界：
当前未重叠、但 broadphase 已命中且预测步内会发生接触的 pair，现在也能进入 contact priority order。

调整：

1. `computeContactSolvePriorityMagnitude(...)`
   - 不再一开始就完全依赖 `probeContactConstraint(...)`
   - 现在会先统一计算：
     - 当前 AABB
     - 预测位姿
     - `computePredictedAABBOverlapDepth(...)`
2. 如果当前已有真实接触：
   - 仍沿原路径
   - 使用 manifold + relative velocity + predictive overlap
3. 如果当前没有真实接触，但预测步内会形成 overlap：
   - 现在会走新的 predictive-only stress 分支
   - 当前只给 priority/stress 使用
   - 不会伪造 manifold，也不会直接进入 contact solve payload

新增测试：

1. `buildContactPriorityOrder includes non-overlapping predictive contact candidate`
   - 验证当前未重叠、但预测步内会发生接触的 broadphase pair
   - 已经可以进入 priority order

当前效果：

1. `contact` 的 predictive priority 不再只覆盖“已经接触的 pair”。
2. broadphase 命中的未来碰撞对现在已经可以被更早排到前面。
3. 这一步补上了上一轮文档里明确记录的边界缺口。

当前边界：

1. 这仍然只是 priority/stress 层面的接入。
2. `prepareContactConstraintPair(...)` 与真实 contact solve 仍要求当前已经形成接触 manifold。
3. 也就是说，当前系统已经能“更早关注”未来接触，但还不能“提前按 contact row 直接求解”未来接触。

这一步的意义：

1. contact predictive 已经从“已接触约束的优先级优化”推进到“未来接触候选的排序感知”。
2. 下一步如果要继续做真正的 predictive contact plan / speculative contact row，就不需要再先回头补 priority candidate 入口。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步仍然控制在调度层，不会冲击现有求解路径。

### contact prepared payload 现在开始支持最小 speculative normal-only pair

在 priority 层已经能识别“当前未重叠但预测步内会接触”的 candidate 之后，这一轮继续往前推进了一小步：
`prepareContactConstraintPair(...)` 现在已经可以为一部分 predictive candidate 真正产出 prepared payload，
但范围仍然严格受限，避免直接冲击现有 grounded / environment 语义。

调整：

1. `ContactPreparedPair`
   - 新增 `speculative` 标记
2. `prepareContactConstraintPair(...)`
   - 当前真实接触时，行为保持不变
   - 当前未接触但预测步内会重叠时：
     - 现在可返回最小 speculative prepared pair
     - 仅生成 normal channel
     - `restitution = 0`
     - `friction = 0`
     - `has_tangent = false`
     - `friction_equation = zeroConstraintEquation()`
3. 当前 speculative contact 只允许 `dynamic-dynamic`
   - 如果任一侧是 static，仍然返回 `null`
   - 保持 dynamic-static 的 grounded / wall / environment 语义不被 speculative contact 提前接管

新增测试：

1. `prepareContactConstraintPair returns speculative normal-only payload for predictive candidate`
   - 验证 predictive candidate 能形成 prepared payload
   - 验证当前 payload 是：
     - speculative
     - normal-only
     - 无 restitution / friction

当前效果：

1. contact predictive 已经不只是 priority candidate，也开始有最小 prepared payload 形态。
2. 对未来会相撞的 `dynamic-dynamic` pair，内核现在已经具备“提前准备 normal 分离约束”的入口。
3. 现有 grounded/resting 语义未被破坏，因为 static 侧仍不走 speculative prepared contact。

当前边界：

1. speculative prepared contact 目前只服务 `dynamic-dynamic`。
2. 仍然没有 speculative friction、speculative restitution、speculative manifold。
3. dynamic-static 的未来接触仍只停留在 priority/order 层，不会提前形成 prepared contact row。

这一步的意义：

1. contact predictive 已经从“可排序的候选”推进到“可准备的最小约束载荷”。
2. 后续如果要继续做真正的 speculative contact row，可以在这层继续补：
   - 更稳定的法线来源
   - bias/budget policy
   - dynamic-static 的安全接入边界
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步目前仍控制在可回归范围内。

### speculative normal 方向现在开始按最小预测重叠轴选择

在最小 speculative prepared pair 已经能落地之后，这一轮继续补了一个底层但关键的稳定性点：
predictive candidate 的 normal 方向不再只靠两个 body 的中心点 delta 猜主轴，
而是开始基于预测 AABB 的最小重叠轴来选分离方向。

调整：

1. 新增 `buildPredictedAABBSeparationDirection(...)`
   - 基于预测后的 AABB 区间直接计算 `x/y/z` 三轴重叠量
   - 选择最小预测重叠轴作为 speculative normal 方向
   - 法线正负仍结合预测中心差决定，保持最小可用分离朝向
2. `prepareContactConstraintPair(...)`
   - 在 predictive-only candidate 生成 speculative prepared pair 时
   - normal channel 改为统一走 `buildPredictedAABBSeparationDirection(...)`
   - 不再退回到只看中心差绝对值的粗糙主轴选择

新增测试：

1. `buildPredictedAABBSeparationDirection uses minimum predicted overlap axis`
   - 验证 speculative normal 会沿最小预测重叠轴生成
   - 当前用例明确覆盖 `y` 轴最小重叠场景

当前效果：

1. speculative prepared contact 的 normal 方向来源更接近真实 penetration axis，而不是中心点启发式。
2. 这让后续继续补 speculative bias / budget shaping 时，输入方向更稳定，不容易在边角接近时跳到错误主轴。
3. 现有 `dynamic-static` 限制保持不变，grounded / wall / resting 语义没有被额外放宽。

当前边界：

1. 这一步只改 speculative normal 的方向来源，没有引入真实 manifold。
2. tie-break 仍然是最小轴优先的简单规则，还没有更复杂的运动趋势/历史缓存稳定器。
3. speculative prepared contact 仍然只允许 `dynamic-dynamic`；`dynamic-static` 未来接触依旧停留在 priority/order 层。

这一步的意义：

1. speculative contact 不再只是“能准备出来”，而是开始具备更可信的方向基元。
2. 这为 Phase 3/4 后续继续补：
   - speculative normal bias policy
   - speculative iteration budget shaping
   - debug/export 中的 speculative metadata 观察
   提供了更稳的底层输入。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过，说明这一步仍然控制在窄范围内，没有破坏当前世界求解语义。

### speculative contact metadata 现在开始进入 row/debug 观察面

在 speculative prepared pair 与最小预测分离方向都已经落地之后，这一轮没有继续扩大 solver 行为，
而是先把 speculative 状态补进 row metadata 与 debug snapshot，解决“能跑但不可观察”的问题。

调整：

1. `ConstraintRowMetadata`
   - 新增 `speculative_contact: bool`
2. contact row build
   - `contact_normal` row 在由 `ContactPreparedPair` 生成 row spec 时
   - 会把 `prepared.speculative` 显式抬到 `ConstraintRowMetadata.speculative_contact`
3. friction row
   - 仍然保持原样
   - 不会因为 speculative normal-only pair 被错误打上 speculative friction metadata
4. debug/export
   - `writeConstraintRowDebugSnapshots(...)` 与 `collectConstraintRowDebugSnapshots(...)`
   - 现在会把这层 metadata 一并稳定导出

新增测试：

1. `collectConstraintRowDebugSnapshots marks speculative contact normal metadata`
   - 验证 speculative prepared pair 生成的 `contact_normal` row 会携带 `speculative_contact = true`
   - 同时验证 friction 路径不会被错误污染
2. `writeConstraintRowDebugSnapshots exports row metadata`
   - 现在也覆盖 metadata 中的 speculative 标记导出

当前效果：

1. speculative/predictive contact 不再只能从 prepared pair 层观察，row/debug 视角也能直接识别。
2. 这让后续继续调：
   - speculative priority floor
   - speculative iteration budget
   - row export / telemetry
   时不再需要反查上游 prepared payload。
3. 由于这一步只扩 metadata 与 build/debug 路径，没有改 solver 数值，所以 grounded / resting / environment 语义保持不变。

当前边界：

1. `speculative_contact` 目前只作为 metadata/debug 观察字段使用，还没有单独参与 priority/budget/shaping。
2. 仍然只有 `contact_normal` row 会携带 speculative 标记；没有 speculative friction row。
3. `dynamic-static` speculative prepared contact 仍未放开。

这一步的意义：

1. Phase 3/4 现在不只是“开始有 speculative prepared row 的前置形态”，而是连可观测性都补上了。
2. 后续如果继续做 speculative bias/budget policy，可以直接基于 row metadata 落地，而不是再回头重构一遍 debug/build 通路。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过。

### speculative metadata 现在开始进入 row priority floor

在 speculative contact 的 row metadata/debug 观察面已经补齐之后，这一轮开始让它第一次进入实际调度行为，
但范围仍然非常保守：先只做 metadata priority floor 的轻微抬升，不改 solver 公式，也不放大到 friction 或 dynamic-static。

调整：

1. 新增 `measureSpeculativeContactPriorityFloor(...)`
   - 仅当 `ConstraintRowMetadata.speculative_contact = true` 时生效
   - 当前只给 base priority 一个非常轻的 floor 抬升（`+5%`）
2. `measureConstraintRowMetadataPriorityFloor(...)`
   - 在原有 `predictive_residual_hint` floor 之上
   - 继续叠加 speculative metadata floor

新增测试：

1. `measureConstraintRowCachedPriority lifts speculative contact priority floor`
   - 验证 speculative metadata 会把 `base_priority = 2.0` 抬到 `2.1`

当前效果：

1. speculative `contact_normal` row 不再只是“可观察”，而是开始获得最小调度倾斜。
2. 这个倾斜仍然比 predictive residual floor 弱得多，避免 speculative metadata 直接压过真实 penetration/velocity stress。
3. 现有世界行为保持稳定，说明这一改动目前仍然停留在低风险调度层。

当前边界：

1. 这一步没有把 speculative metadata 接到 iteration budget、equation bias、max impulse 或 solver apply。
2. 仍然只对 `contact_normal` row 生效。
3. `dynamic-static` speculative prepared contact 仍未放开。

这一步的意义：

1. Phase 3/4 现在已经从“speculative row 可准备、可导出”推进到“speculative row 开始影响调度”。
2. 后续如果继续补：
   - speculative iteration budget shaping
   - speculative bias policy
   - speculative row export/telemetry
   会更自然，因为 metadata 已经进入统一 priority 入口。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过。

### speculative pair 现在开始进入 iteration budget shaping

在 speculative metadata 已经进入 row priority floor 之后，这一轮继续沿同一思路往前推进了一小步：
不是直接改 equation/bias，而是先让“岛内是否存在 speculative pair”影响迭代预算。

调整：

1. 新增 `countSpeculativeContactPairs(...)`
   - 扫描 broadphase pair 子集
   - 统计当前能形成 speculative prepared pair 的 contact pair 数量
2. 新增 `boostIterationBudgetForSpeculativeContacts(...)`
   - 若 speculative pair 数量大于 0，则额外增加 `1` 次迭代 pass
   - 仍保持全局上限 `6`
3. `solveContactConstraintsForPairs(...)`
   - contact-only 路径现在会先统计 speculative pair 数量
   - 再在原有 `computeContactIterationBudget(...)` 基础上做 budget boost
4. `solveConstraintBlock(...)`
   - unified island 路径也会对当前 island 的 `pair_subset` 做同样的 speculative pair 统计
   - 再在 `computeConstraintBlockIterationBudget(...)` 结果上应用同样的 budget boost

新增测试：

1. `countSpeculativeContactPairs detects predictive-only contact pair`
   - 验证 predictive-only `dynamic-dynamic` pair 会被正确计入 speculative count
2. `boostIterationBudgetForSpeculativeContacts adds one extra pass and clamps`
   - 验证有 speculative pair 时预算 `+1`
   - 验证仍然封顶在 `6`

当前效果：

1. speculative contact 现在不只影响 priority floor，也开始影响“愿意多解几轮”的预算。
2. 这个预算提升仍然非常保守：
   - 只多 `1` 次 pass
   - 只有存在 speculative pair 时才生效
   - 仍然不改变 row equation / bias / impulse 上限
3. contact-only 路径与 unified constraint block 路径的预算语义现在保持一致，不会出现一边识别 speculative、另一边完全忽略的情况。

当前边界：

1. 这一步还是没有把 speculative metadata 接到 solver apply、warm start 或 bias 公式。
2. budget boost 当前只看“是否存在 speculative pair”，还没有按数量、stress 或 urgency 做更细分层。
3. `dynamic-static` speculative prepared contact 仍未放开，因此这层 boost 目前本质上还是服务 `dynamic-dynamic` predictive contact。

这一步的意义：

1. speculative contact 已经从“可准备 -> 可观察 -> 可排序”推进到“可多给一轮求解预算”。
2. 后续如果继续补 speculative bias policy，会建立在更合理的调度前提上，而不是在预算完全不变的情况下硬加 bias。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过。

### speculative normal 现在开始进入最小 bias shaping

在 speculative pair 已经能影响 priority 与 iteration budget 之后，这一轮终于开始让它轻微触碰 equation，
但仍然坚持最保守边界：只改 speculative `contact_normal` 的 `bias`，不改 friction、不改 `max_impulse`、不放开 dynamic-static。

调整：

1. 新增 `applySpeculativeContactBiasBoost(...)`
   - 当前只对 `ConstraintRowEquation.bias` 做一个很小的提升（`x1.1`）
   - `effective_mass` 与 `max_impulse` 保持不变
2. `prepareContactConstraintPair(...)`
   - 在 predictive-only `dynamic-dynamic` speculative branch 中
   - `normal_row_plan.equation` 不再原样回填
   - 而是先经过 `applySpeculativeContactBiasBoost(...)`
3. 普通真实接触路径不变
   - 已重叠接触的 normal/friction equation 没有改
   - `dynamic-static` speculative 仍未放开，因此 environment/grounded 语义没有被这一步触碰

测试：

1. `prepareContactConstraintPair returns speculative normal-only payload for predictive candidate`
   - 现在额外验证 speculative prepared pair 的 `normal_equation.bias`
   - 会高于同一方向/深度下的 baseline `buildContactNormalRowPlan(...)`

当前效果：

1. speculative contact 现在第一次开始真正影响 normal equation，而不只是 priority/budget。
2. 这个影响仍然非常轻：
   - 只提升 `bias`
   - 当前按 `predicted_depth - current_depth` 做三档提升：
     - `< 1.0` -> `x1.1`
     - `>= 1.0` -> `x1.15`
     - `>= 2.0` -> `x1.2`
   - 不改 `max_impulse`
   - 不影响 friction
3. 因为它只存在于 predictive-only speculative branch，所以现有 grounded/resting/static environment 行为保持稳定。

当前边界：

1. 这一步没有把 speculative shaping 扩到 friction、restitution 或 tangent row。
2. 当前还是最小分层策略，阈值与倍率仍是硬编码常量，尚未抽成可配置 policy 入口。
3. `dynamic-static` speculative prepared contact 仍未放开。

这一步的意义：

1. speculative contact 已经从“调度层影响”推进到“最小 equation shaping”。
2. 后续如果继续补更精细的 speculative normal policy，可以在这个 helper 上做参数化，而不需要再去改 prepared branch 主体。
3. 现有 `zig test src/physics_kernel.zig`、`zig test src/tick_engine.zig`、`zig test src/physics_world.zig` 全部保持通过。

### Query Layer 兼容修复：恢复 AABB 与旧字段接口

在继续 speculative 内核迭代时，仓库里暴露出一批 Query Layer 接口断层（类型和字段改名后，
`physics/kcc/query_world/query_penetration/collision` 仍在消费旧接口）。这轮先补兼容层，避免主线开发反复被编译错误打断。

调整：

1. `query_types` 兼容恢复：
   - `QueryFilter.group_mask`
   - `QueryHit` 的 legacy 平铺字段（`body_type/material_type/medium_type/surface_condition`）
   - `PenetrationResult` 的 legacy 字段（`entity_id/hit_environment/classification/telemetry`）
   - `AABB/VoxelBox` 兼容类型（整型坐标语义）
2. `physics` 兼容恢复：
   - `aabbHit(...)`
   - `makeAABB(...)`
3. `query_penetration` 兼容恢复：
   - `computePenetrationAABB(...)`（转发到 box 实现）
4. `collision` 调用修复：
   - `voxelBoxHit` 显式走 `physics.voxelBoxHit(...)`

当前效果：

1. Query Layer 新旧接口在当前仓库里重新可并存，避免一次性重构所有调用点。
2. `AABB` 旧接口语义保持可用（包括 KCC 路径），满足主干模块回归需求。
3. 为继续推进 speculative kernel 提供了稳定编译基线。

验证：

1. `zig test src/physics_kernel.zig`
2. `zig test src/tick_engine.zig`
3. `zig test src/physics_world.zig`
4. 三组测试当前已恢复并全部通过（`221/221`）。

### speculative bias scale 进入 row metadata/debug 导出

在 speculative bias 已经接入 normal equation 后，这一轮补了行级可观测性：
把每个 row 实际使用的 speculative bias scale 挂到 metadata。

调整：

1. `ConstraintRowMetadata` 新增字段：
   - `speculative_bias_scale: f32 = 1.0`
2. `buildContactRowSpecForPair(...)` 在 `contact_normal` 且 `prepared.speculative = true` 时：
   - 复用 `measureSpeculativeContactBiasScale(...)`
   - 将结果写入 `metadata.speculative_bias_scale`
3. 非 speculative 或非 normal 行保持默认值 `1.0`。

测试：

1. `writeConstraintRowDebugSnapshots exports row metadata`
   - 新增 `speculative_bias_scale` 导出断言
2. `collectConstraintRowDebugSnapshots marks speculative contact normal metadata`
   - `contact_normal` 断言 `speculative_bias_scale > 1.0`
   - `contact_friction` 断言 `speculative_bias_scale == 1.0`

当前效果：

1. row debug snapshot 现在可以直接观察 speculative bias 档位，不需要反推 equation。
2. 为后续 speculative policy 调参、回归比对和可视化提供稳定的行级观测点。

边界：

1. 这一步只扩展 metadata/debug，不改 solver 数值路径本身。
2. 没有调整 priority、iteration budget、warm-start 或 apply 顺序。

验证：

1. `zig test src/physics_kernel.zig`（`221/221`）
2. `zig test src/tick_engine.zig`（`221/221`）
3. `zig test src/physics_world.zig`（`221/221`）

### speculative bias scale 计算入口进一步收口

在上一轮把 `speculative_bias_scale` 挂入 row metadata/debug 后，这一轮继续做了一个小型协议收口，
目标是让“equation 使用的 bias scale”和“metadata 导出的 bias scale”严格共享同一套计算入口。

调整：

1. 新增 `measureSpeculativeContactBiasScaleWithPolicy(...)`
   - 显式接收 `SpeculativeContactBiasPolicy`
   - `measureSpeculativeContactBiasScale(...)` 变成薄封装（默认 policy）
2. `applySpeculativeContactBiasBoost(...)`
   - 先取一次 `scale`
   - 再用于 `equation.bias` 乘法（避免隐式重复求值）
3. `buildContactRowSpecForPair(...)`
   - speculative `contact_normal` metadata 写入时
   - 改为使用当前 row 的 `payload.direction`
   - 与 row equation 路径保持同源输入

测试：

1. 新增 `measureSpeculativeContactBiasScale matches policy helper`
   - 验证默认入口与显式 policy 入口数值一致
2. 原有 speculative metadata/debug 测试保持通过

当前效果：

1. speculative bias 的“执行值”和“观测值”更加严格对齐。
2. 后续如果把 policy 参数改为可配置，不需要分别修改 equation 与 metadata 路径。

验证：

1. `zig test src/physics_kernel.zig`（`222/222`）
2. `zig test src/tick_engine.zig`（`222/222`）
3. `zig test src/physics_world.zig`（`222/222`）

### speculative metadata 组装收口为单一 helper

在 `speculative_contact` / `speculative_bias_scale` / `speculative_predictive_excess` 三个字段都已接入后，
这一轮把 contact-normal 的 metadata 组装逻辑进一步收口，避免 row 构建阶段分散写字段。

调整：

1. 新增 `withSpeculativeContactDirectionalMetadata(...)`
   - 输入：`metadata + speculative flag + PreparedDirectionalConstraint`
   - 输出：一次性完成
     - `speculative_contact`
     - `speculative_bias_scale`
     - `speculative_predictive_excess`
2. `buildContactRowSpecForPair(...)`
   - `contact_normal` 路径改为直接调用该 helper
   - 移除本地分散的三段 metadata 注入代码

新增测试：

1. `withSpeculativeContactDirectionalMetadata applies speculative fields from direction`
   - 覆盖 speculative / non-speculative 两条路径
   - 验证输入 `direction` 与 metadata 输出的 bias/excess 一致
   - 验证非 speculative 保持默认值

当前效果：

1. speculative metadata 写入入口变成单点，后续扩字段不再需要回头改 row builder 多处代码。
2. 行为无变化，属于纯结构收口。

验证：

1. `zig test src/physics_kernel.zig`（`223/223`）
2. `zig test src/tick_engine.zig`（`223/223`）
3. `zig test src/physics_world.zig`（`223/223`）

### speculative bias metadata 增加 tier 档位

在已有 `speculative_bias_scale` 和 `speculative_predictive_excess` 的基础上，这一轮补了策略档位可观测：
让调试侧可以直接看到当前行命中的是 `base/mid/high` 哪一档，而不必反推。

调整：

1. 新增 `SpeculativeContactBiasTier`：
   - `none/base/mid/high`
2. `ConstraintRowMetadata` 新增：
   - `speculative_bias_tier: SpeculativeContactBiasTier = .none`
3. 新增 `SpeculativeContactBiasProfile` 与 `measureSpeculativeContactBiasProfile(...)`
   - 统一产出：
     - `scale`
     - `tier`
     - `predictive_excess`
4. `measureSpeculativeContactBiasScaleWithPolicy(...)` 改为复用 profile 输出（不再单独分支判定）
5. `withSpeculativeContactDirectionalMetadata(...)` 也改为复用同一 profile：
   - 保证 metadata 的 `tier/scale/excess` 与 equation 路径同源

测试：

1. 新增 `measureSpeculativeContactBiasProfile reports tier and scale consistently`
2. 扩展 `withSpeculativeContactDirectionalMetadata applies speculative fields from direction`
   - 增加 tier 断言（speculative 为 `high`，non-speculative 为 `none`）
3. 扩展 row debug 相关测试：
   - `writeConstraintRowDebugSnapshots exports row metadata`
   - `collectConstraintRowDebugSnapshots marks speculative contact normal metadata`
   增加 tier 导出/一致性断言

当前效果：

1. speculative bias 不再只有“数值”，还带“策略档位”可观测信息。
2. solver 与 metadata 共用一个 policy 判定入口，降低后续调参漂移风险。

验证：

1. `zig test src/physics_kernel.zig`（`224/224`）
2. `zig test src/tick_engine.zig`（`224/224`）
3. `zig test src/physics_world.zig`（`224/224`）

### speculative priority floor 开始消费 tier（从观测进入行为）

在 speculative `tier` 已经进入 metadata/debug 后，这一轮把它第一次接入调度行为：
`measureSpeculativeContactPriorityFloor(...)` 不再固定 `+5%`，而是按 tier 分层抬升。

调整：

1. 新增 `SpeculativeContactPriorityFloorPolicy`：
   - `legacy_fallback_scale = 1.05`
   - `base_scale = 1.05`
   - `mid_scale = 1.075`
   - `high_scale = 1.1`
2. `measureSpeculativeContactPriorityFloor(...)`：
   - `speculative_contact = false`：不变
   - `speculative_contact = true`：按 `speculative_bias_tier` 选择倍率
3. `none` tier 仍保留 fallback（`1.05`）：
   - 保持对潜在旧路径/边界数据的兼容，不引入突变

测试：

1. 现有 `measureConstraintRowCachedPriority lifts speculative contact priority floor` 改为显式断言 `base_scale`
2. 新增 `measureConstraintRowCachedPriority applies stronger speculative floor for higher tier`
   - 覆盖 `base/mid/high` 三档
   - 断言优先级单调递增

当前效果：

1. speculative tier 不再只是观察字段，开始进入 priority 调度行为。
2. 调整仍然保守，最高只到 `+10%`，不触碰 solver equation/warm-start/apply。

验证：

1. `zig test src/physics_kernel.zig`（`225/225`）
2. `zig test src/tick_engine.zig`（`225/225`）
3. `zig test src/physics_world.zig`（`225/225`）

### speculative profile 默认入口进一步收口

在 `SpeculativeContactBiasProfile` 已落地后，这一轮继续把默认策略入口收口，
目标是让所有“默认 policy 下的 profile 消费点”走同一 helper，减少未来策略漂移风险。

调整：

1. 新增 `measureSpeculativeContactBiasProfileDefault(...)`
   - 统一使用 `speculativeContactBiasPolicy()`
2. `withSpeculativeContactDirectionalMetadata(...)`
   - 改为直接使用 `measureSpeculativeContactBiasProfileDefault(...)`
3. `applySpeculativeContactBiasBoost(...)`
   - 改为从默认 profile 读取 `scale`
4. `collectConstraintRowDebugSnapshots marks speculative contact normal metadata` 测试增强：
   - 不再只做范围断言
   - 直接与 `measureSpeculativeContactBiasProfileDefault(prepared.normal)` 做一致性校验（`tier/scale`）

当前效果：

1. 默认 speculative policy 的执行路径与 metadata 导出路径再次收紧到单一入口。
2. 后续若改 policy 阈值或倍率，profile/metadata/equation 会一起变化，回归护栏更强。

验证：

1. `zig test src/physics_kernel.zig`（`224/224`）
2. `zig test src/tick_engine.zig`（`224/224`）
3. `zig test src/physics_world.zig`（`224/224`）

### speculative predictive excess 进入 row metadata

在已有 `speculative_bias_scale` 行级可观测的基础上，这一轮补齐了它的直接来源量：
`predictive_excess = max(0, predicted_depth - depth)`。

调整：

1. `ConstraintRowMetadata` 新增：
   - `speculative_predictive_excess: f32 = 0.0`
2. `buildContactRowSpecForPair(...)` 在 speculative `contact_normal` 上：
   - 计算并写入 `speculative_predictive_excess`
   - 与 `speculative_bias_scale` 同步导出，形成“输入量 + 策略输出量”成对观测
3. 非 speculative / friction 行保持 `0.0`

测试：

1. `writeConstraintRowDebugSnapshots exports row metadata`
   - 增加 `speculative_predictive_excess` 导出断言
2. `collectConstraintRowDebugSnapshots marks speculative contact normal metadata`
   - 改为校验定义一致性：
     - `>= 0`
     - 并与 `max(0, predicted_depth - depth)` 近似相等
   - friction 行保持 `0.0`

说明：

1. 该场景下 speculative 并不保证 `predictive_excess > 0`，因此测试断言按定义值校验，而不是强行要求正值。
2. 这一步依旧只扩充 metadata/debug，不改变 solver 数值行为。

验证：

1. `zig test src/physics_kernel.zig`（`222/222`）
2. `zig test src/tick_engine.zig`（`222/222`）
3. `zig test src/physics_world.zig`（`222/222`）

### speculative iteration budget 按 tier 分档并修复脆弱断言

这一轮把 contact 迭代预算从“只要有 speculative pair 就 +1”升级为“按 speculative tier 分档”，同时修复了对应测试对场景参数过度耦合的问题。

调整：

1. 新增 `measureMaxSpeculativeContactTier(...)`
   - 在 broadphase pair 集合内扫描 speculative contact
   - 复用 `measureSpeculativeContactBiasProfileDefault(prepared.normal).tier`
   - 返回该批次最高 tier（`none/base/mid/high`）
2. 新增 `SpeculativeContactIterationBudgetPolicy` 与 `speculativeContactIterationBudgetPolicy()`
   - `low_tier_extra_passes = 1`
   - `high_tier_extra_passes = 2`
3. `boostIterationBudgetForSpeculativeContacts(...)` 签名升级
   - 从 `(base_budget, speculative_pair_count)`
   - 升级为 `(base_budget, speculative_pair_count, max_speculative_tier)`
   - 规则：`none/base/mid => +1`，`high => +2`，并保持 `<= 6` 上限
4. 两个主调用点完成接线：
   - `solveContactConstraintsForPairs(...)`
   - `solveConstraintBlock(...)`

测试：

1. 新增 `measureMaxSpeculativeContactTier reports highest speculative profile tier`
2. `boostIterationBudgetForSpeculativeContacts` 测试更新为 tier 分档与 clamp 覆盖
3. 修复 tier 测试断言策略：
   - 不再硬编码期望 `.high`
   - 改为与同一 `prepared/profile` 管线推导出的 `expected_tier` 对齐
   - 避免 fixture 数值波动导致“语义正确但断言脆弱”的失败

说明：

1. 本轮仅调整预算 shaping 与测试稳健性，不改 AABB 接触语义。
2. KCC/查询相关 AABB 路径保持原策略。

验证：

1. `zig test src/physics_kernel.zig`（`226/226`）
2. `zig test src/tick_engine.zig`（`226/226`）
3. `zig test src/physics_world.zig`（`226/226`）

### speculative iteration budget 增加 pair 密度微调（保守）

在 tier 分档预算已经接入后，这一轮补了一个非常小的密度维度：
当 island/contact 集中出现较多 speculative pair 时，再额外给 1 次迭代 pass。

调整：

1. `SpeculativeContactIterationBudgetPolicy` 扩展：
   - `dense_pair_threshold: usize = 4`
   - `dense_pair_extra_passes: u8 = 1`
2. `boostIterationBudgetForSpeculativeContacts(...)`
   - 基础仍按 `tier`：`none/base/mid => +1`，`high => +2`
   - 当 `speculative_pair_count >= 4` 时再 `+1`
   - 总预算仍保持 `<= 6` clamp
3. 调整范围仍限制在预算 shaping，不改 AABB/query/contact equation。

测试：

1. `boostIterationBudgetForSpeculativeContacts scales by speculative tier and clamps`
   - 新增 dense base 档断言：`(2, 4, .base) => 4`
   - 新增 dense high 档断言：`(2, 4, .high) => 5`
   - 新增高密度 clamp 断言：`(5, 8, .high) => 6`

说明：

1. 这是保守强化：仅在 speculative pair 密集场景提升迭代预算，避免单一高 tier 之外的密集场景被低估。
2. 未引入“按 pair 数线性增长”，只保留一档阈值，控制求解开销和行为突变风险。
3. AABB 语义保持不变。

验证：

1. `zig test src/physics_kernel.zig`（`226/226`）
2. `zig test src/tick_engine.zig`（`226/226`）
3. `zig test src/physics_world.zig`（`226/226`）

### speculative 统计收口为单次遍历 summary（降重复 prepare 开销）

这一轮只做内核收口，不改行为：
把 `speculative_pair_count` 与 `max_speculative_tier` 的两次扫描合并为一次 summary 扫描。

调整：

1. 新增 `SpeculativeContactSummary`
   - `pair_count`
   - `max_tier`
2. 新增 `measureSpeculativeContactSummary(...)`
   - 单次遍历 `broadphase_pairs`
   - 复用一次 `prepareContactConstraintPair(...)`
   - 同时统计 speculative 数量和最高 tier
3. `countSpeculativeContactPairs(...)` 与 `measureMaxSpeculativeContactTier(...)`
   - 保留对外接口不变
   - 内部改为转发到 summary helper
4. 两个预算调用点改为直接读取 summary：
   - `solveContactConstraintsForPairs(...)`
   - `solveConstraintBlock(...)`

测试：

1. 新增 `measureSpeculativeContactSummary reports pair count and highest tier`
2. 既有计数/tier/预算测试保持通过

说明：

1. 这是纯收口和轻量性能路径优化，AABB 语义与 solver 结果不变。
2. 在 contact-only 与 island block 两条路径上，speculative 指标读取现在更一致，也减少了重复 prepare。

验证：

1. `zig test src/physics_kernel.zig`（`227/227`）
2. `zig test src/tick_engine.zig`（`227/227`）
3. `zig test src/physics_world.zig`（`227/227`）

### speculative 预算拼装收口为统一 helper

这一轮继续做低风险收口：
把“measure summary + boost budget”的组合逻辑做成单一 helper，去掉两处入口的重复拼装代码。

调整：

1. 新增 `boostIterationBudgetForSpeculativePairs(...)`
   - 输入：`s1024, entities, broadphase_pairs, base_budget`
   - 内部流程：
     - `measureSpeculativeContactSummary(...)`
     - `boostIterationBudgetForSpeculativeContacts(...)`
2. 两个调用点改用新 helper：
   - `solveContactConstraintsForPairs(...)`
   - `solveConstraintBlock(...)`

测试：

1. 新增 `boostIterationBudgetForSpeculativePairs uses summary-driven tier-aware boost`
   - 同一场景下对比：
     - 直接 `summary + boostIterationBudgetForSpeculativeContacts(...)`
     - 统一 helper `boostIterationBudgetForSpeculativePairs(...)`
   - 断言两者一致

说明：

1. 行为不变，属于调用层收口。
2. AABB 路径和接触求解语义未改。

验证：

1. `zig test src/physics_kernel.zig`（`228/228`）
2. `zig test src/tick_engine.zig`（`228/228`）
3. `zig test src/physics_world.zig`（`228/228`）

### speculative extra passes 计算收口为独立 helper

这一轮继续做策略层收口，不改预算语义：
把 tier + density 到 extra passes 的映射独立为 helper，降低预算主函数分支复杂度。

调整：

1. 新增 `computeSpeculativeIterationExtraPasses(...)`
   - 输入：`policy, speculative_pair_count, max_speculative_tier`
   - 输出：`extra_passes`
   - 规则保持原样：
     - 无 speculative pair：`0`
     - `none/base/mid`：low 档
     - `high`：high 档
     - 密度阈值命中时追加 dense 档
2. `boostIterationBudgetForSpeculativeContacts(...)`
   - 改为调用上述 helper
   - 仍保留 `@min(..., 6)` clamp

测试：

1. 新增 `computeSpeculativeIterationExtraPasses matches tier and density policy`
   - 覆盖 `0/base/mid/high/dense` 组合
2. 既有预算与 summary helper 测试全部保持通过

说明：

1. 这一步是纯收口与可读性提升。
2. AABB 语义、speculative summary 统计、预算最终结果保持不变。

验证：

1. `zig test src/physics_kernel.zig`（`229/229`）
2. `zig test src/tick_engine.zig`（`229/229`）
3. `zig test src/physics_world.zig`（`229/229`）

### speculative summary 增加 broken 过滤防护

这一轮补了一个防护收口：
在 `measureSpeculativeContactSummary(...)` 内部显式过滤 broken 实例，避免未来调用方忘记预过滤时把失效 pair 计入预算统计。

调整：

1. `measureSpeculativeContactSummary(...)`
   - 新增：`if (inst_a.state == .broken or inst_b.state == .broken) continue;`
2. 其余路径保持不变（summary 统计、tier 计算、budget boost 逻辑不变）。

测试：

1. 新增 `measureSpeculativeContactSummary skips broken instances`
   - 构造可形成 speculative 的 pair
   - 将其中一方状态设为 `.broken`
   - 断言：`pair_count == 0` 且 `max_tier == .none`

说明：

1. 这是 defensive hardening，不改变现有主路径行为。
2. AABB 语义与 solver 公式未改。

验证：

1. `zig test src/physics_kernel.zig`（`230/230`）
2. `zig test src/tick_engine.zig`（`230/230`）
3. `zig test src/physics_world.zig`（`230/230`）

### 预算语义补强：零 speculative pair 时忽略 tier

这一轮补了一条防误用语义测试：
当 `speculative_pair_count == 0` 时，budget boost 必须不受 tier 影响。

调整：

1. 强化 `boostIterationBudgetForSpeculativeContacts scales by speculative tier and clamps`：
   - 新增断言：`boostIterationBudgetForSpeculativeContacts(4, 0, .high) == 4`
   - 与现有 `(..., 0, .none) == 4` 形成对照

说明：

1. 这一步只加测试，不改实现。
2. 目的在于锁定函数契约，避免未来调用方错误传 tier 时造成隐性预算漂移。
3. AABB 语义与 solver 行为未改。

验证：

1. `zig test src/physics_kernel.zig`（`230/230`）
2. `zig test src/tick_engine.zig`（`230/230`）
3. `zig test src/physics_world.zig`（`230/230`）

### speculative pair budget helper 增加空列表快速路径

这一轮补了一个低风险契约/性能收口：
`boostIterationBudgetForSpeculativePairs(...)` 在空 pair 输入时直接返回基准预算（并保持 clamp）。

调整：

1. `boostIterationBudgetForSpeculativePairs(...)`
   - 新增早返回：
     - `if (broadphase_pairs.len == 0) return min(6, base_budget)`
   - 避免空列表场景进入 summary 扫描路径

测试：

1. 新增 `boostIterationBudgetForSpeculativePairs returns clamped base budget when pair list is empty`
   - 断言空列表下：
     - `base_budget=4 -> 4`
     - `base_budget=8 -> 6`（clamp 生效）

说明：

1. 行为语义与现有实现一致，仅显式化并测试固定。
2. AABB 语义、speculative summary 与求解流程未改。

验证：

1. `zig test src/physics_kernel.zig`（`231/231`）
2. `zig test src/tick_engine.zig`（`231/231`）
3. `zig test src/physics_world.zig`（`231/231`）

### iteration budget clamp 收口为统一 helper

这一轮把预算路径中重复的 `min(6, ...)` 收口到统一 helper，降低分散修改风险。

调整：

1. 新增 `clampIterationBudget(budget: u8) u8`
   - 统一封装 `@min(@as(u8, 6), budget)`
2. 下列函数改为复用该 helper：
   - `computeContactIterationBudget(...)`
   - `boostIterationBudgetForSpeculativeContacts(...)`
   - `boostIterationBudgetForSpeculativePairs(...)`（空列表快速返回分支）
   - `computeEnvironmentIterationBudget(...)`
   - `computeConstraintBlockIterationBudget(...)`

测试：

1. 新增 `clampIterationBudget enforces iteration cap at six`
   - 覆盖 `0/5/6/7/255` 输入
   - 固定上限语义

说明：

1. 属于纯收口，不改预算策略与求解行为。
2. AABB 语义与 speculative pipeline 不变。

验证：

1. `zig test src/physics_kernel.zig`（`232/232`）
2. `zig test src/tick_engine.zig`（`232/232`）
3. `zig test src/physics_world.zig`（`232/232`）

### speculative budget 加法防溢出收口

这一轮补了预算加法的防溢出护栏：
`base_budget + extra_passes` 先用宽位累加，再统一 clamp，避免极端输入下 `u8` 先溢出。

调整：

1. 新增 `addIterationBudgetWithClamp(base_budget, extra_passes)`
   - 使用 `u16` 做中间和
   - 对中间值做上界保护后回落到 `u8`
   - 统一走 `clampIterationBudget(...)`
2. `boostIterationBudgetForSpeculativeContacts(...)`
   - 改为调用 `addIterationBudgetWithClamp(...)`

测试：

1. 新增 `addIterationBudgetWithClamp avoids overflow and enforces cap`
   - 覆盖普通加法与极端输入：`(255,255)`
   - 断言始终收口到上限 `6`

说明：

1. 正常业务输入下行为不变。
2. 该改动主要提升函数在极端/误用输入下的健壮性。
3. AABB 语义和求解主流程未改。

验证：

1. `zig test src/physics_kernel.zig`（`233/233`）
2. `zig test src/tick_engine.zig`（`233/233`）
3. `zig test src/physics_world.zig`（`233/233`）

### iteration budget cap 提取为统一常量

这一轮继续做预算层收口：
把上限值 `6` 从 helper 内部硬编码提取为统一常量，减少后续调整时的散点改动。

调整：

1. 新增 `ITERATION_BUDGET_CAP: u8 = 6`
2. `clampIterationBudget(...)` 改为使用该常量
3. `clampIterationBudget` 测试中对“上限命中”断言改为引用常量

说明：

1. 纯命名与常量收口，不改行为。
2. AABB 语义、预算策略和 solver 结果保持不变。

验证：

1. `zig test src/physics_kernel.zig`（`233/233`）
2. `zig test src/tick_engine.zig`（`233/233`）
3. `zig test src/physics_world.zig`（`233/233`）

### speculative tier merge 比较逻辑收口

这一轮继续做小型可维护性收口：
把 speculative tier 的“取较高档位”比较从 summary 内联逻辑提成单独 helper。

调整：

1. 新增 `mergeHigherSpeculativeContactTier(current, candidate)`
   - 封装基于 enum 序的高档位选择
2. `measureSpeculativeContactSummary(...)`
   - 改为调用该 helper 更新 `summary.max_tier`
   - 去掉内联 `@intFromEnum(...)` 比较分支

测试：

1. 新增 `mergeHigherSpeculativeContactTier preserves max tier ordering`
   - 覆盖：相等、升级、保持高档不降级

说明：

1. 纯收口，不改变 tier 统计结果。
2. AABB、预算策略与求解行为保持不变。

验证：

1. `zig test src/physics_kernel.zig`（`234/234`）
2. `zig test src/tick_engine.zig`（`234/234`）
3. `zig test src/physics_world.zig`（`234/234`）

### speculative extra passes 内部加法防溢出

这一轮继续补预算 helper 的极端输入防护：
`computeSpeculativeIterationExtraPasses(...)` 原先在 dense 阈值命中时直接对 `u8` 做 `+=`，默认策略下不会出问题，但如果未来 policy 被调大或测试注入极端值，会存在先溢出再返回的风险。

调整：

1. `computeSpeculativeIterationExtraPasses(...)`
   - dense 额外 pass 不再直接 `extra_passes += ...`
   - 改为复用 `addIterationBudgetWithClamp(...)`
   - 极端 policy 输入也会收口到 `ITERATION_BUDGET_CAP`

测试：

1. 新增 `computeSpeculativeIterationExtraPasses clamps extreme policy inputs`
   - 覆盖 `low_tier_extra_passes = 250`
   - 覆盖 `high_tier_extra_passes = 255`
   - 覆盖 `dense_pair_extra_passes = 255`
   - 确认非零 speculative pair 下都 clamp 到上限，零 pair 仍返回 `0`

说明：

1. 默认 speculative budget 策略行为不变。
2. 这是预算 helper 的健壮性补强，不改 AABB、contact equation、summary 统计或 solver 主流程。

验证：

1. `zig test src/kcc.zig`（`236/236`）
2. `zig test src/physics_kernel.zig`（`236/236`）
3. `zig test src/tick_engine.zig`（`236/236`）
4. `zig test src/physics_world.zig`（`236/236`）

### iteration budget 递增路径统一走 clamp helper

这一轮继续把预算层的小分支统一到同一个加法入口：
`contact/environment/constraint block` 三个 iteration budget helper 原先都直接写 `budget += 1`，虽然当前默认阈值下不会越界，但和 speculative budget 的防溢出路径不一致。

调整：

1. `computeContactIterationBudget(...)`
   - stress 触发的两次预算提升改为 `addIterationBudgetWithClamp(budget, 1)`
2. `computeEnvironmentIterationBudget(...)`
   - stress 触发的两次预算提升改为同一 helper
3. `computeConstraintBlockIterationBudget(...)`
   - active subsystem 与 max stress 触发的预算提升都改为同一 helper

测试：

1. `computeContactIterationBudget increases for higher contact stress`
   - 增加 dense pair + high stress 场景，断言 clamp 到 `ITERATION_BUDGET_CAP`
2. `computeEnvironmentIterationBudget increases for higher environment stress`
   - 增加 dense instance + high stress 场景，断言 clamp 到 `ITERATION_BUDGET_CAP`

说明：

1. 默认预算结果保持不变。
2. 预算提升路径现在全部复用同一个 clamp 加法语义，后续调高基础预算或阈值时更不容易引入局部溢出/漏 clamp。
3. AABB、接触方程、prepared row、apply step 和 solver 主流程未改。

验证：

1. `zig test src/kcc.zig`（`236/236`）
2. `zig test src/physics_kernel.zig`（`236/236`）
3. `zig test src/tick_engine.zig`（`236/236`）
4. `zig test src/physics_world.zig`（`236/236`）
