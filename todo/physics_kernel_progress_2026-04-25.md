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

当前已经统一到这种表达的内容：

1. contact normal
2. contact tangent
3. environment normal

统一字段包括：

- `dir_x`
- `dir_y`
- `dir_z`
- `depth`
- `predicted_depth`

这一步的意义非常大，因为它把“方向型约束”的输入数据形状统一了。

### 8. shared apply step 已经成型

当前底层共享执行通道有：

- `applySingleDisplacementRowStep(...)`
- `applyPairDirectionalDisplacementRowStep(...)`
- `applyPairVelocityImpulseRowStep(...)`

已接入情况：

1. environment warm-start 位移
2. environment solve 位移
3. contact normal warm-start impulse
4. contact normal 位置纠正
5. contact normal 速度冲量
6. contact friction warm-start impulse
7. contact friction solve impulse

也就是说，contact/environment 的大部分 solve-step 已经不再各写专用 apply 模板。

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

当前效果：

1. contact 的 `settle + wake` 不再在多个调用点重复写。
2. environment 的 `位移 + surface response` 不再在批处理和 row 路径各写一套。
3. pair / single 的 wake 协议外形已经对齐。

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

- `169/169` 全通过

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

### B. joint 仍然最专用

joint 已经接入 row 框架，但底层仍保留大量专用求解逻辑：

- `applyJointAngularScalarCorrection(...)`
- `applyJointAxisScalarCorrection(...)`
- `applyJointDirectionalConstraint(...)`
- `applyJointAnchorDistanceConstraint(...)`

这部分还没有像 contact/environment 一样被共享 apply channel 大幅吸收。

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

1. contact 和 environment 的 row finalize 仍分开函数。
2. joint finalize 与 pair contact finalize 语义接近但没有进一步模板化。
3. constraint row build 仍然按 subsystem 代码块组织，而不是按 generic row-prep registry 组织。

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

1. 先做 `buildDirectionalNormalRow(...)`
2. 再做 `buildDirectionalTangentRow(...)`
3. 然后给 joint 定义 prepared channel
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

底层已经从“多套专用约束求解”推进到“共享 row 框架 + 共享 prepared directional channel + 共享 apply step + 共享部分 postprocess/finalize”，而且预测性已经进入 contact/environment 的求解核心；但距离完整通用 Jacobian / lambda 约束系统，仍然还有 joint 通道统一、tangent row 泛化、predictive plan 升级、以及真正通用 row primitive 这几大步。
