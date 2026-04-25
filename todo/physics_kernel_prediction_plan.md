# physics.md 底层优先执行计划

## 目标

- 以根目录 `physics.md` 为唯一范围基准，而不是以现有 `src/chapter01-30.zig` 为主。
- 先补齐统一物理内核，再建设“可预测 / 可控”的短时预测层。
- 暂缓整批章节测试重写，测试只围绕底层能力补最小必要断言。

## 架构主线

### Layer 1: World Physics Kernel

- 统一 Query Layer：raycast / overlap / sweep / penetration 共用同一命中语义。
- 统一 Contact Classification：命中结果必须携带 body / surface / medium / material / condition。
- 统一 Contact Telemetry：摩擦、恢复、伤害修正、穿透阻力、浮力等参数由公共路径产出。
- 逐步补齐 snapshot / rewind / trace / determinism。
- 再往上承接 KCC / 车辆 / 弹道 / 灾害，而不是让这些模块各自重复接地和碰撞语义。

### Layer 1.5: Prediction

- 时间范围：1-5 秒短时预测。
- 核心能力：
  - 线性/分段线性状态外推
  - TTC
  - 冲突窗口
  - 信号灯安全通过窗口
  - 占据预测与风险评估
- 约束：
  - 预测必须建立在 Layer 1 的统一命中语义之上
  - 不在 `sensors` / `network` / `ai_traffic` 内部重复造一套预测逻辑

## 当前优先级

1. 完成查询层接触分类与元数据闭环
2. 补最小预测基座 `src/prediction.zig`
3. 用统一语义回接 `vehicle` / `ai_traffic` / `sensors`
4. 再整理章节级测试与验收矩阵

## 第一批实现切片

### Slice A: Query Metadata

- 扩展 `QueryHit`
- 提供 `ContactClassification`
- 提供 `ContactTelemetry`
- 从 `terrain` 和 `material_pairing` 统一生成命中语义

### Slice B: Prediction Substrate

- `predictLinearState`
- `computeTTC`
- `computeConflictWindow`
- `predictSignalWindow`
- `estimateSafePass`

### Slice C: Determinism / Snapshot

- 统一 world snapshot
- 最小 trace 输出
- 为 rewind 与 prediction 提供稳定状态源

## 近期不作为主线

- 不先做整批 chapter 测试重写
- 不先追求所有高级模块表面通过
- 不先做性能专项优化

## 验收口径

- 能回答“碰到了什么、是什么介质、预期接触参数是什么”
- 能回答“接下来几秒大概率会发生什么”
- 同一语义同时可被 query / vehicle / ai_traffic / sensors / prediction 复用
