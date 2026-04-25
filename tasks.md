# World VM 当前任务

## 当前主线

当前主线改回底层优先。

- 以根目录 `physics.md` 为范围基准
- 先补统一物理内核
- 再补“可预测 / 可控”的短时预测层
- 暂不把“重写整批章节测试”作为最高优先级

参考执行计划：

- `todo/physics_kernel_prediction_plan.md`

## 本轮目标

1. 继续完成统一物理内核的底层闭环
2. 优先补齐“可预测 / 可控 / 可回放”的核心能力
3. 修掉仍停留在近似实现的底层查询与挤出逻辑
4. 只补内核级最小必要测试，不先重写整批 chapter

## 执行顺序

### 第一批：统一命中语义

- `src/query_types.zig`
- `src/query_world.zig`
- `src/query_raycast.zig`
- `src/query_sweep.zig`
- `src/query_penetration.zig`
- `src/material_pairing.zig`
- `src/terrain.zig`

要求：

- 同一次命中必须能说明“碰到了什么”
- query / vehicle / sensors / ai_traffic 后续必须能共用这套语义

当前状态：

- 已完成
- 已补 environment / instance 命中分类与 telemetry
- 已补 query 侧回归测试

### 第二批：最小预测基座

- 新增 `src/prediction.zig`
- 先提供：
  - 线性短时预测
  - TTC
  - 冲突窗口
  - 红绿灯安全通过窗口

要求：

- 预测范围以 1-5 秒为主
- 不允许在多个模块内重复实现相似预测逻辑

当前状态：

- 已完成
- `sensors` / `network` / `ai_traffic` 已复用统一 prediction

### 第三批：统一状态来源

- snapshot
- rewind
- trace
- determinism

要求：

- prediction 不能建立在零散状态之上
- 必须有稳定、可复用、可回放的状态输入

当前状态：

- 已完成基础闭环
- `rewind` / `vm_hook` 已支持 world snapshot / hash / simulate / diff

### 第四批：去掉底层近似实现

- `src/query_sweep.zig`
- `src/raycast.zig`
- `src/vm_hook.zig`
- `src/query_penetration.zig`
- `src/query.zig`

要求：

- `sphere cast` / `box cast` 不能继续停留在中心点近似
- penetration 结果要能给出可信的 depenetration 方向和深度
- 旧入口与新 query 子模块不能行为分裂

当前状态：

- 已完成一轮
- 已补多采样体积 cast
- 已补 AABB penetration 最小平移向量
- 已修负坐标边界崩溃

### 第五批：接触稳定与 KCC 可控挤出

- `src/tick_engine.zig`
- `src/kcc.zig`

要求：

- 小幅落地反弹要能稳定收敛，不要持续抖动
- KCC 不能只会“向上抬几格”，需要具备最小阻力挤出能力

当前状态：

- 已完成第一轮
- 已补 ground settle
- 已把 penetration 接到 KCC resolveCollision

### 第六批：统一 PhysicsWorld / TickEngine 内核语义

- `src/physics_world.zig`
- `src/tick_engine.zig`
- `src/physics.zig`
- `src/collision_event.zig`
- `src/contact_response.zig`
- `src/break_response.zig`
- `src/sleep_response.zig`

要求：

- `PhysicsWorld` 不能继续停留在骨架
- `TickEngine` 与 `PhysicsWorld` 不能在基础移动、碰撞发布、break、sleep 上各自维护一套分裂语义
- blocker-aware 三轴移动、碰撞事件发布、broken 占据移除必须统一到底层 helper

当前状态：

- 已完成当前一轮
- `PhysicsWorld` 已具备真实 authoritative tick / snapshot / debris / bus event / break / sleep 路径
- `TickEngine` 已补齐 `vel_x` / `vel_z` / `vel_y>0` 的三轴 sweep 语义
- `src/physics.zig` 已提供共享 `sweepAxis`
- `src/collision_event.zig` 已提供共享 pending collision queue
- `TickEngine` 与 `PhysicsWorld` 已统一为“先收集、后发布”的碰撞事件节奏
- 已补重复 collision event 去重与重复 BREAK intent 去重
- 已补 `TickEngine` 的 lateral wall / upward ceiling / MOVE intent 回归测试
- 已补“支撑消失唤醒”底层语义：支撑体 break 后，上方 resting 物体会被即时唤醒
- 已补最小 AABB face-contact manifold 基座，可输出 4 点支撑接触
- `query_penetration` 已开始输出最小 manifold 点集，不再只返回 MTV 深度和方向
- `vm_hook` 已导出 penetration manifold，外部链路可直接读取 depth / dir / point_count / points
- 已修重力终端速度钳制，`PhysicsWorld` / `TickEngine` 已统一走共享 vertical velocity clamp

## 验收条件

1. 底层查询能稳定输出统一接触分类与接触参数
2. 新增测试覆盖环境命中与实例命中的元数据一致性
3. 后续预测层可以直接复用 query 结果，而不需要再猜表面和介质
4. cast / penetration / KCC depenetration 不再依赖明显错误的中心点近似
5. snapshot diff 对外链路能覆盖新增底层子系统
6. `PhysicsWorld` 与 `TickEngine` 对基础 blocker-aware 三轴运动和碰撞发布不再出现明显分叉
7. broken 实例不再通过 direct occupancy scan 或 rebuild occupancy 残留阻挡

## 本轮暂不处理

- 整批 `src/chapter*.zig` 重写
- Python 测试体系重写
- 所有高级章节“表面通过”整理
- 性能专项优化

## 当前下一步

1. 继续补 contact / stacking / manifold 稳定性
2. 继续抽离 `TickEngine` / `PhysicsWorld` 的接触编排 helper，减少重复 orchestration
3. 逐步消除 `query.zig` 与 `query_*` 子模块的双轨结构
4. 继续评估 broadphase / manifold / stacking solver 是否仍是当前最大底层缺口
