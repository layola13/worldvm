# physics.md 底层完成度审计

审计日期：2026-04-25

范围基准：

- 以根目录 `physics.md` 为唯一范围源
- 重点判断“底层是否具备能力”，不是判断章节测试文件数量
- 状态分级：
  - `已具备`：存在真实模块，已接入主链路，且不是纯占位
  - `部分具备`：有模块或接口，但求解深度、接线范围、状态回放、测试覆盖仍明显不足
  - `未具备`：基本没有底层实现，或仅有章节名义测试/场景摆拍

---

## 总结结论

当前项目已经从“只有章节名义覆盖”推进到“存在一套可运行的统一物理底层骨架”，但离 `physics.md` 覆盖的完整底层仍有明显距离。

当前更准确的判断是：

- 离散体素世界、基础连续运动、基础碰撞、基础关节、KCC、弹道、破坏、布娃娃、车辆子系统、地表/天气、灾害、传感器、网络回滚、世界快照/预演/预测，这些方向**都已有底层模块**
- 但真正达到 `physics.md` 所要求的“稳定、高保真、统一求解、可回放、可解释”的，主要还是 `部分具备`
- 真正比较扎实的底层主线目前是：
  - 统一 query/contact 语义
  - snapshot / rewind / determinism 基座
  - 1-5 秒短时 prediction 基座
- 当前最主要缺口不是“没有更多章节测试”，而是：
  - 接触流形/接触求解器深度不足
  - query sweep/cast 仍有简化实现
  - 软体/粒子/高级流体仍缺真实底层
  - `PhysicsWorld` 仍是骨架，未成为唯一统一调度真入口
  - 若干高级系统虽可 snapshot，但行为保真度与 `physics.md` 目标仍有距离

---

## 能力分组审计

### 1. 基础运动、重力、阻尼

状态：`已具备`

依据：

- `src/physics.zig`
- `src/tick_engine.zig`

已有内容：

- 重力
- 线性阻尼
- 角阻尼
- 速度积分
- 睡眠阈值与唤醒

不足：

- 仍以简化离散步进和整数速度为主
- 质量、惯性、真实时间尺度仍较粗糙

### 2. 基础碰撞检测与去穿透

状态：`部分具备`

依据：

- `src/physics.zig`
- `src/collision.zig`
- `src/tick_engine.zig`

已有内容：

- 体素占据检测
- AABB 基础命中
- 初步连续下落阻挡
- 基础 impact/damage 计算

不足：

- 缺少成熟接触流形生成器
- 缺少多接触点稳定求解器
- 碰撞法线/接触点求解能力主要集中在 query/raycast 路径，不是完整 solver

### 3. 摩擦、弹性、材质配对

状态：`部分具备`

依据：

- `src/physics.zig`
- `src/material_pairing.zig`
- `src/terrain.zig`
- `src/query_types.zig`

已有内容：

- restitution
- friction
- surface / medium / material / condition 统一语义
- query 返回 contact telemetry

不足：

- 仍偏参数表驱动
- 缺少稳定接触求解中的静摩擦/动摩擦切换细节
- 缺少各向异性摩擦、滚动摩擦等更深层 solver 集成

### 4. 堆叠稳定性与休眠

状态：`部分具备`

依据：

- `src/tick_engine.zig`
- `src/physics.zig`

已有内容：

- sleep / wake 机制
- 基本 resting 状态

不足：

- 没有成熟 island solver
- 高层堆叠、长时间稳定、复杂支撑面稳定性仍未证明
- `physics.md` 要求的稳定 manifold 支撑尚不充分

### 5. 关节与约束

状态：`部分具备`

依据：

- `src/joint.zig`
- `src/tick_engine.zig`
- `src/rewind.zig`

已有内容：

- fixed / hinge / spring / ball_socket
- 全局 joint system
- snapshot / restore / diff 已接入

不足：

- pulley、breakable、复杂限位/马达等能力仍不完整
- 约束求解精度与稳定性距离高级物理仍有差距

### 6. CCD / 高速运动

状态：`部分具备`

依据：

- `src/ccd.zig`
- `src/ballistics.zig`
- `src/vm_hook.zig`

已有内容：

- TOI 基础计算
- 弹道路径与穿透计算

不足：

- angular CCD 仍不充分
- 复杂多物体同帧高速碰撞的一致性未证明
- 某些 cast 仍是中心点近似

### 7. 射线、Sweep、Overlap、Penetration Query

状态：`部分具备`

依据：

- `src/query_world.zig`
- `src/query_raycast.zig`
- `src/query_sweep.zig`
- `src/query_overlap.zig`
- `src/query_penetration.zig`
- `src/query_types.zig`

已有内容：

- 统一 QueryHit 语义
- environment / instance 命中元数据闭环
- 接触分类与接触参数统一表达

不足：

- `sphere_cast` / `box_cast` / query sweep 仍有简化实现
- 旋转盒 sweep 和真实体积 cast 还不够

### 8. KCC

状态：`部分具备`

依据：

- `src/kcc.zig`
- `src/rewind.zig`

已有内容：

- grounded 检查
- move / jump / crouch
- wall slide
- collision resolve
- nearby bodies push
- snapshot / restore / diff

不足：

- step offset、slope limit、ceiling slide、snap to ground 这些 `physics.md` 关键点还未形成完整高置信实现矩阵
- 与统一 query/contact 层的更深复用还可以加强

### 9. 弹道与穿甲穿透

状态：`部分具备`

依据：

- `src/ballistics.zig`
- `src/rewind.zig`

已有内容：

- projectile system
- kinetic energy
- penetration depth
- layered hit
- deflection
- fragment generation
- snapshot / restore / preview

不足：

- 穿透模型仍有简化公式
- 高级材料层、多层结构、偏转/破片反馈还未达到硬核 FPS 级别

### 10. 动态破坏与碎片

状态：`部分具备`

依据：

- `src/destruction.zig`
- `src/collision.zig`
- `src/rewind.zig`

已有内容：

- destroyable state
- fracture / crack / integrity
- debris system
- snapshot / restore / diff

不足：

- 真实结构传播、稳定拓扑断裂、复杂连锁坍塌仍偏初级
- 与世界主求解器的一体化程度有限

### 11. 布娃娃 / 生物力学

状态：`部分具备`

依据：

- `src/ragdoll.zig`
- `src/joint.zig`
- `src/rewind.zig`

已有内容：

- humanoid ragdoll
- motor joints
- active pose blending
- balance state / resurrection
- snapshot / restore / diff

不足：

- 真实主动布娃娃控制仍较浅
- 生物力学、肌肉组织、复杂地面反应远未覆盖 `physics.md` 后段目标

### 12. 车辆总成

状态：`部分具备`

依据：

- `src/vehicle.zig`
- `src/tire.zig`
- `src/suspension.zig`
- `src/drivetrain.zig`
- `src/aerodynamics.zig`
- `src/braking.zig`
- `src/terrain.zig`
- `src/rewind.zig`

已有内容：

- vehicle state
- tire / suspension / drivetrain / aero / braking 子模块
- terrain/weather 对摩擦和滚阻有支持
- 各子系统大多进入 snapshot / restore / diff

不足：

- 这些模块之间仍偏“并列存在”，不等于完整高保真整车解算
- steering / handling / drift / weight transfer / airborne / towing 等更多 `physics.md` 车辆章节尚未形成系统闭环
- `aero_devices` 已 snapshot，但设备级行为仍较轻

### 13. 地表材质与天气

状态：`已具备`

依据：

- `src/terrain.zig`
- `src/material_pairing.zig`
- `src/query_types.zig`

已有内容：

- surface type
- medium type
- weather
- friction / rolling resistance / visibility
- hydroplaning risk
- 已进入 snapshot / restore / diff

不足：

- 更复杂地形变形、积雪堆积、动态路面退化还不充分

### 14. 流体 / 浮力 / 力场

状态：`部分具备`

依据：

- `src/tick_engine.zig`
- `src/terrain.zig`
- `src/disasters.zig`

已有内容：

- buoyancy
- force field
- explosion force
- 水深/积水风险

不足：

- 没有真实流体求解器
- 波浪、流场、体积守恒、浸没体积、拖曳等仍明显不足
- 章节 21 的“流体测试”主要还是摆场景，不构成底层完成

### 15. 粒子

状态：`未具备`

依据：

- 仓库中没有独立 `particles` 底层模块
- `src/chapter22_particles.zig` 仅是场景式章节测试

结论：

- 只有章节名义测试，没有真实粒子系统底层

### 16. 软体 / 布料 / 组织形变

状态：`未具备`

依据：

- 仓库中没有独立 softbody 底层模块
- `src/chapter23_softbody.zig` 仅是场景式章节测试

结论：

- 没有弹簧网络、体积保持、布料约束、自碰撞等真实底层

### 17. 灾害 / 极端环境

状态：`部分具备`

依据：

- `src/disasters.zig`
- `src/terrain.zig`
- `src/rewind.zig`

已有内容：

- earthquake / tsunami / hurricane / wildfire 等事件枚举与参数
- 风场、热辐射、地面位移、连锁反应
- snapshot / restore / diff

不足：

- 更接近“参数化灾害系统”，不是真正多介质耦合物理
- 与车辆/KCC/破坏深度联动仍有限

### 18. 传感器 / 感知

状态：`部分具备`

依据：

- `src/sensors.zig`
- `src/query_*`
- `src/rewind.zig`

已有内容：

- sensor state
- fusion state
- degradation / interference
- occlusion
- 通过 shared prediction 做短时位置预测
- snapshot / restore / diff

不足：

- 仍偏规则化抽象
- 与占据、可见性、多帧目标跟踪、复杂天气耦合还较轻

### 19. 网络同步 / 回滚 / 确定性

状态：`已具备`

依据：

- `src/network.zig`
- `src/rewind.zig`
- `src/tick_engine.zig`
- `src/vm_hook.zig`

已有内容：

- replica / input log
- rollback / reconcile
- CRC / determinism proof
- world snapshot / hash / restore
- future simulated preview
- structured diff

备注：

- 这是当前最完整的底层主线之一
- 仍需继续扩展更多系统字段的接口可见性与回归测试

### 20. 可预测 / 可控（短时预测层）

状态：`已具备`

依据：

- `src/prediction.zig`
- `src/sensors.zig`
- `src/network.zig`
- `src/ai_traffic.zig`
- `src/vm_hook.zig`

已有内容：

- linear prediction
- TTC
- conflict window
- signal window
- safe pass
- snapshot-based prediction / forecast / preview

备注：

- 这是新补上的关键底层能力
- 已经形成“统一预测基座”，不是各模块重复造轮子

### 21. Crash Defense / 崩溃防御

状态：`已具备`

依据：

- `src/crash_defense.zig`
- `src/rewind.zig`

已有内容：

- NaN / Inf 检查
- 场景 clamp
- emergency stop
- snapshot recovery
- stuck 检查
- load reduction
- snapshot / restore / diff

### 22. 统一世界调度器

状态：`部分具备`

依据：

- `src/tick_engine.zig`
- `src/physics_world.zig`

现状判断：

- 真正工作中的主链路是 `tick_engine`
- `physics_world.zig` 已经具备：
  - broadphase 实现
  - event bus 发布路径
  - rewind snapshot 记录
- 但它仍更像“统一骨架入口”，还不是项目唯一 authoritative physics world

结论：

- 项目有“统一世界调度器方向”
- 但还没有完全收敛成唯一 authoritative physics world

### 23. 可扩展性 / 大规模性能

状态：`部分具备`

依据：

- `scene1024`
- occupancy rebuild
- rewind buffer
- 若干 benchmark / scalability chapter

不足：

- `physics_world.zig` 中 broadphase 仍是 TODO
- 目前大量逻辑仍依赖全量遍历或简化路径
- 对 `physics.md` 后段超大规模、多系统联动场景还不够

---

## 当前最关键的底层缺口

### A. 接触求解器深度不足

表现：

- 缺 manifold 稳定支撑
- 缺多接触点持续求解
- 缺高质量摩擦/堆叠/接缝消除

影响：

- 前 1-100 章里大量“看似基础”的稳定性问题都会反复出现

### B. Sweep / Shape Cast 仍有简化近似

表现：

- `raycast.zig` / `query_sweep.zig` / `vm_hook.zig` 中仍有中心点近似

影响：

- CCD、KCC、车辆轮地检测、体积感知精度都会受限

### C. `PhysicsWorld` 还未真正接管全链路

表现：

- `physics_world.zig` 仍留有 TODO
- 实际 authoritative path 仍在 `tick_engine`

影响：

- 子系统接线容易分散
- 底层状态一致性维护成本高

### D. 软体 / 粒子 / 高级流体仍是空洞区

表现：

- 章节测试存在
- 底层模块不存在或不成系统

影响：

- `physics.md` 后段 700-900 的大量能力实际上没有底层支撑

### E. 车辆全系统仍未闭环成整车求解

表现：

- tire / suspension / drivetrain / aero / braking 已存在
- 但系统集成与整车状态耦合仍有限

影响：

- 赛车、漂移、空中姿态、多车网络这些高级章节很难真正稳定通过

---

## 建议的后续底层优先顺序

1. 把 `query sweep / sphere cast / box cast` 从中心点近似升级为真实形状查询
2. 强化 contact manifold / friction / stacking solver
3. 让 `PhysicsWorld` 接管 broadphase / events / snapshot，减少分裂入口
4. 把车辆子系统收敛为统一整车更新链，而不是模块并列
5. 明确决定：
   - 粒子是否要做真实底层
   - 软体是否要做真实底层
   - 高级流体是否只保留近似层
6. 继续补 snapshot / diff / hook 可见性，保证所有底层状态可回放、可比对、可预测

---

## 对章节测试的审计结论

对 `physics.md` 而言，当前最大的偏差不是“测试少”，而是：

- 有不少章节文件只是把实体摆进场景
- 断言只验证 `ticks_to_stable > 0` 之类的弱条件
- 这不能证明底层能力真实存在

因此后续测试原则应是：

- 先补底层能力
- 再用针对性内核测试验证能力
- 章节级测试只能作为验收层，不应冒充底层完成度证明
