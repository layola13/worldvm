# WorldVM Next TODO

## 目标判断

当前仓库不是“差几个接口”，而是“缺一个统一、可验证的物理运行时骨架”。

现状：

- `Scene1024 + Entity16 + TickEngine` 已经能支撑基础体素世界、简单重力、流动、局部破碎。
- `kcc / ragdoll / vehicle / tire / suspension / drivetrain / rewind / ballistics` 已经有模块和 FFI。
- 但这些高级模块大多没有真正接入主世界 Tick，属于旁路系统、独立计算器或样机实现。

结论：

- 想支撑 `physics.md` 的复杂场景，尤其是 `101-190`、`141-360`、`801-900`，下一阶段必须优先补“统一物理世界骨架”，而不是继续横向堆新接口。

## 当前核心问题

### 1. 主循环能力不足

当前 `TickEngine` 真正稳定支持的能力主要是：

- 重力
- 下落
- 简单流体平移
- 基础休眠
- 简单破坏状态切换

当前缺失：

- 接触流形（contact manifold）
- 稳定堆叠求解
- 水平碰撞与滑动
- Kinematic / Dynamic 统一求解
- 统一约束求解迭代
- 统一 CCD / sweep
- 统一材质混合规则

影响：

- `physics.md` 前 100 条只能覆盖基础子集。
- KCC、载具、人体都没有可靠承载层。

### 2. 高级模块没有接入统一世界

当前仓库中已有：

- `kcc.zig`
- `ragdoll.zig`
- `vehicle.zig`
- `tire.zig`
- `suspension.zig`
- `drivetrain.zig`
- `ballistics.zig`
- `rewind.zig`

但问题是：

- 这些模块虽然能通过 `vm_hook.zig` 暴露接口，却没有统一纳入 `TickEngine.stepTick()`。
- 结果是 API 看起来很多，但不代表它们已经成为同一个物理世界的一部分。

### 3. 查询层设计不稳

需要立即重构世界查询接口。

现状问题：

- `physics.isOccupiedGlobal(...)` 同时承担“世界查询”和“忽略自身体素”的职责。
- `kcc.zig` 和 `vehicle.zig` 里把 `undefined` 作为 `inst` 传入这个函数，设计上不安全。

下一步必须拆成两层：

- 纯世界占用查询
- 带忽略 self / ignore mask / layer mask 的碰撞查询

否则：

- KCC grounded
- 车辆接地
- sweep/cast
- trigger
- raycast ignore self

这些能力都会反复出错。

### 4. 回滚系统还不是“世界回滚”

当前 `rewind.zig` 只是在记录轻量状态，不是完整世界快照。

缺失：

- `Scene1024` 快照
- `Instance` 阵列快照
- `TickEngine` 内部状态
- `Joint/KCC/Vehicle/Ragdoll` 状态
- 输入日志
- 快进重演
- 世界级哈希

结论：

- 当前实现不能支撑 `physics.md` 181-190 的网络回滚、确定性验证和 lag compensation。

### 5. 人体系统目前只是骨架

当前 `ragdoll.zig` 能算起点，但远不够支撑新增的人体场景。

当前能力更接近：

- 简单关节链
- 基础断肢开关
- 极简 update

缺失：

- 主动布娃娃（Active Ragdoll）
- PD 电机
- 关节扭矩控制
- 质心平衡
- 骨折阈值
- 肌肉刚度衰减
- 软组织 / 布料 / 软接触
- 人体动作状态机与恢复控制

结论：

- `801-900` 目前只具备极低比例的基础骨架支持。

## 优先级路线

### P0：统一 PhysicsWorld 骨架

目标：

- 把当前“分散模块 + FFI 入口”改造成统一世界求解架构。

要做：

- 新建统一 `PhysicsWorld` 或扩展 `TickEngine` 为世界级协调器
- 把以下状态纳入同一个 world：
- `Scene1024`
- `instances`
- `joints`
- `kcc`
- `vehicles`
- `ragdolls`
- `projectiles`
- `rewind metadata`
- 明确固定步长 `fixed_dt`
- 统一 `pre_step -> broadphase -> narrowphase -> solve -> integrate -> events -> snapshot`

验收：

- 每帧只有一个物理主入口
- 所有高级模块不再依赖独立旁路更新

### P1：重构查询层

目标：

- 建立可靠的碰撞/射线/扫掠查询基础设施

要做：

- 拆分 `isOccupiedGlobal`
- 新增：
- `queryVoxel(world_x, world_y, world_z)`
- `queryOverlap(shape, filters)`
- `raycastSingle(filters)`
- `raycastAll(filters)`
- `sphereCast(filters)`
- `capsuleCast(filters)`
- `boxCast(filters)`
- `computePenetration(filters)`
- 支持：
- `ignore self`
- `layer mask`
- `trigger policy`
- `dynamic/static/kinematic filter`

验收：

- KCC grounded 不再依赖非法指针
- 车辆轮胎接地使用统一查询
- 射线/扫掠/穿透接口能复用同一过滤机制

### P2：重构刚体与接触求解

目标：

- 让主世界真正具备可扩展刚体求解能力

要做：

- 增加 broadphase
- 增加 narrowphase
- 建立 contact manifold
- 增加多点接触支撑
- 引入迭代求解器
- 支持静摩擦/动摩擦/恢复系数组合
- 支持 Kinematic 与 Dynamic 交互
- 支持更稳定的 sleep/wake 传播
- 整理材料组合策略

验收：

- `physics.md` 11-40、71-80 的大量场景能进入“可做、可测”状态

### P3：实现真正的 KCC

目标：

- 让 `101-110` 成为可靠地基，同时服务人体运动逻辑

要做：

- 胶囊体 sweep
- slide 逻辑
- step offset
- slope limit
- snap to ground
- depenetration
- moving platform 跟随
- crouch/stand clearance check
- player vs player resolution
- grounded 缓冲状态

验收：

- 101-110 可以逐条写回归测试
- 人体跑跳、攀爬、滑铲等动作有统一运动学基础

### P4：世界级回滚与确定性

目标：

- 建立真正可用于网络仿真的回滚系统

要做：

- `saveWorldState()`
- `loadWorldState()`
- `computeWorldHash()`
- `recordInputLog()`
- `fastForwardTicks(n)`
- 规范确定性数学策略：
- 固定步长
- 受控浮点或定点
- 稳定遍历顺序
- 稳定排序规则

验收：

- 两次相同输入序列可比对世界哈希
- 支持指定 tick 回滚再快进

### P5：载具从散件升级为闭环系统

目标：

- 让 `vehicle/tire/suspension/drivetrain` 不再是分散函数集合

要做：

- 每轮状态统一更新
- 轮胎接地采样
- 悬挂行程计算
- 垂向载荷
- slip ratio / slip angle
- longitudinal / lateral tire force
- 差速器扭矩分配
- 车身受力汇总
- 空气动力学接入
- 制动、ABS、手刹接入

验收：

- `141-150` 起码能进入真实物理链路
- 后续 `201-360` 才有工程基础

### P6：被动布娃娃

目标：

- 让人体先具备“死后正确”的物理行为

要做：

- 每个肢体独立刚体化
- 关节限制与锥约束
- 断裂阈值
- 每肢体碰撞体
- 自碰撞过滤
- 动量继承
- 尸体稳定堆叠

验收：

- `131-140` 与 `801-805` 的基础关节场景可测

### P7：主动布娃娃与人体平衡

目标：

- 支撑新增的人体 801-900 场景中的关键子集

要做：

- pose target
- PD motor
- CoM 估算
- balance controller
- recovery controller
- stiffness / fatigue scaling
- gait phase
- fall / stumble / recovery 状态机

验收：

- 可先覆盖：
- 站立平衡
- 轻推恢复
- 重心偏移
- 冰面打滑
- 绊倒
- 醉酒延迟控制

### P8：弹道穿透与破坏升级

目标：

- 支撑弹药、破碎、人体穿透类场景

要做：

- 材料密度、厚度、孔隙率
- 射线穿透距离
- 穿透后能量衰减
- 分层命中结果
- 运行时拓扑切分
- 质量与重心重算
- 碎片实例生成

验收：

- `111-130`
- `121-130`
- `875` 一类穿刺/嵌入场景

## 面向 physics.md 的现实覆盖判断

### 目前相对可利用的基础

- 基础重力与离散下落
- 简化阻尼
- 基础 joint 入口
- 基础 raycast / cast 入口
- 少量 ballistics / destruction 辅助函数

### 目前明显不足的区域

- KCC
- 稳定接触求解
- 多点接触流形
- 世界级回滚
- 深度载具动力学
- 主动布娃娃
- 生物力学
- 软体/布料/肌肉

### 粗略完成度判断

- `physics.md 1-100`：约 `30% - 40%`
- `101-190`：约 `10% - 20%`
- `141-360`：约 `15% - 25%`
- `801-900`：约 `10% - 15%`

## 建议执行顺序

不要继续优先新增零散 API。

建议顺序：

1. `PhysicsWorld` 统一调度
2. 查询层重构
3. 接触与约束求解器升级
4. 真正的 KCC
5. 世界级快照/回滚/哈希
6. 载具闭环
7. 被动布娃娃
8. 主动布娃娃
9. 弹道穿透与拓扑破碎

## 下一步最值得立刻落地的任务

如果只选一个下一任务，优先做：

### Task A：统一查询层

原因：

- KCC、载具、射线、人体、弹道、触发器都会依赖它。
- 这是所有高级功能的共同底座。

### Task B：统一 PhysicsWorld step

原因：

- 现在最大的问题不是模块数量，而是模块不在同一个世界里。

### Task C：KCC 真解算器

原因：

- 它直接决定 `101-110` 和大批人体移动场景是否有可能成立。

## 完成标准

下一阶段不应再以“又新增了多少接口”衡量进展。

应以以下标准衡量：

- 是否进入统一主 Tick
- 是否可写 headless 回归测试
- 是否能输出可比对的世界状态
- 是否能稳定回放/回滚
- 是否能作为后续载具与人体的共同底层

## Task A 详细设计：统一查询层

### 目标

建立一个“所有物理子系统共享”的查询层，解决当前以下问题：

- 世界占用查询与自体忽略逻辑耦合
- KCC 和 Vehicle 通过非法 `undefined` 指针调用世界查询
- 射线、体积扫掠、地面检测、触发器、穿透深度各自分散
- 缺少统一过滤器，无法稳定支持 `ignore self / layer mask / trigger policy / dynamic-static-kinematic`

最终目标不是新增几个函数，而是形成一个统一的 `query API surface`。

---

### 一、设计原则

#### 1. 查询必须是纯函数风格

查询层不能隐式依赖调用方类型，也不能依赖某个 `Instance*` 是否存在。

必须满足：

- 输入明确
- 输出明确
- 不修改世界
- 不依赖未定义内存

#### 2. 忽略 self 必须显式表达

不能再通过把 `inst` 传进去，然后在查询函数内部偷偷做“自体排除”。

必须显式支持：

- 不忽略任何对象
- 忽略某个实例
- 忽略某组实例
- 忽略某类层

#### 3. 所有查询共享一套过滤器

以下查询必须统一过滤策略：

- voxel query
- overlap query
- raycast
- sphere cast
- capsule cast
- box cast
- penetration query

否则上层会出现：

- 射线能忽略自身，KCC 不能
- trigger 行为在不同 API 下不一致
- 车辆悬挂与角色 grounded 判定标准不同

#### 4. 查询层先做正确，再做快

第一阶段优先：

- API 正交
- 过滤逻辑统一
- 结果语义稳定
- headless 测试可断言

第二阶段再做：

- broadphase
- 缓存
- 稀疏页索引
- SIMD

---

### 二、建议新增的模块结构

建议把查询层从 `physics.zig` 里拆出来，新增：

- `src/query_types.zig`
- `src/query_filters.zig`
- `src/query_voxel.zig`
- `src/query_overlap.zig`
- `src/query_raycast.zig`
- `src/query_sweep.zig`
- `src/query_penetration.zig`
- `src/query_world.zig`

如果你不想拆太碎，最小可行方案也至少要有：

- `src/query.zig`
- `src/query_types.zig`

但不建议继续把所有查询塞回 `physics.zig`。

原因：

- `physics.zig` 当前已经同时承担重力、阻尼、流体、破坏、AABB、push
- 再塞查询只会继续形成“大杂烩内核”

---

### 三、核心数据结构

#### 1. QueryWorldView

用途：

- 统一把世界数据打包传给查询函数

建议字段：

```zig
pub const QueryWorldView = struct {
    s1024: *scene1024.Scene1024,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
};
```

意义：

- 所有查询都只吃一个 world view
- 避免函数签名反复传 `s1024 + entities + instances`

#### 2. QueryLayerMask

当前仓库没有完整 collision layer 体系，查询层应该先预留。

建议：

```zig
pub const QueryLayerMask = u32;
```

在 `Instance` 或附属状态中后续补：

- `collision_layer`
- `collision_mask`
- `is_trigger`
- `body_type`

#### 3. BodyType

建议补一个统一枚举：

```zig
pub const BodyType = enum(u8) {
    static,
    dynamic,
    kinematic,
    sensor,
};
```

说明：

- 当前项目大量逻辑靠 `flags & 0x01` 判断 static，不够扩展
- KCC、moving platform、trigger、vehicle wheel proxy 都需要更清晰的 body type

#### 4. QueryFilter

这是整个查询层的核心。

建议：

```zig
pub const QueryFilter = struct {
    layer_mask: QueryLayerMask = 0xFFFFFFFF,
    include_static: bool = true,
    include_dynamic: bool = true,
    include_kinematic: bool = true,
    include_sensors: bool = false,
    ignore_environment: bool = false,
    ignore_instance_idx: ?u8 = null,
    ignore_entity_id: ?u16 = null,
    ignore_trigger_only: bool = false,
};
```

第一阶段可以先只真正实现：

- `ignore_instance_idx`
- `include_static`
- `include_dynamic`
- `include_sensors`
- `ignore_environment`

但结构先立好。

#### 5. QueryHit

所有射线/扫掠/API 的标准命中结果应尽量统一。

建议：

```zig
pub const QueryHit = struct {
    hit: bool,
    distance: f32,
    toi: f32,
    position_x: f32,
    position_y: f32,
    position_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    instance_idx: i16,
    entity_id: i32,
    hit_environment: bool,
    hit_sensor: bool,
};
```

说明：

- `distance` 给 raycast 用
- `toi` 给 sweep / CCD 用
- `instance_idx` 比只返回 `entity_id` 更有用
- `hit_environment` 解决场景体素 vs 实例体素的区分

#### 6. OverlapResult

建议：

```zig
pub const OverlapResult = struct {
    hit: bool,
    count: u16,
    first_instance_idx: i16,
    environment_overlap: bool,
};
```

第一阶段可以先不返回完整列表，只返回：

- 是否重叠
- 第一个命中的实例

后面再扩展 `OverlapBuffer`

#### 7. PenetrationResult

用于 depenetration、KCC 卡墙、人体挤压、生成分离方向。

建议：

```zig
pub const PenetrationResult = struct {
    overlapping: bool,
    depth: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    instance_idx: i16,
};
```

---

### 四、建议的 API 分层

#### L0：最基础体素查询

这些 API 必须最先做稳定。

建议函数：

```zig
pub fn queryEnvironmentVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool
pub fn queryInstanceVoxel(world: *const QueryWorldView, inst_idx: u8, gx: i32, gy: i32, gz: i32) bool
pub fn queryAnyVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit
```

职责：

- 单点占用查询
- 明确区分环境占用与实例占用
- 作为所有高层 query 的基元

#### L1：体积重叠查询

建议支持形状：

- AABB
- sphere
- capsule

建议函数：

```zig
pub fn overlapAABB(world: *const QueryWorldView, aabb: AABB, filter: QueryFilter) OverlapResult
pub fn overlapSphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, filter: QueryFilter) OverlapResult
pub fn overlapCapsule(world: *const QueryWorldView, ..., filter: QueryFilter) OverlapResult
```

用途：

- KCC standing clearance
- ragdoll 肢体碰撞检测
- spawn 时 depenetration 检查
- trigger volume 检测

#### L2：射线查询

建议函数：

```zig
pub fn raycastSingle(world: *const QueryWorldView, ray: Ray, filter: QueryFilter) QueryHit
pub fn raycastAll(world: *const QueryWorldView, ray: Ray, filter: QueryFilter, out_hits: []QueryHit) u16
```

需要统一支持：

- `ignore self`
- `layer mask`
- `hit sensors?`
- `backface policy`
- `max distance`

用途：

- 61-70
- vehicle suspension ray
- AI 视线遮挡
- gunfire hitscan

#### L3：扫掠查询

建议函数：

```zig
pub fn sphereCast(world: *const QueryWorldView, ..., filter: QueryFilter) QueryHit
pub fn capsuleCast(world: *const QueryWorldView, ..., filter: QueryFilter) QueryHit
pub fn boxCast(world: *const QueryWorldView, ..., filter: QueryFilter) QueryHit
```

用途：

- KCC movement sweep
- 角色落地预判
- 高速动态障碍物探测
- CCD 前端

#### L4：穿透与分离查询

建议函数：

```zig
pub fn computePenetrationAABB(world: *const QueryWorldView, aabb: AABB, filter: QueryFilter) PenetrationResult
pub fn computePenetrationCapsule(world: *const QueryWorldView, ..., filter: QueryFilter) PenetrationResult
```

用途：

- spawn overlap 修复
- KCC 被门夹住时挤出
- ragdoll 尸堆稳定化
- player vs player 挤压

---

### 五、对现有代码的具体改造点

#### 1. 处理 `physics.isOccupiedGlobal`

当前这个函数承担了过多职责，建议拆成：

- `queryEnvironmentVoxel(...)`
- `queryVoxelWithFilter(...)`
- `queryOverlapEntityAtPose(...)`

现有函数可以暂时保留一版 compatibility wrapper，但要标记为过渡 API。

建议迁移步骤：

1. 先新增新函数
2. 让 `physics.checkFall/checkContinuousFall/checkFlow/checkPush` 改用新函数
3. 再让 `kcc.zig` 和 `vehicle.zig` 改用新函数
4. 最后把旧函数降级为内部 helper 或删除

#### 2. 改造 `kcc.zig`

要改掉的点：

- grounded 查询不能再传 `undefined`
- collision 检测不能直接枚举粗糙栅格点并依赖旧查询函数

建议替换为：

- `overlapCapsule()`
- `capsuleCast()`
- `computePenetrationCapsule()`

短期过渡方案：

- 先保留离散采样
- 但底层调用改成纯世界查询 API

#### 3. 改造 `vehicle.zig`

要改掉的点：

- grounded 检测不能再传 `undefined`
- 每个轮位应该使用统一的 `raycastSingle()`

建议：

- 先把 `checkGrounded()` 改成 4 个轮位向下 raycast
- 后续悬挂系统直接复用这一套

#### 4. 改造 `vm_hook.zig`

需要新增或重构的导出接口：

- `raycast_all`
- `capsule_cast`
- `overlap_capsule`
- `compute_penetration_capsule`
- `query_voxel`

同时要统一返回结构，不要继续靠“手写 float 数组约定”无限扩张。

建议新增 FFI 结构体：

- `QueryHitFFI`
- `PenetrationResultFFI`

这样 Python 侧能稳定断言。

#### 5. 改造 `worldvm.py`

当前 Python 封装大量接口都是零散的。

查询层完成后建议新增高层封装：

- `query_voxel(x, y, z, ...)`
- `raycast_single(...)`
- `raycast_all(...)`
- `capsule_cast(...)`
- `overlap_capsule(...)`
- `compute_penetration(...)`

同时：

- 不要再让 Python 测试直接拼裸 float 数组
- 统一封装为 dict 或 ctypes struct

---

### 六、过滤规则的最低落地版本

为了尽快落地，过滤器可以分两阶段。

#### 第一阶段必须实现

- 忽略单个实例 `ignore_instance_idx`
- 是否包含静态
- 是否包含动态
- 是否命中环境
- 是否命中传感器

#### 第二阶段再补

- layer mask
- collision mask
- ignore entity id
- ignore group
- trigger-only query
- query owner categories

---

### 七、实现顺序建议

#### 第 1 步：新增 query types

先落地：

- `QueryWorldView`
- `QueryFilter`
- `QueryHit`
- `OverlapResult`
- `PenetrationResult`

目标：

- 先稳定签名与返回值

#### 第 2 步：实现 L0 单点查询

先做：

- 环境体素查询
- 实例体素查询
- 单点带过滤查询

目标：

- 替换当前危险的 `isOccupiedGlobal` 使用路径

#### 第 3 步：迁移现有 physics 核心

把以下函数迁移到新查询层：

- `checkFall`
- `checkContinuousFall`
- `checkFlow`
- `checkPush`

目标：

- 主 Tick 继续可用
- 查询接口完成第一次实战接入

#### 第 4 步：修 KCC grounded/collision

先不追求完整 KCC，只修掉非法调用并统一到 query API。

目标：

- grounded 和 collision 检测进入稳定状态

#### 第 5 步：修 Vehicle grounded

目标：

- 给后续 suspension 做准备

#### 第 6 步：补 RaycastAll 与 CapsuleCast

目标：

- 为 `physics.md 61-70`、KCC 和载具提供核心支持

#### 第 7 步：补 PenetrationQuery

目标：

- 支撑 depenetration、卡墙挤出、player-vs-player

---

### 八、测试矩阵

查询层不该只靠集成测试，必须先有小粒度无头测试。

#### A. 单点查询测试

- 查询空体素返回未命中
- 查询环境体素返回环境命中
- 查询实例体素返回实例命中
- `ignore_instance_idx` 后不命中自身

#### B. 射线测试

- 向下射线命中平面
- 忽略自身后不命中角色本体
- `raycastAll` 返回按距离排序
- `include_sensors=false` 时穿透 trigger

#### C. Sweep 测试

- `sphereCast` 卡在窄缝外
- `capsuleCast` 能提前命中墙
- `boxCast` 返回法线一致

#### D. Penetration 测试

- 初始重叠返回非零 depth
- 挤压方向稳定可预测
- 被 `ignore_instance_idx` 后重叠消失

#### E. 迁移回归测试

- `checkFall` 行为不回退
- `checkFlow` 行为不回退
- 车辆 grounded 行为不崩
- KCC grounded 行为稳定

---

### 九、与 physics.md 的直接映射

统一查询层完成后，能直接解锁或显著推进以下部分：

- `18` 初始穿透求解
- `30` 接触点/接触稳定性前置条件
- `51-70` 射线、扫掠、CCD 前端
- `69` ignore self
- `70` trigger policy
- `73-74` Kinematic 平台与载物探测基础
- `101-110` KCC 所有 grounded / slide / step / depenetration 的前置条件
- `141-150` 车辆悬挂射线
- `181-190` 回滚重演中的 hit verification
- `801-900` 人体胶囊体、肢体碰撞体、站立检测、挤压修复

---

### 十、完成标志

Task A 完成，不是指“有几个新函数”，而是满足以下标准：

- `kcc.zig` 不再通过 `undefined` 调世界查询
- `vehicle.zig` 不再通过 `undefined` 做 grounded 检测
- `physics.zig` 的运动/流动/下落逻辑已迁移到新 query API
- `vm_hook.zig` 已暴露统一的查询接口
- Python 测试可以直接断言 query 结果
- 射线、扫掠、重叠、穿透共享一套过滤规则

---

### 十一、建议拆分成的实际开发任务

可以直接拆成以下子任务：

1. 新建 `query_types.zig`，定义统一结果结构
2. 新建 `query_world.zig`，实现环境/实例单点占用查询
3. 为实例增加最小化 `body_type / is_trigger / layer` 元数据入口
4. 实现 `QueryFilter`
5. 把 `physics.checkFall/checkFlow/checkPush` 迁移到 query API
6. 修复 `kcc.zig` grounded/collision 查询
7. 修复 `vehicle.zig` grounded 查询
8. 实现 `raycastAll`
9. 实现 `capsuleCast`
10. 实现 `computePenetrationCapsule`
11. 在 `vm_hook.zig` 暴露统一查询 FFI
12. 在 `worldvm.py` 加统一查询封装
13. 为以上接口补 headless 回归测试

---

### 十二、如果只做最小可行版本

如果你想先快速落地一个 MVP 查询层，最小版本建议只做：

- `QueryWorldView`
- `QueryFilter`
- `queryVoxel`
- `raycastSingle`
- `raycastAll`
- `capsuleCast`
- `computePenetrationCapsule`

然后先改三处调用：

- `physics.zig`
- `kcc.zig`
- `vehicle.zig`

这样就能先把最危险的设计债清掉，并为后续 KCC / 载具 / 人体打地基。

## Task A 补充设计：查询层最终边界

### 核心判断

查询层不要做成“工具函数集合”，要做成“物理内核的只读门面”。

后续所有系统都应统一走：

- `PhysicsWorld`
- `QueryWorldView`
- `Query API`
- `Query result`

这样可以把三类职责彻底拆开：

- 世界怎么存储，是存储层问题
- 世界怎么查询，是 Query 层问题
- KCC / 载具 / 人体怎么消费查询结果，是控制器与求解器问题

当前仓库的主要结构性问题，就是这三层还混在一起。

---

### 一、建议的最终分层

#### 1. Storage Layer

这一层就是当前的：

- `Scene1024`
- `instances`
- `entities`
- `occupancy`

职责：

- 只存储世界数据
- 不承担运动学、载具、人体策略逻辑

#### 2. Query Layer

这一层负责：

- 单点体素查询
- overlap
- raycast
- cast / sweep
- penetration
- filter 评估

职责：

- 只读访问世界
- 统一命中语义
- 统一过滤规则
- 不推进世界状态

#### 3. Solver / Controller Layer

这一层才是：

- `TickEngine`
- `KCC`
- `Vehicle`
- `Ragdoll`
- `Ballistics`

职责：

- 调用 Query API
- 根据查询结果做运动/受力/状态转换

原则：

- 上层系统不能再自己拼世界查询细节

---

### 二、Query 层内部建议再拆四块

#### 1. World Access

职责：

- 负责最低层世界读取

典型函数：

- `sampleEnvironmentVoxel`
- `sampleInstanceVoxel`
- `sampleAnyVoxel`

回答的问题：

- 某个位置有没有东西

#### 2. Filter Evaluation

职责：

- 负责决定某个命中对象要不要参与本次查询

典型判断：

- 是否忽略 self
- 是否允许 static
- 是否允许 dynamic
- 是否允许 kinematic
- 是否允许 sensor
- layer / mask 是否通过

回答的问题：

- 这个命中算不算

#### 3. Primitive Query

职责：

- 执行具体几何查询

典型函数：

- `raycastSingle`
- `raycastAll`
- `overlapAABB`
- `overlapCapsule`
- `capsuleCast`
- `computePenetrationCapsule`

回答的问题：

- 按这个形状和过滤器，结果是什么

#### 4. High-Level Helpers

职责：

- 为上层系统提供轻量组合查询

典型 helper：

- `queryGroundBelowCapsule`
- `querySuspensionContact`
- `querySpawnClearance`

限制：

- 不能重新定义过滤逻辑
- 只能组合底层 Query API

---

### 三、MVP 阶段故意不要先做的事

为了让架构快速落地，以下内容不建议在 Query MVP 第一阶段硬上：

#### 1. 不先做强 broadphase

原因：

- 当前主要瓶颈是设计债，不是性能
- 应先把 API 语义、过滤规则、返回结构钉死

#### 2. 不先做完整 collision layer 体系

原因：

- 现在先把 `QueryFilter` 结构立好更重要
- 具体 layer/mask 规则可以在实例元数据补齐后再扩展

#### 3. 不先追求完美旋转 Box Cast

原因：

- 对当前阶段收益不如 `raycastAll + capsuleCast + penetration`
- KCC、车辆、人体当前更依赖胶囊与射线

#### 4. 不让 Query 返回过度复杂的数据

原因：

- 第一阶段重点是支撑控制器
- 不是构建完整接触流形系统

MVP 阶段优先保证：

- `hit`
- `distance / toi`
- `normal`
- `first blocker`
- `penetration depth`
- `pushout direction`

---

### 四、Query 与 KCC 的直接关系

真正的 KCC 不应再是：

- 先尝试位移
- 撞了就硬抬高几格

而应该是：

1. 用 `capsuleCast` 沿目标位移扫掠
2. 若未命中，直接移动
3. 若命中，判断是否可 `step`
4. 不可 `step` 时，按命中法线做 `slide`
5. 再做二次 sweep
6. 最后做 `ground snap`
7. 若仍有重叠，则跑 `computePenetrationCapsule`

因此 KCC 最依赖的 Query 核心只有三项：

- `capsuleCast`
- `queryGround`
- `computePenetrationCapsule`

结论：

- KCC 的前提不是先写状态机，而是先把“如何感知世界”做对

---

### 五、Query 与载具的直接关系

载具每个轮子的标准闭环，第一步就是查询。

建议标准链路：

1. 计算轮位世界坐标
2. 向下 `raycastSingle`
3. 得到接地点、法线、距离
4. 计算悬挂压缩量
5. 计算垂向载荷
6. 轮胎模型计算纵向/横向力
7. 汇总回车身

这里的关键不是 Pacejka 公式，而是第 2 步必须稳定。

结论：

- 轮胎接地判定必须和角色 grounded、人体落地、传感器遮挡共享一套 Query 规则
- 否则后续不同系统会对同一地面产生不同结论

---

### 六、Query 与人体系统的直接关系

人体系统本质上需要两类查询。

#### 1. 控制型查询

用途：

- 站立
- 跑跳
- 攀爬
- 挂边
- 滑铲
- 平衡修正

核心依赖：

- `capsuleCast`
- `ledge probe`
- `ground normal`
- `head clearance`
- `computePenetration`

#### 2. 被动物理查询

用途：

- 肢体碰撞
- 尸堆稳定
- 受击
- 挤压
- 骨骼/部位排斥

核心依赖：

- `overlap`
- `raycast`
- `penetration`
- `self-filter`

结论：

- 人体不是独立方向，它只是 Query 层的重度消费者
- 没有统一 Query，人体系统会比 KCC 和载具更快失控

---

### 七、建议收敛后的 Query MVP 范围

如果要把 Query MVP 进一步收敛，我建议固定为以下 6 项：

1. `QueryWorldView`
2. `QueryFilter`
3. `queryVoxel`
4. `raycastSingle / raycastAll`
5. `capsuleCast`
6. `computePenetrationCapsule`

并强制首批迁移三处：

1. `physics.zig`
2. `kcc.zig`
3. `vehicle.zig`

这样做的意义：

- 不是“多了几个查询接口”
- 而是把整个工程的架构方向扭正

---

### 八、运行时碰撞元数据放置建议

这里必须提前定边界，否则后面会重复返工。

#### 不建议直接塞进 `Entity16.physics`

原因：

- `Entity16` 更像原型定义
- 运行时碰撞行为更多是实例级语义

例如同一个原型后续可能生成：

- dynamic 箱子
- kinematic 门
- sensor 区域
- query-only proxy

这些不应该全部回写到原型层。

#### 建议新增实例级运行时元数据

建议形式：

- `InstancePhysicsMeta`
- 或并行 runtime metadata 数组

建议字段：

- `body_type`
- `collision_layer`
- `collision_mask`
- `is_trigger`
- `owner_id`
- `query_group`
- `ignore_group`

原型层保留：

- 质量
- 摩擦
- 恢复系数
- 材料
- 硬度

这样职责更清晰：

- 原型决定“是什么”
- 实例元数据决定“怎么参与查询和碰撞”

---

### 九、结果结构统一原则

后续要尽量避免：

- raycast 一套结果
- sweep 一套结果
- 车辆轮胎命中一套结果
- KCC 命中一套结果

建议原则：

- 全项目尽量统一使用同一套 `QueryHit` 思路

即使某些 API 不会填满所有字段，也尽量共享主结构。

原因：

- 减少 Query API 之间的语义漂移
- 减少 `vm_hook.zig` 的 FFI 复杂度
- 减少 Python 测试层的适配成本

---

### 十、补充后的结论

Task A 的意义不是修一个 bug，也不是补几个 FFI。

Task A 的真正意义是：

- 把世界存储、世界查询、物理控制器三层彻底拆开
- 为 KCC、载具、人体、回滚共用同一个查询底座
- 把工程从“模块堆叠”推进到“统一运行时架构”

如果后续继续细化方案，最值得深入讨论的下一个点是：

- `capsuleCast` 在当前体素世界里的 MVP 实现方案

候选方向：

- 离散采样
- swept AABB 近似
- 多球串联近似胶囊
