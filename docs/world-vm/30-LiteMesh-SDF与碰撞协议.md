# 04 LiteMesh、SDF 与碰撞协议

## 1. 目标
整理 `todo2.md` 中几何表示路线：LiteMesh + AABB 预检 + 位图/公式双轨。

## 2. LiteMesh 定义
LiteMesh 作为“逻辑几何体”而非渲染网格：
1. `TopologyRef`（16³ 位图索引或公式索引）
2. `Transform`（位置、朝向、尺度）
3. `LogicMask`（交互/材质/规则标记）
4. `AABB`（快速预检）

## 3. 三层表示法（由轻到重）
1. AABB/8顶点（最轻，预检和寻址）。
2. SDF/几何公式（中层，旋转和组合表达）。
3. 16³ 位图（最重，精细碰撞）。

## 4. 双层碰撞协议
1. **文斗层（Broad-phase）**：AABB 快速排除不相交对。
2. **武斗层（Narrow-phase）**：位图按位与或 SDF 采样精确判定。

## 5. Box 映射规范
`todo2.md` 重点强调 Box 的工程化：
1. 保留几何参数（origin/end 或 center/extents）。
2. 需要精细碰撞时临时光栅化到 16³ 位图。
3. 临时位图可复用缓存，用后可释放。

## 6. SDF 策略
1. 规则体（Box/Sphere/Cylinder/Capsule）优先使用 SDF 参数化存储。
2. 复杂体用“公式组合（并/差/交）+ 必要时局部位图缓存”。
3. 运行时根据场景需求选择采样精度。

## 7. 表示选择建议
1. 高频动态碰撞对象：建议缓存位图。
2. 规则静态对象：优先公式表示。
3. 仅路径预检对象：只用 AABB 即可。

## 8. 最小接口建议
1. `computeAabb(mesh)`
2. `intersectAabb(a,b)`
3. `sampleSdf(mesh, point)`
4. `rasterizeToVoxel(mesh, temp_buffer)`
5. `checkVoxelCollision(maskA, maskB)`

## 9. 风险与控制
1. 风险：实时光栅化导致抖动。  
控制：热对象位图缓存 + 分帧更新。
2. 风险：SDF 与位图结果不一致。  
控制：回归测试对齐同一对象的双轨判定。

## 10. 结论
LiteMesh 不是在替代 3D 图形引擎，而是在建立“逻辑计算友好”的几何表达层，  
让 1024³ 分页场景仍可保持可控计算成本。

## 11. Query 层边界（补充）
当前碰撞协议已有 Broad/Narrow 思想，但缺少统一查询边界。  
本节补充 Query Layer 作为“只读门面”的职责定义。

### 11.1 三层责任切分
1. `Storage Layer`：保存 `Scene/Instance/Entity/Occupancy`。
2. `Query Layer`：只读查询（voxel/overlap/raycast/sweep/penetration）。
3. `Solver Layer`：消费查询结果做移动、冲量、破坏与提交。

约束：
1. Query 不修改世界状态。
2. Solver 不直接访问底层碰撞细节实现。
3. Storage 不承担策略逻辑。

### 11.2 Query 世界视图
建议统一参数入口：
1. `QueryWorldView { scene, instances, entities }`
2. 所有查询函数只接收 world view + filter + shape/ray

收益：
1. 降低函数签名分裂。
2. 便于统一回放与测试。
3. 便于后续替换 Broadphase 实现。

## 12. 统一查询协议（Query Protocol）
建议最小协议集合如下：

### 12.1 单点查询
1. `queryVoxel(gx, gy, gz, filter)`
2. 返回是否命中、命中来源（环境/实例）与实例索引。

### 12.2 重叠查询
1. `overlapAabb(shape, filter)`
2. `overlapCapsule(shape, filter)`
3. 用于站立检测、出生点校验、触发器覆盖检测。

### 12.3 射线查询
1. `raycastSingle(ray, filter)`
2. `raycastAll(ray, filter, out_hits)`
3. 用于武器命中、悬挂采样、可见性检测。

### 12.4 扫掠查询
1. `sphereCast(sweep, filter)`
2. `capsuleCast(sweep, filter)`
3. `boxCast(sweep, filter)`
4. 用于 KCC、CCD 前置检测、动态障碍预判。

### 12.5 穿透查询
1. `computePenetration(shape, filter)`
2. 输出 `depth + direction + blocker`
3. 用于 depenetration 与卡墙挤出。

## 13. 过滤器规范（Filter Contract）
所有查询必须共享一套过滤语义，至少包含：
1. `ignore_instance_idx`
2. `include_static`
3. `include_dynamic`
4. `include_kinematic`
5. `include_sensor`
6. `ignore_environment`
7. `layer_mask`（预留）

### 13.1 过滤优先级
建议固定顺序：
1. 忽略 self / 忽略组
2. body type 过滤
3. layer mask
4. sensor 策略
5. 环境命中策略

固定顺序可保证不同平台行为一致。

## 14. Query 结果结构规范
建议统一 `QueryHit` 主结构，最小字段：
1. `hit`
2. `distance`
3. `toi`
4. `position`
5. `normal`
6. `instance_idx`
7. `entity_id`
8. `hit_environment`
9. `hit_sensor`

说明：
1. `raycast/sweep` 可以共用该结构。
2. `overlap/penetration` 可使用对应扩展结构，但字段语义保持一致。

### 14.1 接触分类扩展字段
若 Query 被用于物理接触前置分类，而不是纯几何检测，建议在 `QueryHit` 或扩展结果中补充：
1. `material_type`
2. `surface_condition`
3. `medium_type`
4. `body_type`

说明：
1. `material_type`：命中对象或命中地表的材质标签。
2. `surface_condition`：命中表面的状态，如 `DRY/WET/ICY/MUDDY/POLISHED`。
3. `medium_type`：该命中后应进入的介质路径，如 `MEDIUM_HARD_CONTACT/MEDIUM_WATER_DEEP`。
4. `body_type`：用于区分 `static/dynamic/kinematic/sensor`。

### 14.2 两类 Query 结果
建议文档中明确区分：
1. **几何 Query**：只回答“是否命中、命中哪里、法线是什么”。
2. **接触 Query**：在几何结果之上，补充材质/表面/介质分类信息。

适用建议：
1. `raycast` 做 AI 可见性时，可只用几何 Query。
2. `capsuleCast` 做 KCC、载具、刚体接触时，应使用接触 Query。
3. `computePenetration` 做 depenetration 时，可先只依赖几何 Query，再按需补分类。

### 14.3 文档级最小结构示例
```c
struct QueryHit {
    bool hit;
    float distance;
    float toi;
    vec3 position;
    vec3 normal;
    int instance_idx;
    int entity_id;
    bool hit_environment;
    bool hit_sensor;
    int material_type;
    int surface_condition;
    int medium_type;
    int body_type;
}
```

## 15. 与 LiteMesh/SDF 的协作方式
Query 层不绑定具体几何表达，统一通过“分层判定”协作：
1. 先 AABB（Broad）。
2. 后 SDF 或位图（Narrow）。
3. 再过滤器裁剪结果。
4. 最后写入标准 `QueryHit`。

### 15.1 缓存建议
1. 高频动态对象：优先位图缓存。
2. 规则静态对象：SDF 直接采样。
3. Query 层只读取缓存，不负责生命周期决策。

### 15.2 与材质/表面/介质的协作方式
几何表达层本身不定义物理响应，但必须能把“命中的是谁”交给分类层。

建议顺序：
1. LiteMesh/SDF/位图先给出几何命中。
2. Query 再根据命中的实例或环境单元查出：
3. `material_type`
4. `surface_condition`
5. `medium_type`
6. 最后交给 Solver 决定接触响应。

约束：
1. 几何表示层不直接编码摩擦、回弹、损伤逻辑。
2. 分类信息可挂在实例元数据或环境体素元数据上。
3. Query 负责搬运分类结果，不负责决定动力学参数。

## 16. 观测接口规范（与 Query 对齐）
建议在 Query 调用链中输出可追踪字段：
1. `query_type`
2. `shape_type`
3. `filter_hash`
4. `candidate_count`
5. `narrow_test_count`
6. `hit_count`
7. `first_hit_distance`
8. `cost_us`

这些字段用于：
1. 性能分析
2. 回归对比
3. 回放一致性验证

### 16.1 分类相关观测字段
若 Query 进入物理接触链，建议追加：
1. `resolved_material_type`
2. `resolved_surface_condition`
3. `resolved_medium_type`
4. `resolved_body_type`

用途：
1. 排查“为什么同样是水泥，湿地和冰面响应不同”。
2. 排查 KCC/Vehicle/Rigidbody 是否拿到了同一分类结果。
3. 为回放系统提供更强的可解释性。

## 17. 验收补充
新增协议验收建议：
1. 同一场景下 `raycastSingle` 与 `raycastAll[0]` 首命中一致。
2. `ignore_instance_idx` 在 KCC 与 Vehicle 两类调用中语义一致。
3. `capsuleCast + computePenetration` 能稳定生成 depenetration 方向。
4. 同输入、同过滤参数下，Query 结果可回放一致。

### 17.1 分类验收补充
1. 相同 `CONCRETE` 几何体在 `DRY/WET/ICY` 三种表面状态下，Query 返回的 `surface_condition` 必须不同。
2. `CONCRETE + ICY` 与 `ICE + DRY` 必须允许返回不同组合，避免把“冰块”和“结冰地面”混为一谈。
3. 同一水池应能区分 `MEDIUM_WATER_SHALLOW` 与 `MEDIUM_WATER_DEEP`，为后续浮沉路径提供依据。
4. KCC、Vehicle 和刚体模块对同一命中点读取到的 `material/surface/medium` 必须一致。

## 18. 本章补充结论
LiteMesh/SDF 解决“怎么表达几何体”，  
Query 协议解决“怎么稳定查询几何体”。  
当 Query 还能稳定返回材质、表面状态和介质分类时，  
两者合并后，才真正构成可复用的碰撞基础设施。
