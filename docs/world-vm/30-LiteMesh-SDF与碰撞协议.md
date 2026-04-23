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
