# 31 Zig 内核实现策略

## 1. 目标
将 `todo2.md` 的 Zig 讨论收敛为可执行工程策略：小体积、零依赖、快编译、快运行。

## 2. 语言与约束
1. 语言：`Zig 0.14.1`。
2. 运行：Win x64 首版。
3. 依赖：默认不引入第三方运行时。
4. 内存：固定预算管理，优先 `FixedBufferAllocator`。

## 3. 模块划分
1. `vm_core`：分页、页表、Tick。
2. `vm_geom`：LiteMesh、AABB、位图转换、SDF采样。
3. `vm_phys`：刚体/流体/气体离散算子。
4. `vm_human`：Personal Universe 运行时。
5. `vm_sound`：声音子宇宙（事件缓冲、声音特征、联动输出）。
6. `vm_chem`：化学子宇宙（标签场、反应规则、扩散步进）。
7. `vm_io`：CLI、ASCII dump、日志。
8. `vm_ext`：外挂接口占位。

## 4. 性能策略
1. 整数优先：空间坐标与碰撞逻辑优先整数路径。
2. SIMD：位图碰撞可用 Zig 向量能力做批量按位操作。
3. 热路径零分配：Tick 内避免临时堆分配。
4. 构建时预计算：常用 LUT 用 `comptime` 预烘焙。

## 5. 构建策略
1. `Debug`：开发与断言。
2. `ReleaseSafe`：验证版本。
3. `ReleaseSmall`：发布小体积版本。

## 6. CLI 建议
1. `run --scenario ... --ticks N`
2. `bench --scenario ...`
3. `dump --view top|front|side|slice`
4. `sim-check --domain physics|chem --case ...`

## 7. 物理算子最小集（v1）
1. `APPLY_FORCE_FALL`
2. `FLOW_STEP`
3. `AABB_CHECK`
4. `VOXEL_AND_CHECK`
5. `STATE_MUTATE`

## 8. 可选生态参考（非强依赖）
1. `zmath`：仅借 SIMD 数学表达。
2. `zphysics`：只作为算法参考，不直接整包接入。

## 9. 工程警戒线
1. 不引入重量级连续物理求解器。
2. 不让内核依赖 Python 运行时。
3. 不在首版耦合训练流程代码。
4. 人宇宙与子宇宙模块必须支持 headless 运行。

## 10. 验收目标
1. 单可执行交付。
2. 关键场景可运行可回放。
3. 调试输出可定位碰撞与分页行为。
4. 具备后续外挂扩展接口。
