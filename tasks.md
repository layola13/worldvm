# World VM 当前任务

## 当前主线

当前优先级最高的工作不是继续扩写章节测试，而是重写 `src/chapter*.zig`，把“场景占坑”改成真正能约束实现的测试体系。

参考执行计划：

- `todo/chapter_test_rewrite_plan.md`
- `todo/chapter_tests_improvement_guide.md`

## 本轮目标

1. 抽取统一章节测试基座 `src/chapter_test_support.zig`
2. 移除每章重复的 `runChapterTest` / `makeInstance` / 弱断言模板
3. 将章节测试分成三类：
   - `scene_assert`
   - `module_contract`
   - `explicit_skip`
4. 先把基础章节改成真实断言
5. 对未落地模块的高级章节显式 `skip`，不再伪装为通过

## 章节重写策略

### 第一批：先落真实场景断言

- chapter01_motion_gravity
- chapter02_collision_detection
- chapter03_friction_elasticity
- chapter04_stacking_sleep

要求：

- 至少断言位置、速度、状态中的一种真实物理结果
- 禁止只写 `ticks_to_stable > 0`

### 第二批：改成模块契约测试

- chapter05 / chapter19 -> `joint.zig`
- chapter06 / chapter20 -> `ccd.zig`
- chapter07 / chapter18 -> raycast / query
- chapter11 -> `kcc.zig`
- chapter12 -> `ballistics.zig`
- chapter13 -> `destruction.zig`
- chapter14 -> `ragdoll.zig`
- chapter15 -> `vehicle.zig`
- chapter16 -> `network.zig`
- chapter17 -> `crash_defense.zig`
- chapter24 -> `terrain.zig`
- chapter25 -> force/explosion capabilities already in `tick_engine.zig`

### 第三批：显式跳过

- chapter22_particles
- chapter23_softbody

要求：

- 使用稳定、明确的 skip reason
- 说明缺失的是模块还是集成点

### 第四批：组合与验收章节

- chapter26_composite
- chapter27_scalability
- chapter28_determinism
- chapter29_realtime
- chapter30_integration

要求：

- 只能组合前面已被证明存在的能力
- 不引入新的假机制

## 验收条件

1. `zig build test` 继续覆盖全部 30 个章节文件
2. `src/chapter*.zig` 中清除弱断言：
   - `ticks_to_stable > 0`
   - `passed = engine.stable or ticks < max_ticks`
3. 所有章节测试必须属于以下之一：
   - 真实场景断言
   - 真实模块契约断言
   - 显式跳过

## 本轮暂不处理

- Python 测试重写
- `docs/physics.md` 口径更新
- `src/physics_tests.zig` 大范围重构
- 性能专项优化
