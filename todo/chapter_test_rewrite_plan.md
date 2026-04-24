# 章节测试重写执行计划

## 摘要

- 本轮只重写 Zig 章节测试，范围限定为 `src/chapter*.zig`。
- 保留现有 30 个 chapter 文件、章节编号、测试 ID 和大体测试名。
- 统一把章节测试改成三种类型：`scene_assert`、`module_contract`、`explicit_skip`。
- 一次性完成整套结构切换：先抽公共 harness，再把 30 个章节全部迁移到新结构；高级章节允许显式 `skip`，但不允许伪装成通过。

## 接口与结构变更

- 新增 `src/chapter_test_support.zig` 作为公共测试支持模块。
- 在该模块中定义统一测试接口：`ChapterCase`、`ChapterCaseKind`、`SceneSnapshot`、`CaseExpectation`。
- 公共模块统一提供 `runSceneCase`、`assertMoved`、`assertStopped`、`assertVelocityChanged`、`assertStateEq`、`assertDistanceApprox`、`assertTOIInRange`、`assertRayHit`、`skipUnsupported` 等 helper。
- 所有 `src/chapter*.zig` 都改成薄壳：只保留 case 定义、真实模块导入、每个 test 块对公共 harness 的调用；删除每章本地的 `runChapterTest`、重复 `makeInstance` 和重复结果拼装逻辑。

## 章节重写矩阵

- `01-04` 改为真实场景断言章节。
- `05` 和 `19` 改为关节契约章节，直接调用 `joint.zig`。
- `06` 和 `20` 改为 CCD/TOI 契约章节，直接调用 `ccd.zig`。
- `07` 改为基础 raycast 章节，`18` 改为高级 query 章节。
- `08` 只测试当前 `tick_engine` 已支持的 kinematic/dynamic 行为。
- `09` 改为极端输入健壮性章节。
- `10` 和 `21` 改为当前液体/介质能力章节。
- `11` 改为 KCC 契约章节，直接导入 `kcc.zig`。
- `12-17`、`24-25` 改为各模块契约章节。
- `22` 整章改为 `explicit_skip`，原因是缺少 `particles` 生产模块。
- `23` 整章改为 `explicit_skip`，原因是缺少 softbody/cloth 求解器。
- `26-30` 改为组合、规模、决定论、实时、集成章节，但只组合已被前面章节证明真实存在的能力。

## 实施顺序

1. 落公共 harness，先让 chapter 文件可以用统一的 `ChapterCase` 和 `SceneSnapshot` 表达三类 case。
2. 先迁移 `01-04`，验证 scene harness、快照采集、基础断言 helper 全部可用。
3. 迁移 `05-25`，对已有生产模块的章节改成 direct contract test；对缺失模块或缺失接线的章节立即改成 `explicit_skip`。
4. 迁移 `26-30`，把组合、规模、决定论、实时、集成章节全部建立在前面已落地的真实能力之上。
5. 收尾清理，确保 30 个 chapter 文件都不再各自定义 `runChapterTest`，并删除所有 `ticks_to_stable > 0` 式弱断言。

## 测试与验收标准

- `zig build test` 必须继续覆盖全部 30 个 chapter 文件，且构建入口不变。
- 所有 `src/chapter*.zig` 中不允许再出现 `ticks_to_stable > 0`、`passed = engine.stable or ticks < max_ticks` 这种弱通过逻辑。
- 所有章节测试都必须满足其一：有真实场景断言；有真实模块契约断言；有带固定 reason 的显式 `skip`。
- 每个 `scene_assert` case 至少包含一个与章节主题直接相关的数值或状态断言。
- 每个 `module_contract` case 必须显式导入并直接调用对应生产模块。
- 每个 `explicit_skip` case 的 reason 必须稳定、具体、可搜索。

## 默认假设

- 这轮不修改 Python 测试、不修改文档、不调整 `build.zig` 的 chapter 列表顺序。
- 现有章节 ID、文件名、章节编号和大体测试名全部保留。
- 对高级章节，`skip` 是允许且推荐的真实状态表达。
- `22 particles` 和 `23 softbody` 视为整章缺模块；`27 scalability` 和 `29 realtime` 不把硬性能门槛放进普通 `zig test`。

