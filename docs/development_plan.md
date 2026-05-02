# WorldVM 工程评估与开发计划

Last updated: 2026-05-02

## 目标

- 将 WorldVM 收敛为一个可构建、可测试、可通过 CLI/Python FFI 使用、可发布的最小可用运行时。
- 优先稳定当前真实代码路径：Zig 内核、场景实例生命周期、Python `ctypes` 包装、CLI 场景、发布包。
- 在验证闭环稳定前，不扩大愿景范围，不把设计文档里的长期设想误认为已实现能力。

## 当前工程评估

- **可用性**：当前工程已具备 headless simulation 的基础可用面，可以本地构建 CLI/shared library，通过 Python wrapper 调用实例生命周期与 tick 行为。
- **工程闭环**：已有 fast/full Zig test gates、Python acceptance smoke、Entity16 ABI fixture 校验、release package 生成与验证脚本。
- **发布能力**：已有 Linux/macOS/Windows artifact 选择逻辑、checksum sidecar、SPDX 2.3 package SBOM、GitHub Release workflow 与 release notes gate。
- **主要风险**：ABI layout 漂移、实例 index 在 compact 后失效、physics 调度路径退化、release artifact 与 manifest/SBOM 不一致。
- **当前边界**：Entity16 已有 ABI v1 fixture，但还没有独立持久化实体文件格式；SBOM 当前覆盖打包文件级 checksum，不声明外部依赖图；benchmark baseline 仍需要 hosted runner 样本收敛。

## 现在可用的能力

- `zig build` 构建 CLI 与 shared library。
- `zig build run -- run --scenario apple_table --ticks 3` 运行内置场景。
- `worldvm.py` 通过 `ctypes` 调用 shared library，覆盖 spawn、tick、instance lifecycle、stable handle 查询与删除。
- `worldvm.py` 支持默认 `zig-out` 路径、`WORLDVM_LIBRARY_PATH` 和 `WorldVM(library_path=...)` 三种 shared-library 加载方式。
- `examples/python_lifecycle.py` 提供可执行的 Python FFI 生命周期示例，并支持 `--library` 指定 shared library。
- `Scene1024` 支持 remove、mark broken、compact，并提供可跨 compact 检查的稳定 instance handle。
- `Entity16` 保持 4096B 布局，并通过 `docs/entity16_abi_v1.json` 与 `tests/fixtures/entity16_abi_v1_default.bin` 固定 ABI v1。
- `docs/ffi_symbols_v1.json` 固定 `src/vm_hook.zig` 当前 public FFI 导出符号和签名。
- `tools/package_release.py` 可以生成本地 release archive，`tools/verify_release_package.py` 可以校验 archive checksum、顶层目录元数据、精确文件清单、manifest、SBOM 元数据/checksum 与必备 payload 文件。

## 验证门槛

### 每次功能变更

```bash
zig build test-fast
python3 tools/verify_entity16_abi.py
python3 tools/verify_ffi_manifest.py
python3 tools/verify_python_wrapper_api.py
python3 tools/verify_readme_snippets.py README.md
python3 -m py_compile worldvm.py
python3 -m unittest tests.test_package_release
python3 -m unittest tests.test_release_verifier
python3 -m unittest tests.test_smoke_release_package
python3 -m unittest tests.test_worldvm_wrapper
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.physics.test_acceptance_scenario
```

### 涉及 physics、tick、scene lifecycle

```bash
zig build check-matrix
zig build test-full
python3 tools/benchmark_scenarios.py --scenario apple_table --ticks 3 --runs 1 --skip-build --baseline benchmarks/ci_benchmark_baseline.json
```

### 涉及发布、打包、CI

```bash
python3 tools/package_release.py --version smoke --skip-build
python3 tools/verify_release_package.py dist/worldvm-smoke-*.tar.gz
python3 tools/smoke_release_package.py dist/worldvm-smoke-*.tar.gz
python3 tools/extract_release_notes.py --version 0.1.0 --require-bullet --reject-placeholder
```

## 开发路线图

### P0：最小可用基线，已完成

- 固定 `Entity16` 扩展块与 ABI version，保持 4KB entity layout。
- 补齐 scene instance lifecycle：remove、mark broken、compact。
- 对外提供稳定 instance handle，避免 compact 后直接暴露 index 导致误删。
- 打通 FFI 与 Python wrapper 的 lifecycle/handle API。
- 固定 FFI symbol manifest，降低导出函数误删或改签名风险。
- 校验 Python wrapper 引用的 FFI 符号都存在于 manifest，降低 wrapper/API 漂移风险。
- 修复 tick engine 到 continuous physics 的调度路径。
- 恢复 crash-defense 对 position/velocity 的安全 clamp。
- 建立 CI、release package、checksum、SBOM、release notes、ABI fixture 与 benchmark smoke。
- 发布包验证覆盖必备 payload，避免 archive 缺少 wrapper、示例、ABI/FFI manifest 或平台二进制。
- 发布包验证拒绝 manifest 之外的额外文件，减少脏文件或意外文件进入 release archive。
- 发布包验证要求 archive 顶层目录与 manifest 的 `name`、`version`、`target` 一致。
- 发布包验证要求 SBOM 的 package name、version、namespace 和创建时间与 manifest 一致。
- 发布包目标标签规范化 `AMD64`/`x64` 为 `x86_64`，`aarch64` 为 `arm64`，避免 workflow artifact suffix 与 archive name 漂移。
- 发布包包含 README 中引用的 validation/release tools 和 smoke tests，避免解包后文档命令缺文件。
- 发布包 smoke 会解包 archive 并运行包内 CLI 与 Python lifecycle example，确认二进制、wrapper 和包内工具导入路径在解包环境可用。

### P1：工程硬化，下一阶段优先

- 在真实 hosted CI runner 上收集 benchmark 样本，并据此收紧 `benchmarks/ci_benchmark_baseline.json`。
- 对 Linux/macOS/Windows release artifact 做一次完整 dry run，确认 package script 与 workflow matrix 一致。
- 继续扩展 Python wrapper public API examples，覆盖 error return 和更多查询接口。
- 只在引入外部依赖后扩展 SBOM dependency metadata；当前不伪造依赖图。
- 继续明确 CLI、FFI、Python wrapper 的 SemVer 兼容性边界，避免内部 Zig refactor 误触 public contract。

### P2：产品化能力

- 如果引入持久化 entity/world 文件格式，新增 Entity16 read/write round-trip tests，并把格式版本纳入 release gate。
- 增加 scenario catalog 与每个 scenario 的预期 invariants，减少 benchmark 只测“能跑”的问题。
- 为 release archive 增加安装/集成说明，例如 Python-only consumer 如何定位 shared library。
- 建立 public FFI symbol manifest，避免导出函数被无意删除或改签名。

### P3：模拟深度

- 深化 material/medium physics，补齐液体、气体、摩擦、弹性、堆叠等场景的可解释验收指标。
- 将 chemistry、semantics、affect、behavior 扩展块从布局保留推进到真实读写与行为集成。
- 设计稳定 external API policy，区分实验性 VM hook、内部 Zig API 与长期支持 API。
- 在性能预算明确后，再做更激进的数据布局、分页、SIMD 或并行调度优化。

## 变更完成定义

- 修改 ABI/layout 时必须更新 ABI manifest、fixture 或校验脚本，并解释兼容性影响。
- 修改 FFI/Python wrapper 时必须有 Python smoke 或 acceptance test 覆盖真实 shared library。
- 修改 `worldvm.py` 的 `self.lib.*` 引用时必须跑 `tools/verify_python_wrapper_api.py`。
- 修改 README Python heredoc 示例时必须跑 `tools/verify_readme_snippets.py README.md`。
- 修改 Python wrapper 纯加载逻辑时必须跑 `tests.test_worldvm_wrapper`。
- 修改 physics/tick/scene lifecycle 时必须至少跑 fast gate，并在影响较大时跑 full gate。
- 修改 release workflow/package script 时必须生成并验证本地 archive。
- 修改 release package payload 或 wrapper/library 路径时必须跑 `tools/smoke_release_package.py`。
- 修改 release smoke 解包或平台 CLI 路径逻辑时必须跑 `tests.test_smoke_release_package`。
- 修改 package target/platform 逻辑时必须跑 `tests.test_package_release`。
- 修改 release package verifier 必备文件或 inventory 逻辑时必须跑 `tests.test_release_verifier`。
- 新增能力必须同步 README 或相关 docs，避免“代码可用但入口不可发现”。
