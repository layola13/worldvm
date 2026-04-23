# 19 实现建议：Rust 与 C

## 1. 目标
给出工程落地建议，帮助快速搭建可运行原型并逐步演进到稳定版本。

## 2. 语言选择建议
### 2.1 C 路线
优点：
1. 二进制可控、运行时极简。
2. 与底层位运算和内存布局天然匹配。
3. 依赖链短，便于嵌入式环境。

风险：
1. 内存安全需要严格规范。
2. 多线程并发易出错。

### 2.2 Rust 路线
优点：
1. 内存安全与并发安全更好。
2. 枚举和模式匹配适合规则引擎。
3. 工具链与测试生态完善。

风险：
1. 首次实现复杂内存布局需要更多设计。
2. 对性能热路径要仔细控制抽象成本。

## 3. 推荐策略
1. 用 Rust 写大多数模块（Parser、Scheduler、Mind、Store）。
2. 用 C 或 Rust `unsafe` 写极热路径（位图运算、碰撞核）。
3. 通过 FFI 或模块边界组合。

## 4. 模块目录建议
```text
/wvm
  /core        // scene, tick, opcode
  /geom        // voxel, transform, collision
  /lod         // focus/periphery scheduler
  /parser      // lex, parse, ir
  /planner     // dag, lane scheduler
  /mind        // shadow sandbox, affect gate
  /kb          // rom builder, loader
  /store       // memory graph, external adapter
  /observe     // trace, ascii render
  /tests       // matrix tests
```

## 5. 数据结构实现要点
1. 先固化 ABI（实体和场景结构）。
2. 所有字段写单元测试验证偏移。
3. 位图操作封装成独立库并压测。

## 6. 并发模型建议
1. 事件级并发（多 Scene 并行）。
2. Scene 内执行保持确定性顺序提交。
3. 共享状态只在 Commit 阶段访问。

## 7. 日志与观测
1. 日志格式结构化（JSONL 或二进制 trace）。
2. 提供 debug/release 不同级别输出。
3. 每次执行保留 `event_id` 贯穿全链路。

## 8. 构建与发布
1. `debug`：开启断言和详细 trace。
2. `release`：关闭重日志、开启 LTO（可选）。
3. 版本号与 ROM 版本联动管理。

## 9. CI/CD 建议
1. PR：跑格式、lint、单测。
2. main：跑集成回归与性能 smoke test。
3. release：跑完整压力与稳定性测试。

## 10. 最小原型顺序
1. Entity/Scene 结构 + 位图运算
2. Tick 四步流水线
3. 最小 8 个 opcode
4. 简化 Parser -> IR
5. Plan DAG + lane 调度
6. LOD + 影子沙盒

## 11. 常见工程坑
1. 过早优化导致架构僵化。
2. 缺乏可解释日志导致难调试。
3. 规则散落在代码中难维护。

建议：规则配置化、行为可回放、接口先稳定后扩展。

## 12. 本章结论
Rust/C 都可行，关键在于“数据结构先行 + 执行可观测 + 热路径可控”。  
先做最小闭环，再扩展能力面。

