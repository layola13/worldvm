# 07 Hook 与工具调用驱动

## 1. 目标
定义 `todo2.md` 中两种外挂接入模式：
1. Tool Call（显式调用）
2. Hook（隐式拦截）

## 2. Tool Call 模式
### 2.1 特点
1. 明确可见、可调试。
2. 易于与 Python/LangChain/Ollama 集成。
3. 对模型侵入性低。

### 2.2 缺点
1. 依赖模型“愿意调用”。
2. 无法覆盖所有潜在错误生成路径。

## 3. Hook 模式
### 3.1 特点
1. 在推理路径中做强制物理审计。
2. 可对高风险语义即时拦截。
3. 能实现“默认安全”。

### 3.2 难点
1. 如何在合适粒度触发拦截（token/句子/段落）。
2. 如何把物理错误回灌成可用纠错信号。
3. 如何避免过度拦截导致输出僵硬。

## 4. 推荐演进路线
1. v1：先 Tool Call（稳定、低风险）。
2. v1.1：增加“高风险语义 Hook”。
3. v2：形成混合模式（默认 Tool，关键路径 Hook）。

## 5. Hook 触发建议
触发关键词/语义类型：
1. 运动与放置（move/put/drop）。
2. 交通与路径（go/turn/stop/light）。
3. 容器与流体（fill/pour/overflow）。

## 6. Hook 返回协议
至少包含：
1. `status`（PASS/FAIL）
2. `reason_code`
3. `break_frame`
4. `repair_hint`

## 7. 回灌策略
1. FAIL 时追加约束上下文并请求模型重试。
2. 对高置信错误可直接阻断危险输出。
3. 保存拦截日志用于后续训练与规则优化。

## 8. 接口封装建议
Zig 内核对外统一 C ABI：
1. `init_kernel`
2. `run_logic_check`
3. `get_trace_summary`
4. `reset_context`

上层可由 Python/Go/Node 封装 SDK。

## 9. 性能注意
1. Hook 调用必须轻量化，避免每 token 重负载模拟。
2. 优先做事件级或句级批量检查。
3. 大数据结构不跨语言频繁拷贝，只传指令与摘要。

## 10. 结论
Tool Call 解决“可用”，Hook 解决“可靠”。  
先把 Tool Call 跑通，再按风险点逐步引入 Hook。
