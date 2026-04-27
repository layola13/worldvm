# Codex AnyRouter Proxy (Deno + TypeScript)

这个工程用于把 Codex 的 `/responses` 请求转发到 AnyRouter，并兼容 AnyRouter 不支持 MCP 工具的场景。

## 关键修复

你的错误：

```json
{"error":{"message":"Missing required parameter: 'tools[15].tools'"}}
```

通常是因为请求里存在 `type: "namespace"`（或 `type: "mcp"`）工具对象，而上游网关期望不同字段结构。

本代理默认会在转发前过滤掉以下工具：

- `type: "namespace"`
- `type: "mcp"`

并保留 `function/custom/web_search` 等常见兼容工具类型。

## 运行

1. 复制环境变量模板：

```bash
cp .env.example .env
```

2. 设置你的上游配置（示例）：

```bash
export LISTEN_HOST=127.0.0.1
export LISTEN_PORT=19091
export UPSTREAM_BASE_URL=https://anyrouter.top/v1
export UPSTREAM_API_KEY='你的 anyrouter key'
export STRIP_MCP_TOOLS=1
```

3. 启动：

```bash
deno task start
```

## Codex 配置示例

在 `~/.codex/config.toml` 增加一个 provider，指向本地代理：

```toml
model_provider = "anyrouter_proxy"
model = "gpt-5.3-codex"

[model_providers.anyrouter_proxy]
name = "anyrouter_proxy"
wire_api = "responses"
requires_openai_auth = false
base_url = "http://127.0.0.1:19091/v1"
```

调用时可直接：

```bash
codex exec "hello"
```

## 验收标准

- 执行 `codex exec "hello"`
- 收到模型回复（非参数错误）

