# Codex auto-continue hook

这是一个最小版 `Stop` hook。

作用：

- 每次 Codex 一轮回答结束后，hook 返回 `{"decision": "block", "reason": "contiune"}`。
- Codex 会阻止这次 stop，并把 `reason` 当成新的 continuation prompt 继续跑下一轮。

文件：

- `continue_on_stop.py`：Python hook 脚本
- `hooks.json`：示例 hook 配置

## 用法

1. 确保 `~/.codex/config.toml` 或项目 `.codex/config.toml` 开了功能：

```toml
[features]
codex_hooks = true
```

2. 把 `codex_hook/hooks.json` 的内容放到下面任一位置：

- `~/.codex/hooks.json`
- `<repo>/.codex/hooks.json`

3. 保证 `command` 路径能找到这个脚本。

如果你在仓库根目录启动 Codex，这个示例命令可以直接用：

```json
"command": "python ./codex_hook/continue_on_stop.py"
```

如果你想改自动续跑的提示词，可以设置环境变量：

```powershell
$env:CODEX_AUTO_CONTINUE_TEXT = "continue"
```

## 注意

- 这个版本是“最简单实现”，默认会一直续跑，可能导致长时间循环。
- `contiune` 按你的原话保留了。如果你其实想要正确拼写，改成 `continue` 就行。
