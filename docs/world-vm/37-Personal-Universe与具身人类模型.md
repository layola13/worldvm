# 37 Personal Universe 与具身人类模型

## 1. 目标
把“人”建模为独立嵌套宇宙（Personal Universe），使感知、生理、情绪、记忆和行为在统一运行时内闭环。

## 2. 设计定位
1. 大世界不直接操作人体内部细节，只通过轻量指针挂载人宇宙实例。
2. 人宇宙遵循与主 VM 一致的原则：分页、事件驱动、可回放、可审计。
3. 人宇宙输出“可观察摘要”，不泄露全部内部状态。

## 3. 核心结构
1. `HumanRootPage`：人宇宙根页，维护生命体征与调度状态。
2. `AffectPage`：情绪寄存器与门控状态。
3. `MemoryPage`：事件记忆锚点与检索索引。
4. `BodyStatePage`：简化神经/血液/肌肉/消化状态。
5. `SubUniversePorts`：挂载声音宇宙和化学宇宙的输入输出端口。

## 4. 事件驱动模型
1. 外部事件以结构化包注入：`visual/audio/chemical/social/task`。
2. 人宇宙按事件类型激活最小子页集合。
3. 产出三类结果：
   1. `affect_delta`
   2. `behavior_bias`
   3. `physio_delta`

## 5. 与影子沙盒关系
1. 人宇宙可调用影子沙盒做“先想后做”预演。
2. 高强度事件必须走多分支预演短窗。
3. 预演结果只影响门控，不直接改外部世界状态。

## 6. 具身联动边界
1. 与物理宇宙：通过动作意图和能量约束交互。
2. 与声音宇宙：通过 `audio_input_port` 接收听觉结果。
3. 与化学宇宙：通过 `chemical_input_port` 接收嗅味觉/代谢结果。
4. 与语言层：输出表达风格权重与澄清偏好。

## 7. 内存与性能建议
1. 单人宇宙推荐预算：`64KB ~ 256KB`（按设备档位可调）。
2. 空闲期低频心跳，事件触发时短窗升频。
3. 所有跨宇宙消息传摘要，避免大结构拷贝。

## 8. 最小接口
```c
init_personal_universe(entity_id, cfg)
inject_personal_event(entity_id, event)
step_personal_universe(entity_id, tick_budget)
read_personal_outputs(entity_id, out_summary)
```

## 9. 观测与验收
1. 同输入下输出稳定可回放。
2. 情绪门控对动作白名单有可观测影响。
3. 高强度事件后存在可测恢复曲线。
4. 输出摘要可解释且不越权泄露内部页数据。

## 10. 本章结论
Personal Universe 不是附属插件，而是“人类具身行为”的正式运行时边界。  
它让 WVM 在保持可解释性的前提下，具备更真实的人体与情绪耦合能力。
