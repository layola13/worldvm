# 09 Bagua ISA 指令集

## 1. 目标
定义 WVM 的最小可用指令集（6-bit，64 opcode），确保动作、关系、情绪调制可统一编码。

## 2. 指令设计原则
1. 定长编码，便于快速解码。
2. 高复用原语，复杂行为由组合生成。
3. 明确副作用范围，方便审计与回滚。

## 3. 指令编码建议
### 3.1 指令头
```c
typedef struct {
    uint8_t opcode;       // 0..63
    uint16_t actor_id;
    uint16_t target_id;
    int8_t magnitude;
    uint8_t duration;
    uint8_t flags;
} Instr;
```

### 3.2 flags（建议）
1. bit0: requires_focus
2. bit1: interruptible
3. bit2: safety_checked
4. bit3: writes_relation
5. bit4: writes_affect

## 4. Opcode 分段
1. `0x00-0x0F` 控制类
2. `0x10-0x1F` 空间/交互类
3. `0x20-0x2F` 物理类
4. `0x30-0x3F` 认知/沟通类

## 5. 推荐基础指令
### 5.1 控制类
1. `NOP`
2. `SLEEP`
3. `WAKE`
4. `JUMP`
5. `COMMIT`
6. `ABORT`

### 5.2 空间/交互类
1. `MOVE`
2. `ROTATE`
3. `APPROACH`
4. `MOVE_AWAY`
5. `BIND`
6. `UNBIND`
7. `GRAB`
8. `RELEASE`

### 5.3 物理类
1. `FALL`
2. `FLOW`
3. `HEAT`
4. `COOL`
5. `BREAK`
6. `MERGE`
7. `PUSH`
8. `PULL`

### 5.4 认知/沟通类
1. `FOCUS`
2. `AVOID`
3. `FREEZE`
4. `CONFIRM`
5. `DENY`
6. `ASK`
7. `REPAIR`
8. `EVALUATE`

## 6. 执行语义规范
每条指令必须定义：
1. 前置条件（precondition）
2. 状态读集合（read set）
3. 状态写集合（write set）
4. 失败码（error code）
5. 可回滚性（reversible or not）

## 7. 指令路由（Jump Table）
```c
typedef int (*OpFn)(Scene32*, const Instr*);
extern OpFn BAGUA_OPS[64];
```
1. 未实现 opcode 必须映射到 `op_unimplemented`。
2. 路由层不做业务判断，只负责分发。

## 8. 组合策略
复杂动作由模板组合，例如：
1. 倒水：`GRAB(cup)` -> `APPROACH(mouth)` -> `ROTATE(cup,45)` -> `RELEASE`
2. 打招呼：`FOCUS(target)` -> `WAVE_HAND` -> `ASK`
3. 回避威胁：`FOCUS(threat)` -> `MOVE_AWAY` -> `FREEZE`（必要时）

## 9. 幂等与重放
1. 指令应尽量设计为幂等或可检测重复执行。
2. 每条指令携带执行序号，支持重放调试。
3. 不可幂等指令必须记录补偿逻辑。

## 10. 安全守卫
执行前必须经过策略守卫：
1. 权限检查
2. 对象合法性检查
3. 资源预算检查
4. 禁止规则检查

## 11. 错误码建议
1. `E_OK`
2. `E_PRECONDITION`
3. `E_COLLISION`
4. `E_BUDGET`
5. `E_POLICY`
6. `E_TIMEOUT`

## 12. 指令级可解释性
每次执行记录：
1. 指令 ID
2. 输入参数
3. 关键状态前后值
4. 成功/失败与原因码

## 13. 版本化
1. opcode 编号一旦发布尽量不改。
2. 新能力优先占用保留位，不重定义旧语义。
3. 提供 opcode 映射表用于跨版本兼容。

## 14. 本章结论
Bagua ISA 是 WVM 的执行语义中心。  
只要指令语义稳定，计划、知识、心智三层都能在同一执行模型上复用。

