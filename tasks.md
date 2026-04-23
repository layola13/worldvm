# World VM 任务清单

## 完成状态

| 模块 | 状态 | 文件 |
|------|------|------|
| 项目骨架+CLI | done | build.zig, main.zig |
| Entity16 | done | entity16.zig |
| Scene32 | done | scene32.zig |
| 物理算子 | done | physics.zig |
| Tick引擎 | done | tick_engine.zig |
| 地址编码 | done | address.zig |
| 1024分页 | done | scene1024.zig |
| 渲染 | done | renderer.zig |
| 场景 | done | scenarios.zig |
| 单元测试 | done | physics_test.zig |
| 三世界分页 | todo | - |
| 心智系统 | todo | - |
| Hook接口 | todo | - |

## Phase 1: MVP完善

### 已完成
- FORCE_FALL, FLOW_STEP, FORCE_PUSH, BREAK
- AABB碰撞检测
- 10个单元测试通过
- water_flow场景正常运行

### 待完成
1. 性能优化 (目标: Tick p95 < 200us)
2. hammer_glass场景验证
3. ReleaseSmall构建验证 < 5MB

## Phase 2: 1024分页

### 已实现
- address.zig: 64位地址编码
- scene1024.zig: 虚拟分页, LRU替换
- scene1024_test.zig: 单元测试

### 地址设计
- 64-bit: [world:4][reserved:15][page:15][local:15]
- page_x/y/z: 0-31 (32^3宏页)
- local_x/y/z: 0-31 (页内体素)

## Phase 3: 三世界分页

| 世界 | 内容 |
|------|------|
| 物理世界 | 拓扑/碰撞/重力/流体 |
| 心理世界 | 情绪/意图/权重 |
| 编程世界 | 规则/算子/元数据 |

## Phase 4: 心智系统

### 情绪寄存器
- valence: -128..127
- arousal: 0..255
- certainty: 0..255
- control: 0..255

### 影子沙盒
- 候选方案预演
- 情绪影响调度

## Phase 5: Hook接口

### 协议
- status: PASS/FAIL
- reason_code: u8
- break_frame: u32
- repair_hint: string

## 项目结构

worldvm/
  build.zig
  src/
    main.zig           - CLI入口
    entity16.zig       - 16^3实体
    scene32.zig        - 32^3场景
    scene1024.zig      - 1024^3分页
    address.zig        - 地址编码
    physics.zig        - 物理算子
    tick_engine.zig    - Tick引擎
    renderer.zig       - ASCII渲染
    scenarios.zig      - 场景
    physics_test.zig   - 测试
    scene1024_test.zig - 分页测试
  docs/world-vm/       - 设计文档
  tasks.md             - 本清单

## 运行命令

```bash
zig build && ./zig-out/bin/worldvm run -s apple_table -t 50
zig test src/physics_test.zig
zig test src/scene1024_test.zig
```
