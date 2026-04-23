# World VM 实现任务清单

## 当前完成状态

| 模块 | 状态 | 代码 |
|------|------|------|
| 项目骨架+CLI | ✅ | build.zig, main.zig |
| Entity16 | ✅ | entity16.zig |
| Scene32 | ✅ | scene32.zig |
| Tick引擎 | ✅ | tick_engine.zig |
| 物理算子 | ✅ | physics.zig |
| 渲染 | ✅ | renderer.zig |
| 场景 | ✅ | scenarios.zig |
| 单元测试 | ✅ | physics_test.zig |
| FLOW_STEP | ✅ | physics.zig |
| ReleaseSmall | ✅ | build.zig |

---

## 🔴 紧急修复：FLOW_STEP 流体物理

### 问题
water_flow 场景只运行 1 tick 就停止

### 原因
checkFall/checkFlow 检查目标位置时，没有排除实体自身当前位置

### 修复方案
1. physics.zig 添加 isOccupiedByOther() 函数
2. checkFall() 使用 isOccupiedByOther
3. checkFlow() 使用 isOccupiedByOther

### 验证
zig build && ./zig-out/bin/worldvm run --scenario water_flow --ticks 50

期望：水从 y=20 持续下落到地面，运行 > 20 ticks

---

## 🎯 Phase 1: MVP 完善

### 1.1 物理算子
- [x] FORCE_FALL - 重力下落
- [x] FORCE_PUSH - 推力移动
- [x] BREAK - 脆性破碎检测
- [x] AABB - 碰撞包围盒
- [x] FLOW_STEP - 流体扩散
- [x] 物理引擎整合 step_physics()

### 1.2 性能优化
- 目标：Tick p95 < 200μs（当前 ~11ms Debug）
- 热路径零堆分配
- SIMD 位运算批量碰撞
- 预计算 LUT（旋转、方向）

### 1.3 测试
- [x] AABB 交集
- [x] 静态实体不下落
- [x] 动态实体会下落
- [x] 脆性材料破碎
- [x] isOccupiedByOther 测试
- [x] water_flow 集成测试

### 1.4 ReleaseSmall 构建
- 修改 build.zig 添加 ReleaseSmall 模式
- 验证二进制 < 5MB

---

## 🎯 Phase 2: 1024³ 虚拟分页

### 2.1 64位地址编码
- [x] 64位地址编码 (address.zig)

### 2.2 页表结构
- [x] 页表结构 (scene1024.zig)

### 2.3 活跃页池
- [x] 活跃页池 (scene1024.zig)

### 2.4 Scene1024
替代 Scene32 作为全局空间
- [ ] 逻辑空间 0..1023
- [ ] 物理只保留活跃页

---

## 🎯 Phase 3: 三世界分页

| 世界 | 内容 | 同步机制 |
|------|------|----------|
| 物理世界 | 拓扑/碰撞/重力/流体 | 硬约束 |
| 心理世界 | 情绪/意图/权重 | 事件广播 |
| 编程世界 | 规则/算子/元数据 | 总线提交 |

---

## 🎯 Phase 4: 心智子系统

### 情绪寄存器
AffectBlock { valence: i8, arousal: u8, certainty: u8, control: u8 }

### 影子沙盒
- 候选方案预演
- 情绪影响调度优先级

---

## 🎯 Phase 5: Hook 接口

pub const HookResult = struct {
    status: enum { PASS, FAIL },
    reason_code: u8,
    break_frame: u32,
    repair_hint: []const u8,
};

---

## 📁 关键文件

| 文件 | 说明 |
|------|------|
| physics.zig | 物理算子核心 |
| tick_engine.zig | Tick 引擎核心 |
| scene32.zig | 32³ 场景沙盒 |
| entity16.zig | 16³ 实体定义 |
| renderer.zig | ASCII 渲染 |
| scenarios.zig | 内置场景 |
| build.zig | 构建配置 |
| main.zig | CLI 入口 |

---

## 🔧 FLOW_STEP 修复详细说明

### 当前 physics.zig 结构

```zig
// 已存在但可能有问题
fn checkFall(...) // 检查下方是否可下落
fn checkFlow(...) // 检查流体可流向何方
fn applyFall(...) // 执行下落
fn applyFlow(...) // 执行流动

// 需要添加
fn isOccupiedByOther(...) // 排除自身的占用检查
```

### isOccupiedByOther 实现参考

```zig
fn isOccupiedByOther(scene: *Scene32, inst: *Instance, entities: []Entity16, sx: i8, sy: i8, sz: i8) bool {
    // 超出边界视为占用
    if (!scene32.inBounds(sx, sy, sz)) return true;
    // 位置空闲
    if (!scene32.isOccupied(scene, sx, sy, sz)) return false;
    // 检查是否是自己
    const entity = &entities[inst.entity_id];
    const lx = sx - inst.pos_x;
    const ly = sy - inst.pos_y;
    const lz = sz - inst.pos_z;
    if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
        if (entity16.testVoxel(entity, lx, ly, lz)) return false;
    }
    return true;
}
```

### 测试脚本

创建 test_water_debug.zig 调试：

```zig
const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");
const scenarios = @import("scenarios.zig");

pub fn main() void {
    var scene = scene32.initScene();
    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(.water_flow, &scene, &entities);
    scene32.rebuildOccupancy(&scene, &entities);
    
    const inst = scene.instances[0];
    std.debug.print("Water at pos=({},{},{})
", .{inst.pos_x, inst.pos_y, inst.pos_z});
    
    const result = physics.checkFlow(&scene, &inst, &entities);
    std.debug.print("checkFlow: flowed={}
", .{result.flowed});
    
    // 测试 checkFall
    const fall = physics.checkFall(&scene, &inst, &entities);
    std.debug.print("checkFall: can_fall={}
", .{fall.can_fall});
}
```

运行：zig run src/test_water_debug.zig

---

## 📂 项目结构

```
worldvm/
├── build.zig          # Zig 构建配置
├── src/
│   ├── main.zig        # CLI 入口 (run/bench/dump)
│   ├── entity16.zig    # 16³ 实体定义 (4KB)
│   ├── scene32.zig     # 32³ 场景沙盒
│   ├── tick_engine.zig # 四步 Tick 引擎
│   ├── physics.zig     # 物理算子
│   ├── renderer.zig     # ASCII 渲染
│   ├── scenarios.zig    # 内置场景
│   └── physics_test.zig # 单元测试
├── docs/world-vm/      # 设计文档
├── tasks.md            # 本任务清单
└── zig-out/            # 编译输出
```

### 运行命令

```bash
# Debug 构建 + 运行
zig build && ./zig-out/bin/worldvm run --scenario apple_table --ticks 50 -v

# 性能基准
zig build && ./zig-out/bin/worldvm bench --scenario apple_table

# 场景导出
zig build && ./zig-out/bin/worldvm dump --scenario water_flow

# 单元测试
zig test src/physics_test.zig
```

### Entity16 结构

- 4KB 定长结构
- topology: u64[64] = 4096 bits = 16³ 体素
- physics: mass, hardness, material, flags
- 坐标映射: bit_index = z + x*16 + y*16*16

### Scene32 结构

- 32³ 空间 (32768 个位置)
- occupancy: u64[512] = 4096B
- 最多 128 个实例
- focus_x/y/z: 焦点中心
- tick_rate: 1 = 每帧更新

### 内置场景

- apple_table: 苹果从高处落到桌面
- hammer_glass: 锤子砸玻璃
- water_flow: 水从高处流下
