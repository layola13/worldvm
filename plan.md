  # physics.md 驱动的底层优先任务重排图（加入“可预测”一级能力）

  ## 摘要

  - 以 physics.md 的 90 章 / 900 案例 为总纲，当前 src/chapter01-30 不再驱动主任务排
    序。
  - 主线继续坚持 离散、可解释、可回放 的世界物理内核。
  - 在原路线图基础上，新增一个必须并列对待的一级目标：短时可预测（1-5 秒）。
  - 这里的“可预测”不是单纯决定论，而是：
      - 能预计未来几秒谁会出现
      - 能预计碰撞/交汇/通过窗口
      - 能预计红绿灯、安全通过、TTC、短时可达空间
      - 且这些预测结果对上层决策是稳定、可控、可调参的
  - 最终主线变为四层：
      - Layer 1 世界物理内核
      - Layer 1.5 统一短时预测层
      - Layer 2 已规划域扩展
      - Layer 3 远期研究域

  ## 主线重排

  ### Layer 1：世界物理内核

  这是近期唯一基础主线，保持不变，但它现在必须为预测层提供稳定输入。

  #### Phase 1：统一几何与查询底座

  - 落地 AABB -> Narrow -> QueryHit 固定协议，统一 raycast/sweep/overlap/
    penetration。
  - QueryHit 固定包含：
      - geometry hit
      - material_type
      - surface_condition
      - medium_type
      - body_type
  - sphereCast/boxCast 去掉 center-ray 近似。
  - tick_engine 固定为近期权威执行内核；physics_world 只做 orchestrator/wrapper。
  - 目标：给后续预测层提供稳定的“当前状态 + 查询语义”。

  #### Phase 2：接触分类与响应主链

  - 固化接触管线：
      - Detect
      - Classify
      - PairResolve
      - Impulse/Friction
      - DamageEval
      - MediumPost
      - Emit
  - 新增核心结构：
      - Contact
      - ContactClassification
      - MaterialPairResponse
      - ContactTelemetry
  - 睡眠/唤醒、摩擦、下陷、浮力、拖曳等必须进入统一响应路径。
  - 目标：给预测层提供可外推、可解释的动力学规则，而不是一堆分散特判。

  #### Phase 3：约束、CCD 与崩溃防御

  - joint.zig 升级为 lite constraints 子系统，只覆盖近期必要类型：
      - fixed
      - hinge
      - slider
      - spring
      - ball_socket
  - ccd.zig 接入主步进，固定 TOI 子步进、迭代上限和 no-progress watchdog。
  - crash_defense 与 solver、CCD、force field 主链打通。
  - 目标：让预测层面对高速对象、约束对象时仍然能给出可信短时预测，而不是对这类对象彻
    底失明。

  #### Phase 4：事件、快照、回放与决定论

  - TickEngine 每 tick 固定输出：
      - PhysicsTraceEvent
      - ContactTelemetry
      - WorldSnapshot
      - WorldHash
  - rewind 升级为真实世界快照，但只覆盖 Layer 1 必需状态。
  - physics_world 的 broadPhase/handleEvents/recordSnapshot TODO 在本阶段清零。
  - 目标：保证“同一预测输入 + 同一当前状态”可以重复复现，作为预测可信性的底座。

  ### Layer 1.5：统一短时预测层（新增一级能力）

  这是新主线，优先级高于所有域扩展，但建立在 Layer 1 之上。

  #### 目标

  - 提供 1-5 秒滚动预测，服务车辆、交通、传感器、路口博弈、风险评估、短时避障与安全通
    过判断。
  - 不做“长时规划器替代品”，不做全局路径规划。
  - 只解决“未来几秒会发生什么”和“现在还能不能安全做动作”。

  #### 新增统一能力

  - 新增独立预测层，建议命名为 Prediction Layer 或 Future Query Layer，位于：
      - Physics/Query/WorldSnapshot 之上
      - Planner/AI/Vehicle/Traffic/Safety 之下
  - 所有领域共用同一套预测接口，不允许：
      - sensors 自己一套未来估计
      - ai_traffic 自己一套路口预测
      - planner 再私做一套 TTC/through-window 逻辑

  #### 预测层输入

  - 当前世界快照 WorldSnapshot
  - 活跃对象列表与分类信息
  - 统一查询接口 QueryWorldView
  - 已知外部时序事件：
      - 红绿灯状态与切换 ETA
      - 交通灯周期
      - 自车/他车输入约束
      - 执行器上限与反应延迟
  - 可选：传感器置信度/遮挡/退化信息

  #### 预测层最小接口

  - predict_state(entity_id, horizon_s, dt_step)
  - predict_occupancy(region, horizon_s)
  - predict_conflict(a, b, horizon_s)：输出未来冲突窗口
  - compute_ttc(a, b)：统一 TTC，不再散落实现
  - estimate_safe_pass(signal/light/crossing, actor, horizon_s)
  - predict_signal_window(signal_id, horizon_s)
  - predict_risk_score(actor, maneuver, horizon_s)

  #### 预测层最小输出

  - PredictedStateSeries
  - PredictedOccupancySeries
  - ConflictWindow
  - TTCResult
  - SafePassResult
  - RiskAssessment

  #### 预测层必须可控

  - 固定最大预测时间窗：近期只支持 1-5 秒
  - 固定预测步长配置：例如 100ms / 200ms
  - 固定对象数预算：只预测活跃对象与相关邻域
  - 固定求值路径：
      - 先短时轨迹外推
      - 再碰撞/占据/冲突查询
      - 再事件 ETA 合成
      - 再风险判断
  - 不允许预测逻辑直接变成“黑箱启发式脚本集合”

  #### 近期只支持的预测类型

  - TTC / 交汇时间
  - 红绿灯通过窗口
  - 未来 1-5 秒邻域占据
  - 短时碰撞风险
  - 短时安全空间是否存在
  - 短时目标会不会从遮挡侧出现

  #### 近期不支持的预测类型

  - 长时 10 秒以上宏观交通演化
  - 全局路径重规划
  - 灾害大尺度场预测
  - 高阶人体动作意图预测
  - 黑箱学习型轨迹网络

  #### 预测层与现有模块的整合要求

  - sensors.predictObjectPosition 收敛成预测层底层 helper，不再作为终局接口。
  - network.predict 仅保留网络复制/回滚语义，不承担物理世界未来估计。
  - ai_traffic.checkRedLight/checkVehicleAhead 后续应迁移到预测层统一接口之上。
  - 所有 TTC、通过窗口、未来冲突判断都必须共用预测层。

  #### 预测层优先解锁的章节族

  - 直接服务：
      - 18 高级传感器与预测
      - 33-35 多车交互 / AI / 回溯
      - 41-70 自动驾驶、交通、天气、路口、特殊交通
  - 提前铺垫：
      - 71-80 灾害下短时风险预估
  - 不直接服务：
      - 81-90 生物力学/体育/极限运动，暂不作为近期驱动

  ## Layer 2：已规划域扩展

  这些章节保留在路线图里，但只能在 Layer 1 与 1.5 稳定后推进。

  ### Pack A：角色、弹道、破坏、生物刚体

  - 覆盖章节：11-18
  - 顺序固定：
      1. 11 KCC
      2. 12 Ballistics
      3. 13 Destruction
      4. 14 Ragdoll
      5. 16 Force/Explosion
      6. 17 Interaction
      7. 18 Sensors
      8. 15 Vehicle 的基础接入仅作桥接，完整车辆栈转入 Pack B
  - 依赖：
      - Layer 1 Query/Contact/CCD 完整
      - Layer 1.5 预测层最小接口可用
  - 约束：
      - KCC/弹道/破坏/布娃娃都不允许再自建碰撞、地表、风险判断逻辑

  ### Pack B：车辆动力学与竞速域

  - 覆盖章节：21-40
  - 依赖：
      - Layer 1 完整 contact/material/medium
      - Layer 1.5 短时预测层
      - Network/Rewind 决定论口径稳定
  - 约束：
      - 车辆模块必须消费统一 terrain/material/query/prediction
      - 红绿灯/并线/刹停窗口/危险预估不能分散在交通脚本中各写一套

  ### Pack C：感知、交通与端到端驾驶域

  - 覆盖章节：41-70
  - 依赖：
      - Layer 1 的 Query/Trace/Determinism
      - Layer 1.5 的短时预测层
      - Layer 2 Pack B 的 vehicle stack
  - 这是“可预测”最核心的消费层：
      - 预计遮挡后出现的车/人
      - 预计未来几秒可通过的信号窗口
      - 预计并线、路口、汇入的冲突时间
      - 预计未来安全空间是否收缩
  - 约束：
      - 所有短时预测必须走统一 Prediction Layer
      - planner/AI 只做决策，不做第二套世界未来估计

  ### Pack D：灾害与环境极值域

  - 覆盖章节：71-80
  - 依赖：
      - Layer 1 的 medium/terrain/event/snapshot
      - Layer 1.5 的短时预测层
  - 预测层在这里的职责只限：
      - 未来几秒安全空间
      - 灾害波前/风场/洪水局部到达估计
      - 短时避险窗口
  - 不扩张为大尺度气象模拟平台

  ## Layer 3：远期研究域

  ### Research Pack：生物力学、软组织、体育与极限运动

  - 覆盖章节：81-90
  - 继续定义为“研究域”，不进入近期主线。
  - 即使引入预测层，这里仍然不提到近期实现，因为需要：
      - 主动平衡控制
      - 软组织/肌肉驱动
      - 高复杂人体动作预测
      - 水体/装备/肢体多重耦合
  - 只有在以下前提满足后才立项：
      - Layer 1 完整
      - Layer 1.5 稳定
      - Layer 2 Pack A-C 稳定
      - 已确认是否允许局部混合连续子系统

  ## 关键接口与类型调整

  - 保留 TickEngine 为权威固定步进内核；PhysicsWorld 收敛为 orchestrator。
  - 新增或收敛这些核心类型：
      - Contact
      - ContactClassification
      - MaterialPairResponse
      - ContactTelemetry
      - PhysicsTraceEvent
      - WorldSnapshot
      - WorldHash
      - PredictedStateSeries
      - PredictedOccupancySeries
      - ConflictWindow
      - TTCResult
      - SafePassResult
      - RiskAssessment
  - 模块整合要求：
      - sensors 只负责观测与置信度，不再终局负责未来推演
      - network.predict 只保留同步/回滚语义
      - ai_traffic 的红绿灯/跟车/路口风险判断逐步迁入预测层
      - vehicle、planner、traffic、safety 必须共用 Prediction Layer

  ## 验收与测试路线

  ### Layer 1 验收

  - 先通过这些案例族，再允许开启 Layer 1.5：
      - 1-10 基础运动与重力
      - 11-20 基础碰撞检测
      - 21-30 摩擦与弹性
      - 31-40 堆叠与休眠
      - 181-200 中和决定论/崩溃防御直接相关的底层部分
      - 最小场景回归
      - trace/hash 一致性测试

  ### Layer 1.5 验收

  - 必须新增一组“未来 1-5 秒”预测验收，不依赖 UI：
      - 对向来车 TTC
      - 遮挡侧来车短时出现预测
      - 红绿灯切换窗口预测
      - 前车减速后的安全跟车窗口
      - 路口左侧来车未来冲突时间
  - 验收标准：
      - 同输入重复运行结果一致
      - 小扰动参数变化带来连续、可解释的风险变化
      - 预算受控，不会因预测层引入不可上界的耗时
      - planner/traffic/sensors 读取到同一预测结果，不再出现多版本 TTC

  ### Layer 2 验收

  - 每个 Pack 单独收口：
      - Pack A -> Pack B -> Pack C -> Pack D
  - 每个 Pack 都必须提供：
      - 模块级契约测试
      - 至少一组跨子系统集成场景
      - trace/hash 验证
      - 若消费预测层，必须补未来 1-5 秒预测正确性回归

  ### Layer 3 启动门槛

  - 仅当 Layer 1、Layer 1.5、Layer 2 稳定后，才为 81-90 立项。
  - 在此之前，Research Pack 只保留需求文档和前置条件，不进入实现 backlog。

  ## 默认假设

  - “可预测”被正式提升为一级能力，不再被视为决定论或网络预测的附属品。
  - 近期只做 短时 1-5 秒 预测，不做长时宏观演化。
  - 预测层是所有域共享的统一层，不允许交通、传感器、planner 各自发展独立预测逻辑。
  - 总任务源头仍是 physics.md，但实现顺序仍由“世界物理内核 + 短时预测层”决定，而不是
    按 90 章线性推进。
  - 章节测试体系继续延后；在 Layer 1 和 Layer 1.5 未稳定前，不继续做大规模章节壳重
    写。
