# Async Active SLAM with FastSLAM 2.0

异步双循环主动SLAM系统：高频 FastSLAM 2.0 后端 + 低频多准则融合决策。

## 项目概述

本项目实现了一套**异步双循环主动SLAM架构**，核心创新在于将高频状态估计与低频主动决策解耦：

- **高频循环（20-50 Hz）**：FastSLAM 2.0 后端持续运行，负责运动预测、数据关联、EKF landmark更新与粒子重采样，确保定位连续性。
- **低频循环（条件触发）**：多准则效用融合决策器仅在触发条件满足时执行，负责前沿检测、信息增益评估与目标选择，避免目标频繁切换与计算资源浪费。

两层之间通过**共享信念状态**和**状态机**进行协调。该系统采用**双地图策略**：Landmark-based FastSLAM 负责定位，独立的 Occupancy Grid (OG) 地图负责探索决策。

## 项目结构

```
├── active_slam/                  # 自主实现 — 主动SLAM核心
│   ├── async_active_slam_main.m      # 完整异步主循环（含仿真demo）
│   ├── multi_criteria_utility_fusion.m  # 四层融合效用函数
│   ├── state_machine_update.m        # 状态机协调高频/低频循环
│   ├── trigger_conditions.m          # 多维度触发条件判断
│   ├── visualize_realtime.m          # 实时可视化
│   ├── og_map/                       # OG地图模块
│   │   ├── OccupancyGridMap.m        # 自包含OG地图类
│   │   ├── update_occupancy_grid.m   # 逆传感器模型
│   │   ├── bresenham_line.m          # Bresenham射线算法
│   │   └── extract_landmarks_from_scan.m  # 激光扫描 → landmark
│   └── utils/                        # 工具函数
│       ├── extract_pose_info.m       # 粒子云 → 协方差 + D-opt/T-opt
│       ├── compute_local_map_entropy.m   # FOV内Shannon熵
│       ├── compute_renyi_entropy_local.m # Rényi熵折扣
│       ├── build_landmark_graph.m    # 共视图 + 代数连通度 λ₂
│       ├── compute_path_entropy.m    # 路径安全熵
│       ├── detect_frontiers_og.m     # OG地图前沿检测
│       ├── adaptive_weight_scheduler.m   # 自适应权重调度
│       └── predict_pose_and_cov.m    # 位姿/协方差预测
│
├── fastslam/                     # Tim Bailey 开源 FastSLAM
│   ├── fastslam1/                    # FastSLAM 1.0
│   ├── fastslam2/                    # FastSLAM 2.0
│   ├── fastslam2r/                   # FastSLAM 2.0 (rewritten)
│   ├── KF_cholesky_update.m
│   ├── KF_joseph_update.m
│   ├── add_feature.m
│   ├── feature_update.m
│   ├── resample_particles.m
│   ├── sample_proposal.m
│   └── ... (其他核心函数)
│
├── 前后端主循环.md               # 核心设计文档：异步双循环架构
├── 地图实现细节.md               # 核心设计文档：双地图策略与OG地图
└── 融合效用函数.md               # 核心设计文档：四层多准则融合框架
```

> **注意**：`fastslam/` 目录下的代码来源于 **Tim Bailey and Juan Nieto (2004-2006)** 的开源 FastSLAM 仿真器。`active_slam/` 目录下的全部代码为本项目自主实现。

## 核心设计思路

项目根目录下的三个 Markdown 文件记录了部分核心设计思路：

1. **`前后端主循环.md`** — 异步双循环架构设计：
   - 高频 FastSLAM 2.0 后端与低频主动规划器的解耦
   - 四类触发条件（时间/空间/不确定性/事件）
   - 状态机协调（INIT → EXECUTING → REPLANNING → LOOP_CLOSURE → TERMINATED）
   - 完整的数据流时序图与参考架构对比

2. **`地图实现细节.md`** — 双地图策略实现：
   - Landmark-based SLAM 与 OG 地图如何共存
   - 从激光扫描提取 landmark 的完整流程
   - 逆传感器模型（log-odds）更新 OG 地图
   - Bresenham 射线追踪、前沿检测、自适应权重

3. **`融合效用函数.md`** — 多准则效用函数融合框架：
   - 信息论层：Shannon 熵 + Rényi 熵折扣
   - 最优设计层（TOED）：D-opt / T-opt 从粒子协方差提取
   - 图论层：Landmark 共视图的代数连通度 λ₂(L)
   - 几何层：距离惩罚 + 路径安全熵 + 转向惩罚
   - 自适应权重调度与四层融合的统一数学框架

## 快速开始

### 环境要求

- MATLAB（推荐 R2020a 或更高版本）
- **建议**：Image Processing Toolbox（`bwconncomp` 用于前沿检测）
- **可选**：Navigation Toolbox（若有，可将 `OccupancyGridMap` 替换为内置 `occupancyMap`）
- **可选**：ROS Toolbox（如需连接 ROS/Gazebo）

### 运行仿真

在 MATLAB 命令行中执行：

```matlab
>> addpath('fastslam');
>> addpath('fastslam/fastslam2');
>> addpath('active_slam');
>> addpath('active_slam/og_map');
>> addpath('active_slam/utils');
>> [trajectory, maps, entropy_history, state_log, criteria_log] = async_active_slam_main('simulation');
```

仿真将在一个 20 m × 20 m 的矩形环境中运行，机器人自动探索直至地图未知区域比例低于 5% 或无剩余前沿。

### 输出说明

| 输出变量 | 说明 |
|---------|------|
| `trajectory` | 3 × T 真实轨迹 [x; y; θ] |
| `maps` | 地图历史快照（cell 数组）|
| `entropy_history` | 位姿熵历史 |
| `state_log` | 状态机状态历史 |
| `criteria_log` | 各层准则值历史（用于消融实验）|

## 四层效用融合框架

| 准则层 | 核心公式 | 信息源 |
|--------|---------|--------|
| **信息论 (IT)** | H_map(FOV) − H_Rényi(FOV, α)，α = 1 + k_α·ln(1+D_opt) | OG 地图 + 位姿不确定性 |
| **最优设计 (TOED)** | −ln(D_opt_pred) | 粒子协方差矩阵 |
| **图论 (Graph)** | λ₂(L) — 共视图 Fiedler 值 | Landmark 共视图 |
| **几何 (Geo)** | −w_d·dist − w_p·H_path − w_t·\|Δθ\| | 距离 + 路径安全 + 转向 |

**融合公式**：

```
U_total = w_IT·U_IT + w_TOED·U_TOED + w_Graph·U_Graph + w_Geo·U_Geo
```

权重通过 `adaptive_weight_scheduler` 根据探索进度（未知区域比例）、位姿不确定性和粒子退化程度自适应调整。

## 消融实验建议

利用 `criteria_log` 可进行以下对比实验：

| 实验组 | 启用的准则层 |
|--------|-------------|
| Baseline | 仅 U_Geo |
| IT-only | U_IT + U_Geo |
| TOED-only | U_TOED + U_Geo |
| Graph-only | U_Graph + U_Geo |
| Full Fusion | 全部四层 + 自适应权重 |

**评价指标**：探索完成步数、最终定位 RMSE、地图覆盖率、计算开销。

## 关键参数

在 `async_active_slam_main.m` 或调用 `multi_criteria_utility_fusion` 时修改：

| 参数 | 含义 | 推荐范围 |
|------|------|---------|
| `k_alpha` | Rényi 映射系数 | 0.5 ~ 3.0 |
| `w_dist` | 距离惩罚权重 | 0.1 ~ 0.5 |
| `w_path` | 路径熵惩罚权重 | 0.05 ~ 0.3 |
| `w_turn` | 转向惩罚权重 | 0.01 ~ 0.1 |
| `N_period` | 时间触发周期 | 20 ~ 100 步 |
| `spatial_threshold` | 空间触发阈值 | 2.0 ~ 4.0 m |
| `D_opt_threshold` | 高不确定性判定阈值 | 根据实际标定 |

## 注意事项

1. **Landmark 数量控制**：激光端点作为 landmark 会导致每帧数百个点。`extract_landmarks_from_scan` 已做去重，但长期运行仍可能累积。如需限制，可对 `particle.xf` 定期裁剪。
2. **OG 地图边界**：`OccupancyGridMap` 是固定尺寸的。若机器人可能超出初始边界，需预先设定足够大的地图。
3. **计算效率**：效用函数对每个候选目标计算 FOV 熵。若候选前沿很多，建议限制 `n_candidates <= 15`。
4. **数值稳定性**：主循环中已包含 NaN/Inf 清洗与协方差矩阵正则化，但在极端环境下仍可能遇到数值问题。

## 致谢

- **FastSLAM 后端**：基于 Tim Bailey and Juan Nieto (2004-2006) 的开源 MATLAB 实现。
- **理论基础**：Stachniss (2005), Carrillo et al., Khosoussi et al., Placed & Castellanos 等主动 SLAM 经典文献。

## License

- `active_slam/` 目录下的代码采用 MIT License（详见 LICENSE）。
- `fastslam/` 目录下的代码版权属于 Tim Bailey and Juan Nieto，保留其原始许可条款。
