# 主动SLAM — 多准则融合效用函数实现

## 文件结构

```
active_slam/
├── multi_criteria_utility_fusion.m      # 核心：四层融合效用函数
├── trigger_conditions.m                 # 多维度触发条件判断
├── state_machine_update.m               # 状态机协调高频/低频循环
├── async_active_slam_main.m             # 完整异步主循环（含仿真demo）
├── og_map/
│   ├── OccupancyGridMap.m               # 自包含OG地图类（无需Navigation Toolbox）
│   ├── bresenham_line.m                 # Bresenham射线算法
│   ├── update_occupancy_grid.m          # 逆传感器模型更新OG地图
│   └── extract_landmarks_from_scan.m    # 激光扫描 → landmark + 最近邻数据关联
└── utils/
    ├── extract_pose_info.m              # 粒子云 → 位姿协方差 + D-opt/T-opt + N_eff
    ├── compute_local_map_entropy.m      # FOV内Shannon熵
    ├── compute_renyi_entropy_local.m    # FOV内Rényi熵（含位姿不确定性折扣）
    ├── build_landmark_graph.m           # 共视图 + 代数连通度λ₂(L)
    ├── compute_path_entropy.m           # 路径安全熵（Bresenham）
    ├── detect_frontiers_og.m            # OG地图前沿检测（需Image Processing Toolbox）
    ├── adaptive_weight_scheduler.m      # 自适应四层权重调度
    └── predict_pose_and_cov.m           # 预测动作后的位姿/协方差/D-opt
```

## 与现有FastSLAM代码的关系

本实现**复用** `fastslam/` 目录下 Tim Bailey 的核心函数：
- `fastslam2/sample_proposal.m` — FastSLAM 2.0 最优提议分布
- `feature_update.m` — EKF landmark更新
- `add_feature.m` — 新增landmark初始化
- `resample_particles.m` — 系统重采样
- `compute_jacobians.m`, `multivariate_gauss.m`, `pi_to_pi.m` — 底层工具

**需要添加的适配层**（已在本目录实现）：
1. `extract_landmarks_from_scan.m` — 替换原有的预定义landmark观测，改为从激光扫描实时提取
2. `OccupancyGridMap` + `update_occupancy_grid.m` — 双地图策略中的独立OG地图

## 快速开始

### 1. 运行仿真demo

在MATLAB中：
```matlab
% 将现有fastslam代码加入路径
addpath('fastslam');
addpath('fastslam/fastslam2');

% 将本目录加入路径
addpath('active_slam');
addpath('active_slam/og_map');
addpath('active_slam/utils');

% 运行完整主动SLAM仿真
[trajectory, maps, entropy_history, state_log, criteria_log] = async_active_slam_main('simulation');
```

### 2. 单独测试效用函数

```matlab
% 创建简易OG地图
og_map = OccupancyGridMap(20, 20, 0.1, [-10, -10]);

% 构造假粒子
for i = 1:100
    particles(i).xv = [randn()*0.5; randn()*0.5; randn()*0.1];
    particles(i).w = 1/100;
    particles(i).xf = [1, 2, 3; 1, -1, 2];  % 3个landmarks
    particles(i).Pf = repmat(eye(2)*0.1, [1,1,3]);
end

% 候选目标
candidates = [5, 0; 0, 5; -5, 0; 0, -5];
robot_pose = [0; 0; 0];

% 计算效用
params = struct('sensor_range', 10, 'k_alpha', 1.0);
[utilities, best_idx, criteria] = multi_criteria_utility_fusion(...
    particles, og_map, candidates, robot_pose, params);

disp(utilities);
```

## 四层效用框架公式速查

| 层 | 公式 | 信息源 |
|---|---|---|
| **IT** | `H_map(FOV) - H_Renyi(FOV, α)`，其中 `α = 1 + k_α·ln(1+D_opt_pred)` | OG地图 + 位姿不确定性 |
| **TOED** | `-ln(D_opt_pred)` | 粒子协方差 |
| **Graph** | `λ₂(L)` — 共视图Fiedler值 | landmark共视图 |
| **Geo** | `-w_d·dist - w_p·H_path - w_t·\|Δθ\|` | 距离 + 路径安全 + 转向 |

融合：`U_total = w_IT·U_IT + w_TOED·U_TOED + w_Graph·U_Graph + w_Geo·U_Geo`

## 关键参数调优

在 `multi_criteria_utility_fusion.m` 中修改 `utility_params`：

| 参数 | 含义 | 推荐范围 |
|---|---|---|
| `k_alpha` | Rényi映射系数 | 0.5 ~ 3.0 |
| `w_dist` | 距离惩罚权重 | 0.1 ~ 0.5 |
| `w_path` | 路径熵惩罚权重 | 0.05 ~ 0.3 |
| `w_turn` | 转向惩罚权重 | 0.01 ~ 0.1 |
| `D_opt_threshold` | 高不确定性判定阈值 | 根据实际标定 |

## 消融实验建议

`criteria_log` 输出每次决策时各层的原始值，可用于：
- **Baseline**: 仅 `U_Geo`
- **IT-only**: `U_IT + U_Geo`
- **TOED-only**: `U_TOED + U_Geo`
- **Full Fusion**: 全部四层 + 自适应权重

对比指标：探索完成步数、最终定位RMSE、地图覆盖率。

## 依赖项

- **必需**: MATLAB基础环境
- **建议**: Image Processing Toolbox（`bwconncomp` 用于前沿检测）
- **可选**: Navigation Toolbox（若有，可将 `OccupancyGridMap` 替换为内置 `occupancyMap`）
- **ROS模式**: ROS Toolbox（连接Gazebo时使用）

## 注意事项

1. **landmark数量控制**: 激光端点作为landmark会导致每帧数百个点。`extract_landmarks_from_scan` 已做去重（同一landmark的多束激光合并），但长期运行仍可能累积。如需限制，可对 `particle.xf` 定期裁剪或只保留当前视野内的landmark。
2. **OG地图边界**: `OccupancyGridMap` 是固定尺寸的。若机器人可能超出初始边界，需动态扩容或预先设定足够大的地图。
3. **计算效率**: 效用函数对每个候选目标计算FOV熵。若候选前沿很多，建议限制 `n_candidates <= 15`。
