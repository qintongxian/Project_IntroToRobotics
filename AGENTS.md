# AGENTS.md — Async Active SLAM

## 项目背景

本项目是一个 MATLAB 实现的异步主动 SLAM 系统，核心架构为：
- **高频循环**：FastSLAM 2.0 后端（基于 Tim Bailey 开源代码），持续运行保证定位连续性
- **低频循环**：多准则效用融合决策器，仅在触发条件满足时执行

**关键区分**：`fastslam/` 是第三方开源代码（Tim Bailey, 2004-2006），`active_slam/` 是自主实现。

## 技术栈

- **语言**：MATLAB（无 Simulink）
- **依赖工具箱**：基础 MATLAB 即可运行；Image Processing Toolbox 建议（前沿检测用 `bwconncomp`）；Navigation Toolbox 和 ROS Toolbox 可选
- **第三方代码**：`fastslam/` 目录不可随意修改接口，但可在 `active_slam/` 中做适配层

## 代码规范

### 文件组织
- `active_slam/*.m` — 顶层模块（主循环、效用融合、状态机、触发条件）
- `active_slam/og_map/*.m` — OG 地图与传感器模型
- `active_slam/utils/*.m` — 辅助函数
- `fastslam/` — 只读第三方代码，修改需谨慎

### 命名约定
- 函数名使用 `snake_case`
- 结构体字段：`.xv` (位姿), `.xf` (landmark), `.Pf` (协方差), `.w` (权重) — 沿用 Tim Bailey 的命名以兼容 `fastslam/` 代码
- 自定义结构体使用完整单词，如 `trigger_params`, `utility_params`

### 关键接口

#### FastSLAM 粒子结构（来自 Tim Bailey 代码）
```matlab
particle.xv   % 3×1 位姿 [x; y; theta]
particle.Pv   % 3×3 位姿协方差
particle.xf   % 2×N landmark 坐标
particle.Pf   % 2×2×N landmark 协方差
particle.w    % 标量权重
particle.da   % 数据关联历史
```

#### 主程序入口
```matlab
[trajectory, maps, entropy_history, state_log, criteria_log] = async_active_slam_main('simulation');
```

#### 效用融合函数
```matlab
[utilities, best_idx, criteria] = multi_criteria_utility_fusion(...
    particles, og_map, candidate_goals, robot_pose, params);
```

## 修改注意事项

1. **FastSLAM 接口层**：`extract_landmarks_from_scan.m` 是连接激光扫描与 Tim Bailey FastSLAM 的关键适配层。修改此文件时需确保输出 `zf`（已知 landmark 观测）、`idf`（索引）、`zn`（新 landmark）的格式与 `sample_proposal.m` / `add_feature.m` 兼容。

2. **OG 地图类**：`OccupancyGridMap.m` 是自包含实现（不依赖 Navigation Toolbox）。字段包括 `Width`, `Height`, `Resolution`, `Origin`, `LogOdds`。提供了 `getOccupancyMatrix()`, `unknownRatio()`, `world2grid()` 等方法。

3. **状态机状态**：`'INIT'`, `'EXECUTING'`, `'REPLANNING'`, `'LOOP_CLOSURE'`, `'TERMINATED'`。修改状态转换逻辑时请同步更新 `state_machine_update.m` 和 `async_active_slam_main.m` 中的 `switch` 分支。

4. **触发条件优先级**：时间 → 空间 → 不确定性 → 事件（在 `trigger_conditions.m` 中按此顺序判断）。调整顺序会影响系统行为。

5. **数值稳定性**：主循环中已有 NaN/Inf 清洗和协方差正则化。新增涉及矩阵求逆或特征值分解的代码时，务必加入类似的容错处理。

## 运行与测试

### 基本运行
```matlab
addpath('fastslam');
addpath('fastslam/fastslam2');
addpath('active_slam');
addpath('active_slam/og_map');
addpath('active_slam/utils');
[trajectory, maps, entropy_history, state_log, criteria_log] = async_active_slam_main('simulation');
```

### 单独测试模块
- 测试效用函数：参考 `active_slam/README_active_slam.md` 中的"单独测试效用函数"部分
- 测试 OG 地图：直接构造 `OccupancyGridMap` 对象并调用 `update_occupancy_grid`
- 测试前沿检测：在已有 OG 地图上调用 `detect_frontiers_og`

## 已知限制

- 仿真模式使用简化的点 landmark 环境和纯追踪控制器，未包含复杂障碍物碰撞检测
- ROS 模式 (`'ros'`) 在主程序中有接口占位但尚未完整实现
- `OccupancyGridMap` 为固定尺寸，不支持动态扩容
- 激光扫描转 landmark 时每帧可能产生大量点，长期运行内存会增长

## 文档索引

| 文档 | 内容 |
|------|------|
| `README.md` | 项目概览、快速开始、运行说明 |
| `前后端主循环.md` | 异步双循环架构设计细节 |
| `地图实现细节.md` | 双地图策略、OG 地图更新、前沿检测 |
| `融合效用函数.md` | 四层效用融合框架、公式推导、参数调优 |
| `active_slam/README_active_slam.md` | 文件结构、与 FastSLAM 的关系、公式速查 |
