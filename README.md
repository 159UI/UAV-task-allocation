# 异构无人集群任务分配仿真系统

# 项目状态
Developing — 核心算法与 GUI 已完成，待参数敏感性分析和实验记录填充

# 当前目标
完成 SA 与 CS 算法的对比实验，填充实验记录，撰写课程设计报告

# 下一步（周一）
1. 运行 001_参数敏感性分析实验（T0/α/Pa/N_pop 对收敛影响）
2. 运行 002_规模鲁棒性测试（N/M/K 变化下的 30 次独立运行）
3. 生成课程设计报告（可用 docx 技能协助）

# 最近更新
- 2026-07-04：修复 MATLAB MCP Error 5201（卡巴斯基拦截），卸载重装后恢复正常
- 2026-07-04：修复 inputParser 参数名冲突（R_range→reward_range，MATLAB 大小写不敏感）
- 2026-07-04：修复 plot_allocation_table 中 uitable 父容器类型问题
- 2026-07-04：调优默认参数（L_max=150, ω=0.3）
- 2026-07-04：SA 和 CS 全部跑通验证

# 已验证参数
```
场景: N=5, M=20, K=8, L_max=150, ω=0.3, P=50, d_safe=2
SA(500iter): J≈52.8, 访问5目标, ≈0.04s
CS(500iter): J≈52.8, 访问5目标, ≈2.35s
```
算法能分配到无人机挂载 2~3 个目标的链式路径，util 分布 34%~94%

# 风险
- MATLAB 在线授权依赖 MathWorks Service Host，卡巴斯基可能再次拦截
- CS 运行时间约为 SA 的 50 倍，大规模场景需考虑

# 相关 ADR
（暂无）

---

## 项目概述

本系统面向**海事巡检场景**，实现基于 MATLAB 的异构无人集群任务分配仿真。核心功能：

- **两种启发式算法**：模拟退火（SA，经典）与布谷鸟搜索（CS，2009）
- **统一编码策略**：优先级向量法 + 贪婪分配解码
- **避障路径规划**：惩罚函数法处理静态圆形障碍物
- **完整 GUI 交互**：场景参数调优、算法运行、多维度对比

## 目录结构

```
src/
├── run_main.m                  # 脚本入口（无 GUI 调试模式）
├── uav_main_gui.m              # 🖥️ 主 GUI 界面（程序化，不依赖 App Designer）
├── generate_scenario.m         # 随机场景生成（inputParser 参数解析）
├── collision_check.m           # 线段-障碍物碰撞检测（点到线段最短距离法）
├── distance_penalty.m          # 惩罚法距离计算：欧氏距离 + P×穿越数
├── decode_individual.m         # ⭐ 优先级向量→分配方案解码（贪婪分配）
├── calc_objective.m            # 目标函数 J = ΣR - ω·Σ(L_i/L_max) 计算
├── simulated_annealing.m       # 🌡️ SA 算法（Swap/Insert/Reverse 邻域操作）
├── cuckoo_search.m             # 🪺 CS 算法（Mantegna 莱维飞行 + 发现丢弃）
├── levy_flight.m               # 莱维飞行步长生成（Mantegna 算法）
├── plot_path.m                 # 二维路径图（含障碍物圆形标注）
├── plot_convergence.m          # 收敛曲线（SA/CS 同图对比）
└── plot_allocation_table.m     # 分配结果表（axes 文本模式 / uitable 模式自适应）
```

## 运行方式

```matlab
% 方式一：GUI 模式（推荐）
>> uav_main_gui

% 方式二：脚本模式（批量实验）
>> run_main
```

## 关键参数说明

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| N | 5 | 无人机数量 |
| M | 20 | 目标数量 |
| K | 8 | 障碍物数量 |
| L_max | 150 | 每架无人机最大航程（场景 100×100 需 ≥150） |
| ω | 0.3 | 航程权衡系数（越小越追求收益） |
| P | 50 | 避障惩罚常数 |
| T0 (SA) | 100 | 初始温度 |
| α (SA) | 0.95 | 降温速率 |
| N_pop (CS) | 25 | 种群规模 |
| Pa (CS) | 0.25 | 发现概率 |

## MATLAB 兼容性说明

- **MATLAB R2024a** 开发验证
- `inputParser` 参数名**大小写不敏感**（`R_range` 与 `r_range` 冲突 → 改为 `reward_range`）
- `uitable` 不能以 axes 为父级 → 自适应判断父容器类型

## 数学模型

### 目标函数
$$J(\Pi) = \sum_{j \in V} R_j - \omega \cdot \sum_{i=1}^N \frac{L_i}{L_{\max}}$$

### 约束条件
1. **互斥约束**：每个目标最多被访问一次
2. **航程约束**：每架无人机实际航程 ≤ L_max
3. **起降约束**：所有路径以基地为起点和终点
