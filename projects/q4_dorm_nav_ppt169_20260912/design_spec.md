# q4_dorm_nav_ppt169_20260912 - Design Spec

> Human-readable design narrative — rationale, audience, style, color choices, content outline. Read once by downstream roles for context.
>
> Machine-readable execution contract: `spec_lock.md`. Executor re-reads `spec_lock.md` before every SVG page. On divergence, `spec_lock.md` wins.

## I. Project Information

| Item | Value |
| ---- | ----- |
| **Project Name** | q4_dorm_nav_ppt169_20260912 |
| **Canvas Format** | PPT 16:9 (1280×720) |
| **Page Count** | 9 |
| **Design Style** | B) General Consulting + 学术答辩 / 极简信息图 |
| **Target Audience** | 课程指导教师与同班同学（《智能汽车与自动驾驶》课程） |
| **Use Case** | 课堂口头汇报，约 4 分钟；投影仪 16:9 播放 |
| **Created Date** | 2026-09-12 |

---

## II. Canvas Specification

| Property | Value |
| -------- | ----- |
| **Format** | PPT 16:9 |
| **Dimensions** | 1280×720 |
| **viewBox** | `0 0 1280 720` |
| **Margins** | 左右 80px，上 56px，下 48px（沿用模板封面标题 x=81 的左边界） |
| **Content Area** | 1120×616 |

---

## III. Visual Theme

### Theme Style

- **Style** | B) General Consulting + 学术答辩 / 极简信息图
- **Theme**: Light theme
- **Tone**: 学术、克制、结论先行；信息密度中等偏低，靠留白和分节线而不是卡片堆叠

> **模板配色校准（沿用问题一）**：`docs/东大模板6.pptx` 的 theme XML 把 accent1 写成东大红 `#E92E25`，但把 25 张幻灯片全部导出为 SVG 后统计像素用色，实际主色是**深蓝 `#17375E` / `#002060` + 金黄 `#FFC000` + 正文深灰 `#404040`**；东大红只在目录页（slide_02）作为强调色出现。本 deck 采用模板**实际**的视觉语言：深蓝负责结构、金黄负责强调、深灰负责正文，**东大红降级为"否定 / 警告"专用色**（用于 ✗ 与被推翻的假设、以及"未达标"的那一格）。红色只标记"错的"，不参与装饰。

### Color Scheme

| Role | HEX | Purpose |
| ---- | --- | ------- |
| **Background** | `#FFFFFF` | 页面背景 |
| **Secondary bg** | `#F2F2F2` | 表格斑马纹、公式条底、注释块 |
| **Primary** | `#17375E` | 页面标题、结构线、章节号、图标主色 |
| **Accent** | `#FFC000` | 关键数字、主因标记、强调下划线、页脚色条 |
| **Secondary accent** | `#002060` | 深蓝底块、封面装饰多边形、结论条底色 |
| **Body text** | `#404040` | 正文 |
| **Secondary text** | `#757070` | 注释、单位、补充说明 |
| **Tertiary text** | `#A6A6A6` | 页码、脚注 |
| **Border/divider** | `#D9D9D9` | 表格线、分节线 |
| **Success** | `#2E7D32` | 已验证 / 结论成立的标记（✓） |
| **Warning** | `#E92E25` | 未达标的一格、被推翻的假设、陷阱（✗ ⚠） |

### Gradient Scheme

本 deck 不使用渐变。模板的层次感来自**旋转梯形多边形的叠压 + 投影**，Executor 应沿用这一手法（见 §V），而不是用渐变模拟。

---

## IV. Typography System

### Font Plan

**Typography direction**: 模板原生路线——Century Gothic 负责拉丁文与数字（公式、单位、`1450.6 m` 的几何感），微软雅黑负责中文；全片只用这一组搭配。

| Role | Chinese | English | Fallback tail |
| ---- | ------- | ------- | ------------- |
| **Title** | `"Microsoft YaHei"` | `"Century Gothic"` | `sans-serif` |
| **Body** | `"Microsoft YaHei", "PingFang SC"` | `Arial` | `sans-serif` |
| **Emphasis** | — | `"Century Gothic"` | `sans-serif` |
| **Code** | — | `Consolas, "Courier New"` | `monospace` |

**Per-role font stacks**:

- Title: `"Century Gothic", "Microsoft YaHei", "PingFang SC", sans-serif`
- Body: `"Microsoft YaHei", "PingFang SC", Arial, sans-serif`
- Emphasis: `"Century Gothic", Arial, sans-serif`
- Code: `Consolas, "Courier New", monospace`

> 公式与变量单独走 Code 栈（Consolas 覆盖希腊字母 δ κ α），避免公式被比例字体撑得参差。

### Font Size Hierarchy

**Baseline**: Body font size = 20px

| Purpose | Ratio to body | This deck | Weight |
| ------- | ------------- | --------- | ------ |
| Cover title (hero headline) | 2.5-5x | 64px | Bold |
| Chapter / section opener | 2-2.5x | — (无过渡页) | Bold |
| Page title | 1.5-2x | 34px | Bold |
| Hero number (consulting KPIs) | 1.5-2x | 36px | Bold |
| Subtitle | 1.2-1.5x | 26px | SemiBold |
| **Body content** | **1x** | **20px** | Regular |
| Annotation / caption | 0.7-0.85x | 15px | Regular |
| Page number / footnote | 0.5-0.65x | 12px | Regular |

---

## V. Layout Principles

### Page Structure

- **Header area**: y 56–120，页标题 34px 深蓝居左，标题下方一条 `#FFC000` 短色条（宽 56px、高 5px）作为固定记号
- **Content area**: y 140–660，宽 1120（x 80–1200）
- **Footer area**: y 664–700，左侧 12px `#A6A6A6` 页码，右侧 12px `#A6A6A6` 短标签；`#FFC000` 竖色条贴右下角（沿用模板的 19.1×57.31 页脚色条）

### Layout Pattern Library

| Pattern | 用在 |
| ------- | ---- |
| **Full-bleed + floating text** | P01 封面、P09 结尾（继承模板 cover / ending shell） |
| **Single column centered** | P08 结论（breathing，靠留白与分隔线组织） |
| **Asymmetric split (4:6)** | P03 / P04 / P05 / P06 —— 左列结论与数字，右列数据图 |
| **Top-bottom split** | P02（上方流程条 + 下方动图）、P07（上方表 + 下方双图） |

> **统一手法（模板视觉语言）**：装饰不用渐变与卡片阴影，而用**旋转梯形多边形叠压**（`#17375E` → `#FFC000` → `#002060`，配 `feDropShadow`）。内容页只在右上角放一组小尺寸多边形做记号，不抢占正文区。

### Spacing Specification

**Universal**

| Element | Recommended Range | Current Project |
| ------- | ---------------- | --------------- |
| Safe margin from canvas edge | 40-60px | 左右 80px / 上 56px / 下 48px |
| Content block gap | 24-40px | 32px |
| Icon-text gap | 8-16px | 12px |

**Non-card containers**（P08 为无卡片页）

- 纵向节奏由**留白 + 1px `#D9D9D9` 分隔线**承担，块间距 36–44px
- **Line-height**: 1.5× body = 30px
- 公式条独占一行，底 `#F2F2F2` 圆角 6px、内边距 10/16
- 编号（① ② ③）用 34px `#FFC000` Century Gothic，与正文 20px 形成字号落差

**Table containers**（P06 / P07）

- 表头行底 `#17375E`、文字 `#FFFFFF` 18px；数据行 20px；斑马纹 `#F2F2F2`
- 单元格内边距 12/16；行高 46px；表底线 1px `#D9D9D9`
- 关键数字用 `#FFC000` 或 `#17375E` Bold，不加粗整行
- **未达标的一格**用 `#E92E25`（P06 的 MPC @ 8 m/s）

---

## VI. Icon Usage Specification

### Source

- **Built-in icon library**: `templates/icons/tabler-outline`（线性风格，与本 deck 的克制调性一致）
- **Usage method**: SVG placeholder `<use data-icon="tabler-outline/<name>" .../>`
- **stroke_width**: `2` —— 全片每一个 `<use data-icon>` 都必须带 `stroke-width="2"`

### Recommended Icon List

| Purpose | Icon Path | Page |
| ------- | --------- | ---- |
| 地图构建 / 路网 | `tabler-outline/map-2` | P02, P03 |
| 参考路径 / 寻路 | `tabler-outline/route-2` | P02, P04 |
| 车辆与转向 | `tabler-outline/steering-wheel` | P02, P06 |
| 实验分析 | `tabler-outline/trending-up` | P02, P07 |
| 关键技术点 | `tabler-outline/target` | P04, P08 |
| 陷阱 / 曲率尖峰 | `tabler-outline/alert-triangle` | P04 |
| 速度剖面 / 限速 | `tabler-outline/gauge` | P05 |
| 几何与公式机理 | `tabler-outline/math-function` | P05 |
| 分岔取舍 | `tabler-outline/git-branch` | P06 |
| 已验证 | `tabler-outline/circle-check` | P03, P06 |
| 未达标 | `tabler-outline/circle-x` | P06, P08 |
| 结论 / 建议 | `tabler-outline/bulb` | P08 |

> 以上 12 个图标名均已在 `templates/icons/tabler-outline/` 下逐个核实存在。Executor 只能使用此清单内的图标；清单外一律不用图标（P01 / P09 无图标）。

---

## VII. Visualization Reference List

**Read-audit** (mandatory):

```
Catalog read: 70 templates / 10 categories

Per-page selection (one row per viz page):
  P02 process_flow      | summary-quote: "Pick for 3-8 sequential steps connected by simple arrows."
  P03 kpi_cards         | summary-quote: "Pick for 4-8 standalone numeric metrics shown as overview cards (2x2 or 1x4)."
  P04 bar_chart         | summary-quote: "Pick for single-series category value comparison, 3-8 categories."
  P05 kpi_cards         | summary-quote: "Pick for 4-8 standalone numeric metrics shown as overview cards (2x2 or 1x4)."
  P06 comparison_table  | summary-quote: "Pick for 2-4 plans/products compared across many feature rows (dense matrix)."
  P07 basic_table       | summary-quote: "Pick for plain tabular text/number grid, 3-8 columns."
  P08 vertical_list     | summary-quote: "Pick for 3-6 numbered key points each with a short description."

Runners-up considered (6 entries, drawn from real second-best matches in this deck):
  numbered_steps   | rejected for P02: the four stages are a pipeline with an explicit deliverable per stage (GPX / 限速剖面 / 控制器 / 结论), and process_flow's summary covers connector arrows between stages; numbered_steps reserves itself for "3-6 horizontal sequential steps with numeric emphasis", which would emphasise count over hand-off.
  dumbbell_chart   | rejected for P04: dumbbell needs "5-10 items" of before-vs-after; P04 compares only three processing states of one quantity (κ_max), below the item band.
  line_chart       | rejected for P05: P05's numbers are three independent operating-point values (设计上限 / 急弯限速 / 减速段长度), not a continuous series over a shared axis — the PNG already carries the continuous curve.
  feature_matrix_table | rejected for P06: its summary gates on "competitive feature checklist with checkmarks across products"; P06 compares three controllers on four *numeric* metrics, which is a dense numeric matrix, not a feature checklist.
  bullet_chart     | rejected for P07: bullet_chart requires an explicit target per KPI; P07's three speeds have no target values, only the monotone direction.
  icon_grid        | rejected for P08: only 3 conclusions, below icon_grid's "4-9 parallel features" band, and each carries a two-clause engineering statement rather than a short capability label.
```

| Visualization Type | Reference Template | Used In |
| ------------------ | ------------------ | ------- |
| process_flow | `templates/charts/process_flow.svg` | P02 |
| kpi_cards | `templates/charts/kpi_cards.svg` | P03, P05 |
| bar_chart | `templates/charts/bar_chart.svg` | P04 |
| comparison_table | `templates/charts/comparison_table.svg` | P06 |
| basic_table | `templates/charts/basic_table.svg` | P07 |
| vertical_list | `templates/charts/vertical_list.svg` | P08 |

> **与内嵌图的分工**：P02–P07 各自还内嵌 1–2 张 matplotlib 出图的 PNG / GIF（见 §VIII）。这些 PNG 是**证据**，上述图表模板提供的是**结论的骨架**（流程条、指标卡、对照表）。Executor 不得用图表模板重画 PNG 里已有的数据曲线——两者是"结论条 + 证据图"的关系，不是重复。

---

## VIII. Image Resource List

全部为**已有图片**（不做 AI 重新生成，也不做矢量重绘）。来源 `outputs/dorm_figures/`，已复制到本项目 `images/`。

| Filename | Dimensions | Ratio | Purpose | Type | Acquire Via | Status | Reference |
| -------- | --------- | ----- | ------- | ---- | ----------- | ------ | --------- |
| `fig13_chase_cam.gif` | 700×489 | 1.43 | P02 全程跟随视角动画（任务全貌） | Diagram | user | Existing | 证据图：车实际跑完全程的第一视角 |
| `fig1_road_graph.png` | 1081×1026 | 1.05 | P03 校区路网与寻路结果 | Diagram | user | Existing | 证据图：1745 节点 / 1989 边 |
| `fig3_curvature.png` | 1417×637 | 2.22 | P04 曲率：原始折线 vs 平滑后（对数级差） | Diagram | user | Existing | 证据图：κ 从 1.60 降到 0.117 |
| `fig4_speed_profile.png` | 1417×637 | 2.22 | P05 曲率限速后的速度剖面与减速斜坡 | Diagram | user | Existing | 证据图：8 → 0 的 15 m 斜坡 |
| `fig6_algorithms_map.png` | 1156×527 | 2.19 | P06 PP / LQR / MPC 三种算法轨迹叠底图 | Diagram | user | Existing | 证据图：三条轨迹都贴线 |
| `fig9_error_vs_speed.png` | 1027×701 | 1.47 | P07 横向误差随速度单调上升 | Diagram | user | Existing | 证据图：3 → 5 → 7 m/s |
| `fig11_error_vs_lookahead.png` | 1091×701 | 1.56 | P07 误差与转向抖动随前视距离的权衡 | Diagram | user | Existing | 证据图：L_f 扫描 |
| `fig7_corner_zoom.png` | 880×897 | 0.98 | P08 全程唯一急弯（s=589 m, R2.4 m）放大 | Diagram | user | Existing | 证据图：误差峰值全部落在这里 |

> **全部标 `no-crop`**：这些是实验数据图，含坐标轴刻度与图例，裁切会丢信息。Executor 必须按原图比例定容器并 `preserveAspectRatio="xMidYMid meet"`。
>
> **GIF 特例**：`fig13_chase_cam.gif` 是 60 帧动图（2.79 MB，约 5 s 循环）。导出为 PPTX 时按 `image/gif` 内嵌，放映模式下自动播放。容器按 700×489 原比例。
>
> **注意**：Executor **不得**用 Read 打开这些图片查看内容。图片信息一律以本表与 `spec_lock.md images` 为准。

---

## IX. Content Outline

### Part 1: 任务与地图

#### Slide 01 - 封面

- **Layout**: 继承模板 cover shell（`templates/dongda/svg-flat/slide_01.svg`）
- **Title**: 从寝室到教室：一次完整的自主导航
- **Subtitle**: 问题四：地图构建 · 参考路径 · 轨迹跟踪 · 实验分析
- **Info**: 东南大学 / 汇报人 · 指导教师（沿用模板两栏，姓名留空待填）

#### Slide 02 - 任务全貌：四个环节 ★

- **Layout**: Top-bottom split — 上方四段流程条 + 下方动画
- **Title**: 从地图到结论，四个环节串成一条链
- **Visualization**: process_flow (see §VII)
- **Content**:
  - ① 地图构建 —— 离线 OSM 路网建图，Dijkstra 寻路 → 交付 `GPX`
  - ② 参考路径生成与设置 —— 倒圆角 + 曲率平滑 + 曲率限速 → 交付 `速度剖面`
  - ③ Pure Pursuit 实现 —— 前视点 → 阿克曼转角，与 LQR / MPC 同接口互换 → 交付 `控制器`
  - ④ 实验数据分析 —— 16 组对照 + 18 次参数扫描 → 交付 `结论`
  - 摘要条：全程 `1450.6 m`，`2903` 个参考点，`7` 个拐点，离线运行不联网
  - 下图：`fig13_chase_cam.gif`

#### Slide 03 - 地图构建

- **Layout**: Asymmetric split (4:6) — 左列指标卡，右列路网图
- **Title**: 用仓库自带的离线 OSM 切片建一张可通行路网
- **Visualization**: kpi_cards (see §VII)
- **Content**:
  - 指标卡：`1745` 节点 / `1989` 边 / `146.7 km` 总长 / `1` 个连通分量
  - 数据源：`planet_118.792,31.875_118.837,31.902.osm.geojson.xz`（1.3 × 3 km），标准库 `lzma` 解压，不装新依赖
  - 建图：OSM 的 way 拆成边，路口（被多条 way 共用的节点）合并成同一顶点
  - **连通分量 = 1**：整片校区路网连通，任意两点一定可达
  - 地标吸附：橘园 8 舍偏离 `44.4 m`（节点 227），教学楼 1 偏离 `74.5 m`（节点 1109）—— 这是建筑质心到道路中心线的真实距离，不是定位误差
  - 寻路：Dijkstra，`54` 个路网节点，原始折线 `1463.5 m`
  - 右图：`fig1_road_graph.png`

#### Slide 04 - 参考路径：全程最大的坑 ★

- **Layout**: Asymmetric split (4:6) — 左列三步阶梯 + 结论，右列曲率图
- **Title**: OSM 折线在路口是尖角，曲率超车辆能力 5.7 倍
- **Visualization**: bar_chart (see §VII)
- **Content**:
  - 现象：两条路在路口共用一个节点，转向接近 90°。重采样后直接微分 → `κ_max = 1.6049 1/m`（`R = 0.62 m`）
  - 车辆物理极限：`tan(35°) / 2.50 = 0.280 1/m`（`R = 3.57 m`）→ **参考路径要求的是车辆能力的 5.7 倍**，控制器全程饱和
  - 柱状（三步阶梯）：原始折线不平滑 `1.6049` / 原始折线 + 平滑 `0.1355` / 倒圆角 + 平滑 `0.1166`（单位 1/m）
  - **关键结论（金黄框）**：平滑贡献 `92%` 的改善，倒圆角只有 `14%`。倒圆角仍然值得做——它让参考路径在几何上是**物理可实现的圆弧**，而不只是一条被抹平的折线
  - 陷阱（⚠ `#E92E25`）：原代码只在"曲率限速"档平滑曲率，"匀速"档用原始微分 —— 导致"倒圆角 + 匀速"报出 `25.4 m/s²`，反而比"原始 + 匀速"的 `17.8` 更差。**曲率是路径的属性，不是速度剖面的属性**，已改为无条件平滑
  - 右图：`fig3_curvature.png`

### Part 2: 跟踪与实验

#### Slide 05 - 速度设置：把曲率变成限速

- **Layout**: Asymmetric split (4:6) — 左列三张指标卡 + 递推说明，右列速度剖面
- **Title**: 曲率限速 + 两遍递推，把每个点的速度压到可行域内
- **Visualization**: kpi_cards (see §VII)
- **Content**:
  - 指标卡：`2.5 m/s²` 设计侧向加速度上限 / `4.47 m/s` 急弯处限速 / `15 m` 减速斜坡长度
  - 公式：`v = √(a_lat_max / κ)`；直线段顶到目标速度，`#16` 拐点（`κ = 0.417 1/m`）限到 `4.47 m/s`
  - 两遍扫描：**倒推**刹车 `vᵢ² ≤ vᵢ₊₁² + 2·a_brake·Δs`，**正推**加速 `vᵢ² ≤ vᵢ₋₁² + 2·a_accel·Δs`
  - 陷阱（⚠）：两遍必须写成**逐点递推**，不能写成数组表达式。写成 `np.sqrt(speed[1:]**2 + 2a·ds)` 时右边用的是**未限速的**原始数组，`min()` 之后除最后一点外什么都没改 —— 车会以巡航速度冲到最后 `0.12 m` 才刹车
  - 修正后：`8 → 0` 的斜坡长 `15 m`，峰值侧向加速度精确等于 `2.500 m/s²`
  - 右图：`fig4_speed_profile.png`

#### Slide 06 - 三种算法对比

- **Layout**: Asymmetric split (4:6) — 左列对照表 + 一句话，右列轨迹图
- **Title**: 同一参考路径，PP / LQR / MPC 的取舍
- **Visualization**: comparison_table (see §VII)
- **Content**:
  - 表（`@5 m/s`）：| | 误差均值 | 误差峰值 | 最大前轮转角 |
    - PP：`0.051` / `1.593` / `28.4°` —— 实现 20 行，误差峰值最大
    - LQR：`0.036` / `0.521` / `35.0°` —— 峰值最小，但转角常年顶到饱和
    - MPC：`0.029` / `0.870` / `35.0°` —— 均值最优，计算量最大
  - PP 误差峰值高，因为它只盯一个前视点，转弯天然"切内弯"
  - LQR 的最大前轮转角**每一组都精确等于 35.000°** —— 这不是优点，是它没有转角约束，只能靠饱和兜底
  - MPC 把将来 `0.8 s`（`8 步 × 0.1 s`）的误差写进代价函数，转弯提前打方向
  - 一句话（金黄条）：**MPC 精度最好，LQR 次之但转角无余量，PP 最粗糙但最便宜**
  - 右图：`fig6_algorithms_map.png`

#### Slide 07 - 两条单调规律

- **Layout**: Top-bottom split — 上方速度表 + 下方双图
- **Title**: 速度 ↑ 误差 ↑；前视距离 ↑ 抖动 ↓
- **Visualization**: basic_table (see §VII)
- **Content**:
  - 表（曲率限速，PP，只改目标速度）：| 速度 | 误差均值 | 误差峰值 | 峰值侧向加速度 |
    - `3 m/s`：`0.046` / `1.417` / `1.049`
    - `5 m/s`：`0.051` / `1.593` / `2.811`
    - `7 m/s`：`0.060` / `1.643` / `3.908`
  - **两项都单调递增**，与问题一的 `aₙ = v²κ` 推导一致
  - `L_f` 取舍（18 次扫描）：`L_f` ↑ → 误差峰值 ↑（@8 m/s：`1.057 → 4.117 m`）；`L_f` ↑ → 转向抖动 ↓（@3 m/s 转向速率 P95：`23.8 → 1.8 °/s`；转向反向 `463 → 1` 次）
  - 机理：`L_f` 小 → 增益 `2L/L_f` 大 → 贴线紧但抖；`L_f` 大 → 平顺但转弯半径被提前切掉
  - 下图：`fig9_error_vs_speed.png`、`fig11_error_vs_lookahead.png`

#### Slide 08 - 结论 ★

- **Layout**: Single column centered, negative-space-driven（无卡片）
- **Title**: 这条路线只有一个真正的难点
- **Visualization**: vertical_list (see §VII)
- **Content**:
  - 主结论（金黄框）：**s = 589 m 处那个转角 87°、半径只有 2.4 m 的急弯，决定了全程的跟踪质量上限。** 16 组对照 + 18 次扫描的误差峰值，无一例外全部落在这一点
  - ① 参考路径的质量比控制算法更关键 —— 不平滑时 LQR 报出 `91.04 m/s²` 侧向加速度，平滑后 `8.61`，**差 10.6 倍**
  - ② 曲率限速把峰值侧向加速度从 `7.97` 压到 `4.54 m/s²`，代价是终点误差多 `0.3 m`（相对 `1450 m` 可忽略）
  - ③ 唯一未达标的一格：**MPC @ 8 m/s 差 0.125 m 没进停车判定窗口**（✗ `#E92E25`）—— 根因是 PP / LQR 都有终点制动项 `√(2·0.9·d)`，MPC 的加速度来自自己的 QP，没有这一项
  - 右图：`fig7_corner_zoom.png`

#### Slide 09 - 结尾

- **Layout**: 继承模板 ending shell（`templates/dongda/svg-flat/slide_25.svg`）
- **Title**: 恳请各位老师批评指正
- **Info**: 汇报人 / 姓名

---

## X. Speaker Notes Requirements

- **Total duration**: 约 4 分 20 秒（P01 20s / P02 40s / P03 35s / P04 50s / P05 30s / P06 40s / P07 30s / P08 45s / P09 10s）
- **Style**: **口语化**（课堂口播稿，允许"我们""说白了""其实"这类自然口语；不用要点式，每页一段连续讲稿；避免书面腔和排比句）
- **Purpose**: report（汇报成果）+ persuade（说服老师结论成立）
- **File naming**: `notes/01_cover.md` … `notes/09_ending.md`，与 SVG 同名
- **Master**: `notes/total.md`，用 `#` 标题行分页（供 `total_md_split.py` 切分）

---

## XI. Technical Constraints Reminder

### SVG Generation Must Follow:

1. viewBox: `0 0 1280 720`
2. Background uses `<rect>` elements
3. Text wrapping uses `<tspan>` (`<foreignObject>` FORBIDDEN)
4. Transparency uses `fill-opacity` / `stroke-opacity`; `rgba()` FORBIDDEN
5. FORBIDDEN: `mask`, `<style>`, `class`, `foreignObject`, `textPath`, `animate*`, `script`
6. Text characters: raw Unicode only (—, →, ×, ≈, ≤, √, κ, δ, Δ, ①, ✗, ✓); HTML named entities FORBIDDEN. XML reserved chars escaped as `&amp;` `&lt;` `&gt;`
7. `marker-start` / `marker-end` allowed only with `<marker>` in `<defs>`, `orient="auto"`, triangle/diamond/circle shape
8. `clipPath` allowed **only on `<image>`** — single shape child in `<defs>`; never on shapes/groups/text

### PPT Compatibility Rules:

- `<g opacity="...">` FORBIDDEN (group opacity); set opacity on each child element
- Image transparency uses an overlay mask layer (`<rect fill="#FFFFFF" opacity="0.x"/>`)
- Inline styles only; external CSS and `@font-face` FORBIDDEN
- Icons: `<use data-icon="tabler-outline/<name>" stroke-width="2" .../>` — the placeholder is expanded by post-processing
- Embedded images: `<image href="../images/<file>" .../>`, `preserveAspectRatio="xMidYMid meet"` (all 8 are `no-crop`)
