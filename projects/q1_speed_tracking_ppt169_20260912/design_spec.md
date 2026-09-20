# q1_speed_tracking_ppt169_20260912 - Design Spec

> Human-readable design narrative — rationale, audience, style, color choices, content outline. Read once by downstream roles for context.
>
> Machine-readable execution contract: `spec_lock.md`. Executor re-reads `spec_lock.md` before every SVG page. On divergence, `spec_lock.md` wins.

## I. Project Information

| Item | Value |
| ---- | ----- |
| **Project Name** | q1_speed_tracking_ppt169_20260912 |
| **Canvas Format** | PPT 16:9 (1280×720) |
| **Page Count** | 8 |
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

> **模板配色校准（重要）**：`docs/东大模板6.pptx` 的 theme XML 把 accent1 写成东大红 `#E92E25`，但把 25 张幻灯片全部导出为 SVG 后统计像素用色，实际主色是**深蓝 `#17375E` / `#002060` + 金黄 `#FFC000` + 正文深灰 `#404040`**；东大红只在目录页（slide_02）作为强调色出现。本 deck 采用模板**实际**的视觉语言：深蓝负责结构、金黄负责强调、深灰负责正文，**东大红降级为"否定 / 警告"专用色**（用于 ✗ 与被推翻的假设）。这样既忠实于模板，又让红色恢复了功能性——红色只标记"错的"，不参与装饰。

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
| **Warning** | `#E92E25` | 被推翻的假设、陷阱、平台缺陷（✗ ⚠） |

### Gradient Scheme

本 deck 不使用渐变。模板的层次感来自**旋转梯形多边形的叠压 + 投影**，Executor 应沿用这一手法（见 §V），而不是用渐变模拟。

---

## IV. Typography System

### Font Plan

**Typography direction**: 模板原生路线——Century Gothic 负责拉丁文与数字（公式、单位、`Part N.` 的几何感），微软雅黑负责中文；全片只用这一组搭配。

> Century Gothic 随 Microsoft Office 安装，属 Office 预装字体；本机已有 Office（模板即为 .pptx），故可安全置于栈首。栈尾仍以通用 `sans-serif` 兜底。

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

> 标题栈以 Century Gothic 领先，使拉丁字母与数字（`R² = 0.99881`、`aₙ = v²κ`、页码）获得模板的几何字感，中文自动下沉到微软雅黑。公式与变量单独走 Code 栈（Consolas 覆盖希腊字母 β κ ψ̇），避免公式被比例字体撑得参差。

**Concord / contrast 备选**：本 deck 采用**对比组合**（几何无衬线标题 × 中文字黑正文）。若后续希望改为**和谐组合**（全片同一族，仅靠字重区分），把 Title 栈换成 `"Microsoft YaHei", "PingFang SC", sans-serif` + weight 700 即可，其余不变。

### Font Size Hierarchy

**Baseline**: Body font size = 20px（内容偏精简，但含公式与数据表，取中等密度）

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

### Layout Pattern Library (combine or break as content demands)

| Pattern | 用在 |
| ------- | ---- |
| **Full-bleed + floating text** | P01 封面、P08 结尾（继承模板 cover shell） |
| **Single column centered** | P07 结论与建议（breathing，靠留白与分隔线组织） |
| **Asymmetric split (4:6)** | P03 / P04 / P05 / P06 —— 左列结论与公式，右列数据图 |
| **Top-bottom split** | P03（上方设置条 + 下方数据表 + 右侧双图） |
| **Numbered vertical list** | P02、P07 |
| **Bilateral list** | P06 |
| **Negative-space-driven** | P07 |

> **统一手法（模板视觉语言）**：装饰不用渐变与卡片阴影，而用**旋转梯形多边形叠压**（`#17375E` → `#FFC000` → `#002060`，配 `feDropShadow`）。内容页只在右上角放一组小尺寸多边形做记号，不抢占正文区。这一手法直接取自模板封面与过渡页。

### Spacing Specification

**Universal**

| Element | Recommended Range | Current Project |
| ------- | ---------------- | --------------- |
| Safe margin from canvas edge | 40-60px | 左右 80px / 上 56px / 下 48px |
| Content block gap | 24-40px | 32px |
| Icon-text gap | 8-16px | 12px |

**Non-card containers**（P02 / P06 / P07 为无卡片页）

- 纵向节奏由**留白 + 1px `#D9D9D9` 分隔线**承担，块间距 36–44px
- **Line-height**: 1.5× body = 30px
- 公式条独占一行，底 `#F2F2F2` 圆角 6px、内边距 10/16
- 编号（① ② ③）用 34px `#FFC000` Century Gothic，与正文 20px 形成字号落差

**Table containers**（P03 / P05）

- 表头行底 `#17375E`、文字 `#FFFFFF` 18px；数据行 20px；斑马纹 `#F2F2F2`
- 单元格内边距 12/16；行高 46px；表底线 1px `#D9D9D9`
- 关键数字用 `#FFC000` 或 `#17375E` Bold，不加粗整行

---

## VI. Icon Usage Specification

### Source

- **Built-in icon library**: `templates/icons/tabler-outline`（线性风格，与本 deck 的克制调性一致）
- **Usage method**: SVG placeholder `<use data-icon="tabler-outline/<name>" .../>`
- **stroke_width**: `2` —— 全片每一个 `<use data-icon>` 都必须带 `stroke-width="2"`

### Recommended Icon List

| Purpose | Icon Path | Page |
| ------- | --------- | ---- |
| 实验设置 / 量测条件 | `tabler-outline/ruler-measure` | P03 |
| 圆周运动与速度量 | `tabler-outline/gauge` | P03 |
| 轨迹跟踪 | `tabler-outline/route-2` | P04 |
| 几何缺陷 / 公式机理 | `tabler-outline/math-function` | P04 |
| 误差上升趋势 | `tabler-outline/trending-up` | P05 |
| 陷阱 / 统计口径 | `tabler-outline/alert-triangle` | P05 |
| 被推翻的假设 | `tabler-outline/circle-x` | P06 |
| 结论 / 主要矛盾 | `tabler-outline/target` | P02, P07 |
| 工程建议 | `tabler-outline/bulb` | P07 |
| 已验证 | `tabler-outline/circle-check` | P03, P06 |

> 以上 10 个图标名均已在 `templates/icons/tabler-outline/` 下用 `ls | grep` 逐个核实存在。Executor 只能使用此清单内的图标；清单外一律不用图标（本 deck 图标本就极少，P01/P08 无图标）。

---

## VII. Visualization Reference List

**Read-audit** (mandatory):

```
Catalog read: 70 templates / 10 categories

Per-page selection (one row per viz page):
  P02 vertical_list     | summary-quote: "Pick for 3-6 numbered key points each with a short description."
  P03 basic_table       | summary-quote: "Pick for plain tabular text/number grid, 3-8 columns."
  P05 dumbbell_chart    | summary-quote: "Pick for before-vs-after or two-state difference across 5-10 items."
  P06 pros_cons_chart   | summary-quote: "Pick for bilateral pros/cons list, 2-5 items per side."
  P07 vertical_list     | summary-quote: "Pick for 3-6 numbered key points each with a short description."

Runners-up considered (5 entries, drawn from real second-best matches in this deck):
  numbered_steps   | rejected for P02: the deck's own headline is "三件事同时发生" — the three causes are simultaneous, not an ordered sequence; numbered_steps' summary reserves it for "3-6 horizontal sequential steps with numeric emphasis" and its connector-arrow idiom would imply a causal order the data does not support.
  icon_grid        | rejected for P07: only 3 recommendations, below icon_grid's "4-9 parallel features/capabilities" band, and each carries a two-clause engineering action rather than a short capability label.
  fishbone_diagram | rejected for P06: Ishikawa suits one effect with 4-6 cause branches; P06 is three independent hypotheses each individually falsified by its own evidence counter, which reads as three separate claims, not one spine with branches.
  consulting_table | rejected for P03: its summary gates on "high-density tables with embedded micro bar visuals"; P03's cells are plain numbers and formulas with no embedded-bar requirement, so basic_table is the honest choice.
  comparison_table | rejected for P06: it needs "2-4 plans/products compared across many feature rows" — P06 has three claims against their counter-evidence, which is a bilateral 3-vs-3 list, not a feature matrix.
```

| Visualization Type | Reference Template | Used In |
| ------------------ | ------------------ | ------- |
| vertical_list | `templates/charts/vertical_list.svg` | P02, P07 |
| basic_table | `templates/charts/basic_table.svg` | P03 |
| dumbbell_chart | `templates/charts/dumbbell_chart.svg` | P05 |
| pros_cons_chart | `templates/charts/pros_cons_chart.svg` | P06 |

> **与内嵌图的分工**：P03/P04/P05/P06 各自还内嵌 1–2 张 matplotlib 出图的 PNG（见 §VIII）。这些 PNG 是**证据**，上述图表模板提供的是**结论的骨架**（编号列表、对照表、双边清单）。Executor 不得用图表模板重画 PNG 里已有的数据曲线——两者是"结论条 + 证据图"的关系，不是重复。
>
> **P05 特例**：`dumbbell_chart` 只承载 low→high 的三行差值（0.446→0.525 / 0.342→0.476 / 0.226→0.348），作为页面顶部的紧凑摘要条；`fig04` PNG 作为其下方的大图细节。二者量纲一致但粒度不同（摘要 vs 全速度曲线），是常见的 "summary strip + detail chart" 组合，不构成重复。

---

## VIII. Image Resource List

全部为**已有图片**（用户选择"直接嵌入现有 PNG"，不做 AI 重新生成，也不做矢量重绘）。来源目录 `docs/figures/`，已复制到本项目 `images/`。

| Filename | Dimensions | Ratio | Purpose | Type | Acquire Via | Status | Reference |
| -------- | --------- | ----- | ------- | ---- | ----------- | ------ | --------- |
| `fig01_steer_vs_v.png` | 见原图 | ~1.6 | P03 稳态转角 vs 速度（水平线 = 与速度无关） | Diagram | user | Existing | 证据图：稳态转角与速度无关 |
| `fig03_an_vs_v.png` | 见原图 | ~1.6 | P03 法向加速度 vs 速度（贴合 v²κ 理论线） | Diagram | user | Existing | 证据图：法向加速度平方增长 |
| `fig06_trajectory_pp_low.png` | 见原图 | ~1.6 | P04 PP 低速轨迹内切 0.437 m（放大视图） | Diagram | user | Existing | 证据图：同心偏心圆 |
| `fig11_cut_regression.png` | 见原图 | ~1.6 | P04 主回归 cut = 0.93·L_f·β，R² = 0.99881 | Diagram | user | Existing | 证据图：主因回归 |
| `fig04_lateral_max_vs_v.png` | 见原图 | ~1.6 | P05 稳态最大误差 vs 速度（三算法单调上升） | Diagram | user | Existing | 证据图：难度随速度上升 |
| `fig17_pp_high_lateral_error.png` | 见原图 | ~1.6 | P05 PP 高速误差曲线：平台，不是尖峰 | Diagram | user | Existing | 证据图：误差形态 |
| `fig13_steer_raw_vs_limited.png` | 见原图 | ~1.6 | P06 原始锯齿 vs 加真实速率限制后（0.61 → 0.21 rad） | Diagram | user | Existing | 证据图：平台缺陷 |

> **全部 7 张标 `no-crop`**：这些是实验数据图，含坐标轴刻度与图例，裁切会丢信息。Executor 必须按原图比例定容器并 `preserveAspectRatio="xMidYMid meet"`。
>
> **注意**：Executor **不得**用 Read 打开这些 PNG 查看内容（skill 硬性规定）。图片信息一律以本表与 `spec_lock.md images` 为准。

---

## IX. Content Outline

### Part 1: 结论与证据

#### Slide 01 - 封面

- **Layout**: 继承模板 cover shell（`templates/dongda/svg-flat/slide_01.svg`）
- **Title**: 为什么速度越快，路径跟踪越难
- **Subtitle**: 问题一：理论分析与实验验证
- **Info**: 东南大学 / 汇报人 · 指导教师（沿用模板的"汇报人 / ___""指导教师 / ___"两栏，姓名留空待填）

#### Slide 02 - 结论先行：三条原因 ★

- **Layout**: Single column, numbered vertical list
- **Title**: 速度越快越难，因为三件事同时发生
- **Visualization**: vertical_list (see §VII)
- **Content**:
  - ① 转弯的侧向加速度按速度平方增长 —— 物理硬约束
    `aₙ = v²κ`；3 → 7 m/s 需求从 `0.75` 涨到 `4.08 m/s²`（×5.44）
  - ② 前视距离随速度变长，而 PP 的稳态内切量正比于前视距离 —— **本文主因**
    `e_ss ≈ 0.93 · (L₀ + 0.35v) · β`，`R² = 0.99881`
  - ③ 每拍前进距离变长，可用修正次数按 `1/v` 减少 —— 时间代价
    每拍 `0.3 m → 0.7 m`
  - 底注（金黄横线之上）：**不是"转向不够用"** —— 稳态转角与速度无关，高速只用到限幅的 38%

#### Slide 03 - 实验设置与三个速度量

- **Layout**: Top-bottom split — 上方设置条 + 下方左表右图
- **Title**: 实验设置与三个速度量
- **Visualization**: basic_table (see §VII)
- **Content**:
  - 设置条：`vdm_lab` 运动学自行车模型 · `dt = 0.1 s` · 轴距 `L = 2.50 m` · 转角限幅 `±35°` · 速率限幅 `45°/s` · 圆周 `R = 12.0 m`
  - 规模：3 算法 × 3 速度档 = 9 组，另加 19 组受控实验
  - 表：| 量 | 3 m/s | 7 m/s | 倍数 | 规律 |
    - 稳态转角 δ | 0.2126 | 0.2149 rad | 几乎不变 | `δ ≈ arctan(Lκ)`
    - 横摆角速度 ψ̇ | 0.2575 | 0.5270 rad/s | ×2.05 | `ψ̇ = vκ`
    - 法向加速度 aₙ | 0.750 | 3.172 m/s² | ×4.23 | `aₙ = v²κ`
  - 脚注：表中为全程均值；峰值 `aₙ` 与 `v²κ` 理论相对误差 ≤ 3.42%
  - 右图：`fig01_steer_vs_v.png`、`fig03_an_vs_v.png`

#### Slide 04 - 主因：内切量 ≈ 0.93 · L_f · β ★

- **Layout**: Asymmetric split (4:6) — 左列结论与公式，右列两张证据图
- **Title**: 主因：PP 稳定跑在一个同心的、半径更小的圆上
- **Content**:
  - 现象：3 m/s 时内切 `0.437 m`，而理论转角只有 `0.205 rad` —— 转向远未饱和
  - 13 组受控实验：`corner_cut ≈ 0.9302 · (L_f · β) − 0.0280 m`；`R² = 0.99881`，最大残差 `0.0227 m`
  - 决定性对照：把 `lf/lr` 改成 `(2.5, 0)` 使 `β = 0` → 内切量 `−0.0423 m`，几乎为零
  - 机理链：PP 转角公式假设无侧偏，车辆实际沿 `ψ+β` 运动；`v ↑ → L_f = L₀ + 0.35v ↑ → 内切 ↑`
  - 右图：`fig06_trajectory_pp_low.png`、`fig11_cut_regression.png`

#### Slide 05 - 误差随速度上升（附一个陷阱）

- **Layout**: Asymmetric split (4:6) — 左列摘要条 + 陷阱警示，右列两张证据图
- **Title**: 稳态最大横向误差三算法全部单调上升
- **Visualization**: dumbbell_chart (see §VII)
- **Content**:
  - 摘要条（dumbbell）：PP `0.446 → 0.525`；LQR k `0.342 → 0.476`；MPC `0.226 → 0.348 m`
  - 精度排序：`MPC > LQR > PP`
  - 陷阱（⚠ `#E92E25`）：**全程平均误差反而随速度下降**（PP `0.286 → 0.275`）——高速下直线段（误差恒为 0）占的采样点更多，把均值稀释了
  - 结论：判断难度必须看**稳态段和最大值**，不能看全程平均
  - 右图：`fig04_lateral_max_vs_v.png`、`fig17_pp_high_lateral_error.png`

#### Slide 06 - 三个"想当然"被数据推翻 ★

- **Layout**: Bilateral list — 左"想当然"，右"数据结论"
- **Title**: 三个"想当然"被数据推翻
- **Visualization**: pros_cons_chart (see §VII)
- **Content**:
  - Q1 高速时转向不够用？ ✗ 稳态转角与速度无关，高速只用掉限幅的 38%
  - Q2 误差随速度单调累积？ ✗ 全程平均误差不增反降（统计口径陷阱）
  - Q3 高速下 LQR 闭环失稳，需要增益调度？ ✗ 极点始终在单位圆内：`max|λ| 0.844 (v=3) → 0.706 (v=7)`
  - Q3 展开（金黄框）：真相是**平台缺陷** —— 控制器命令了 `9 rad/s` 的转角跳变（车辆上限 `0.7854 rad/s`），**超标 11 倍**；加上真实速率限制后 `J_delta 5.49 → 0.47`，精度几乎不变
  - 结论：该振荡是测试平台的伪现象，不是算法问题；固定 K 的跨速度裕度足够，**不需要增益调度**
  - 配图：`fig13_steer_raw_vs_limited.png`

#### Slide 07 - 结论与工程建议

- **Layout**: Single column centered, negative-space-driven（无卡片）
- **Title**: 结论与工程建议
- **Visualization**: vertical_list (see §VII)
- **Content**:
  - 结论：速度越快越难 = **平方级物理代价 + 线性级几何代价 + 1/v 时间代价**；主要矛盾是 PP 的稳态内切量正比于前视距离
  - ① 前视距离不能盲目随速度增大 → 引入曲率前馈或转角偏置补偿
  - ② 终点接近限速必须用**沿路径剩余弧长**，而非直线距离
  - ③ 仿真平台必须在执行器层施加 `max_steer_rate`，否则平滑性指标不可信

#### Slide 08 - 结尾

- **Layout**: 继承模板 ending shell（`templates/dongda/svg-flat/slide_25.svg`）
- **Title**: 恳请各位老师批评指正
- **Info**: 汇报人 / 姓名

---

## X. Speaker Notes Requirements

- **Total duration**: 约 4 分 05 秒（P01 25s / P02 50s / P03 50s / P04 60s / P05 40s / P06 65s / P07 35s / P08 10s）
- **Style**: 正式但口语化（课堂答辩口播稿），每页一段连续讲稿，不用要点式
- **Purpose**: report（汇报成果）+ persuade（说服老师结论成立）
- **File naming**: `notes/01_cover.md` … `notes/08_ending.md`，与 SVG 同名
- **Master**: `notes/total.md`，用 `#` 标题行分页（供 `total_md_split.py` 切分）
- **素材来源**: `sources/问题1_PPT内容与演讲稿.md` 的逐页演讲稿，原样搬运，不改写

---

## XI. Technical Constraints Reminder

### SVG Generation Must Follow:

1. viewBox: `0 0 1280 720`
2. Background uses `<rect>` elements
3. Text wrapping uses `<tspan>` (`<foreignObject>` FORBIDDEN)
4. Transparency uses `fill-opacity` / `stroke-opacity`; `rgba()` FORBIDDEN
5. FORBIDDEN: `mask`, `<style>`, `class`, `foreignObject`, `textPath`, `animate*`, `script`
6. Text characters: raw Unicode only (—, →, ×, ≈, ≤, β, κ, ψ̇, ①, ✗, ✓, R²); HTML named entities FORBIDDEN. XML reserved chars escaped as `&amp;` `&lt;` `&gt;`
7. `marker-start` / `marker-end` allowed only with `<marker>` in `<defs>`, `orient="auto"`, triangle/diamond/circle shape
8. `clipPath` allowed **only on `<image>`** — single shape child in `<defs>`; never on shapes/groups/text

### PPT Compatibility Rules:

- `<g opacity="...">` FORBIDDEN (group opacity); set opacity on each child element
- Image transparency uses an overlay mask layer (`<rect fill="#FFFFFF" opacity="0.x"/>`)
- Inline styles only; external CSS and `@font-face` FORBIDDEN
- Icons: `<use data-icon="tabler-outline/<name>" stroke-width="2" .../>` — the placeholder is expanded by post-processing
- Embedded PNGs: `<image href="../images/<file>" .../>`, `preserveAspectRatio="xMidYMid meet"` (all 7 are `no-crop`)
