"""Build the NullPointer VDM experiment report as a visually verified DOCX."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION_START
from docx.enum.table import WD_ALIGN_VERTICAL
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[1]
INK = "000000"
TEAL = "4F6865"
PALE_TEAL = "E9F0EE"
PALE_WARM = "F7EEE8"
GRAY_BORDER = "D9D9D9"


def set_font(run, size=10.5, bold=False, color=INK, name="Microsoft YaHei"):
    run.font.name = name
    run._element.rPr.rFonts.set(qn("w:eastAsia"), name)
    run._element.rPr.rFonts.set(qn("w:ascii"), "Arial")
    run._element.rPr.rFonts.set(qn("w:hAnsi"), "Arial")
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = RGBColor.from_string(color)


def shade(cell, fill):
    props = cell._tc.get_or_add_tcPr()
    node = OxmlElement("w:shd")
    node.set(qn("w:fill"), fill)
    props.append(node)


def set_cell_border(cell, color=GRAY_BORDER):
    tc_pr = cell._tc.get_or_add_tcPr()
    borders = tc_pr.first_child_found_in("w:tcBorders")
    if borders is None:
        borders = OxmlElement("w:tcBorders")
        tc_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        tag = qn(f"w:{edge}")
        element = borders.find(tag)
        if element is None:
            element = OxmlElement(f"w:{edge}")
            borders.append(element)
        element.set(qn("w:val"), "single")
        element.set(qn("w:sz"), "6")
        element.set(qn("w:color"), color)


def mark_table_header(row):
    props = row._tr.get_or_add_trPr()
    node = OxmlElement("w:tblHeader")
    node.set(qn("w:val"), "true")
    props.append(node)


def add_page_field(paragraph):
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = "PAGE"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.append(begin)
    run._r.append(instr)
    run._r.append(end)


def add_paragraph(doc, text="", *, bold_lead=None, align=None, size=10.5, space_after=6):
    p = doc.add_paragraph()
    if align is not None:
        p.alignment = align
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.line_spacing = 1.35
    if bold_lead and text.startswith(bold_lead):
        lead = p.add_run(bold_lead)
        set_font(lead, size=size, bold=True)
        rest = p.add_run(text[len(bold_lead):])
        set_font(rest, size=size)
    else:
        run = p.add_run(text)
        set_font(run, size=size)
    return p


def add_heading(doc, text, level=1):
    p = doc.add_paragraph(style=f"Heading {level}")
    p.paragraph_format.space_before = Pt(12 if level == 1 else 8)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run(text)
    set_font(run, size=14 if level == 1 else 11.5, bold=True)
    return p


def add_caption(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(8)
    run = p.add_run(text)
    set_font(run, size=8.5, color="555555")


# Native Office Math Markup Language (OMML) helpers.  These map the report's
# LaTeX-style expressions to editable Word equations instead of ordinary text.
def m_element(name):
    return OxmlElement(f"m:{name}")


def m_run(text):
    run = m_element("r")
    props = m_element("rPr")
    font = m_element("scr")
    font.set(qn("m:val"), "Cambria Math")
    props.append(font)
    run.append(props)
    node = m_element("t")
    node.text = text
    run.append(node)
    return run


def m_group(*parts):
    group = m_element("e")
    for part in parts:
        group.append(part)
    return group


def m_sub(base, subscript):
    node = m_element("sSub")
    base_node = m_element("e")
    base_node.append(m_run(base))
    sub_node = m_element("sub")
    sub_node.append(m_run(subscript))
    node.extend((base_node, sub_node))
    return node


def m_sup(base, superscript):
    node = m_element("sSup")
    base_node = m_element("e")
    base_node.append(m_run(base))
    sup_node = m_element("sup")
    sup_node.append(m_run(superscript))
    node.extend((base_node, sup_node))
    return node


def m_frac(numerator, denominator):
    node = m_element("f")
    num = m_element("num")
    num.append(m_group(*numerator))
    den = m_element("den")
    den.append(m_group(*denominator))
    node.extend((num, den))
    return node


def add_equation(doc, *parts):
    """Add one centered, editable Word equation assembled from OMML parts."""
    paragraph = doc.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_after = Pt(4)
    paragraph.paragraph_format.line_spacing = 1.15
    math_para = m_element("oMathPara")
    math = m_element("oMath")
    for part in parts:
        math.append(part)
    math_para.append(math)
    paragraph._p.append(math_para)
    return paragraph


def add_table(doc, headers, rows, widths=None, font_size=7.5):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    table.autofit = False
    header = table.rows[0]
    mark_table_header(header)
    for index, label in enumerate(headers):
        cell = header.cells[index]
        cell.text = str(label)
        shade(cell, TEAL)
        set_cell_border(cell)
        cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
        paragraph = cell.paragraphs[0]
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        for run in paragraph.runs:
            set_font(run, size=font_size, bold=True, color="FFFFFF")
        if widths:
            cell.width = Cm(widths[index])
    for row_index, values in enumerate(rows):
        cells = table.add_row().cells
        fill = PALE_TEAL if row_index % 2 == 0 else "FFFFFF"
        for index, value in enumerate(values):
            cell = cells[index]
            cell.text = str(value)
            shade(cell, fill)
            set_cell_border(cell)
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            paragraph = cell.paragraphs[0]
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER if index != 0 else WD_ALIGN_PARAGRAPH.LEFT
            for run in paragraph.runs:
                set_font(run, size=font_size)
            if widths:
                cell.width = Cm(widths[index])
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return table


def image(doc, path, width_cm, caption):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    shape = p.add_run().add_picture(str(path), width=Cm(width_cm))
    shape._inline.docPr.set("descr", caption)
    shape._inline.docPr.set("title", caption)
    add_caption(doc, caption)


def algorithm_label(key):
    return {"pp": "PP", "lqr_kinematic": "运动学 LQR", "lqr_dynamic": "动力学 LQR", "mpc": "执行器感知 MPC"}.get(key, key)


def speed_label(key):
    return {"low": "低速", "medium": "中速", "high": "高速"}.get(key, key)


def fmt(value, digits=3):
    return f"{float(value):.{digits}f}"


def collect_gpx_results():
    """Read completed GPX runs created by run_experiment.py, if all exist."""
    results = []
    for algorithm in ("pp", "lqr_kinematic", "mpc"):
        candidates = sorted(
            (ROOT / "outputs").glob(f"*_{algorithm}_homework_route_1_low"),
            key=lambda item: item.stat().st_mtime,
        )
        if not candidates:
            continue
        output_dir = candidates[-1]
        metrics_path = output_dir / "metrics.json"
        trajectory_path = output_dir / "trajectory.csv"
        reference_path = output_dir / "reference_path.csv"
        if not (metrics_path.exists() and trajectory_path.exists() and reference_path.exists()):
            continue
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        with trajectory_path.open(encoding="utf-8") as handle:
            trajectory = list(csv.DictReader(handle))
        with reference_path.open(encoding="utf-8") as handle:
            reference = list(csv.DictReader(handle))
        if not trajectory or not reference:
            continue
        peak = max(trajectory, key=lambda row: abs(float(row["lateral_error"])))
        target_index = min(int(peak["target_index"]), len(reference) - 1)
        duration = float(trajectory[-1]["time"]) - float(trajectory[0]["time"])
        results.append(
            {
                "algorithm": algorithm,
                "reached_goal": metrics["reached_goal"],
                "route_length_m": float(reference[-1]["s_m"]),
                "duration_s": duration,
                "mean_lateral_error_m": metrics["mean_lateral_error_m"],
                "max_lateral_error_m": metrics["max_lateral_error_m"],
                "peak_time_s": float(peak["time"]),
                "peak_lon_lat": f"{reference[target_index]['lon_deg']}, {reference[target_index]['lat_deg']}",
                "peak_curvature": float(peak["curvature"]),
                "peak_steer": float(peak["steer"]),
                "output_dir": str(output_dir),
            }
        )
    return results


def collect_navigation_completion():
    """Read the verified Amap-route completion experiment when available."""
    output_dir = ROOT / "outputs" / "nullpointer_completion"
    metrics_path = output_dir / "navigation_sweep_metrics.csv"
    metadata_path = output_dir / "navigation_metadata.json"
    if not (metrics_path.exists() and metadata_path.exists()):
        raise FileNotFoundError("缺少寝室—教学楼补齐实验数据，请先运行 run_nullpointer_completion.py")
    with metrics_path.open(encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key in (
            "level", "route_length_m", "duration_s", "mean_abs_lateral_error_m",
            "max_abs_lateral_error_m", "mean_abs_steer_rate_radps", "max_abs_steer_rad",
            "peak_time_s", "peak_x_m", "peak_y_m", "peak_lateral_error_m",
        ):
            row[key] = float(row[key])
        row["reached_goal"] = row["reached_goal"].strip().lower() == "true"
    return rows, json.loads(metadata_path.read_text(encoding="utf-8")), output_dir


def prepare_document():
    doc = Document()
    section = doc.sections[0]
    section.top_margin = Cm(2.2)
    section.bottom_margin = Cm(2.0)
    section.left_margin = Cm(2.25)
    section.right_margin = Cm(2.25)
    styles = doc.styles
    styles["Normal"].font.name = "Microsoft YaHei"
    styles["Normal"]._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    styles["Normal"].font.size = Pt(10.5)
    for name in ("Title", "Heading 1", "Heading 2"):
        styles[name].font.color.rgb = RGBColor(0, 0, 0)
        styles[name]._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = footer.add_run("NullPointer 车辆路径跟踪控制实验报告  第 ")
    set_font(r, size=8, color="555555")
    add_page_field(footer)
    r = footer.add_run(" 页")
    set_font(r, size=8, color="555555")
    return doc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=ROOT / "outputs/nullpointer_report_data/report_data.json")
    parser.add_argument("--output", type=Path, default=ROOT / "deliverables/NullPointer_VDM_实验报告.docx")
    args = parser.parse_args()
    data = json.loads(args.data.read_text(encoding="utf-8"))
    gpx_results = collect_gpx_results()
    navigation_rows, navigation_metadata, navigation_dir = collect_navigation_completion()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    doc = prepare_document()

    # Cover.
    doc.add_paragraph().paragraph_format.space_after = Pt(42)
    title = doc.add_paragraph(style="Title")
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = title.add_run("车辆路径跟踪控制实验报告")
    set_font(run, size=24, bold=True)
    subtitle = doc.add_paragraph()
    subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = subtitle.add_run("Pure Pursuit  线性二次型调节器  线性模型预测控制")
    set_font(run, size=13, color="333333")
    doc.add_paragraph().paragraph_format.space_after = Pt(42)
    cover_table = doc.add_table(rows=3, cols=2)
    cover_table.autofit = False
    mark_table_header(cover_table.rows[0])
    for row, (label, value) in zip(cover_table.rows, (("课程实验", "自行车模型与路径跟踪"), ("小组名称", data["group_name"]), ("完成日期", "2026 年 9 月"))):
        for cell, text, fill in ((row.cells[0], label, PALE_WARM), (row.cells[1], value, "FFFFFF")):
            cell.text = text
            shade(cell, fill)
            set_cell_border(cell)
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
            for r in cell.paragraphs[0].runs:
                set_font(r, size=11, bold=(cell == row.cells[0]))
    doc.add_paragraph().paragraph_format.space_after = Pt(24)
    add_paragraph(doc, "报告结论：在 868 m 寝室—教学楼导航路线中，四类控制器均到达终点；速度升高时，运动学 LQR 出现转向饱和与控制激烈度显著上升，动力学 LQR 保持更稳定的误差水平。", align=WD_ALIGN_PARAGRAPH.CENTER, size=10.5)
    doc.add_page_break()

    add_heading(doc, "摘要", 1)
    add_paragraph(doc, "本实验围绕自行车模型路径跟踪问题，实现并比较 Pure Pursuit、运动学 LQR、动力学 LQR 与线性 MPC。除双移线、直角弯、S 弯和半径 12 m 的圆形路径外，报告根据高德步行导航截图重建了寝室至教学楼的 868 m 本地坐标参考路径。以速度、最大转角和轴距为三个独立变量开展敏感性实验，并在相同路线、相同采样周期下比较运动学与动力学 LQR。进一步针对传统 MPC 首步控制量可能突变的问题，提出执行器感知转角变化率约束。结果显示：高速会放大运动学 LQR 的转向饱和与控制激烈度；动力学 LQR 在所测速度范围内保持更低的跟踪误差。")
    add_paragraph(doc, "关键词：自行车模型；路径跟踪；导航参考路径；Pure Pursuit；LQR；模型预测控制；转角变化率约束", size=9.5)

    add_heading(doc, "1 实验目标与分工", 1)
    add_paragraph(doc, "本报告完成三项课程任务，并补齐综合导航验证：比较 PP、运动学 LQR、动力学 LQR 与 MPC 在不同路径几何下的表现；在固定曲率圆形路径中验证速度、横摆角速度和法向加速度关系；根据高德导航截图重建寝室至教学楼的本地参考路径，完成地图依据、路径生成、PP 跟踪、峰值误差定位和实验数据分析。NullPointer 小组在学生版代码中补齐四类控制器，并在 MPC 中实现执行器感知首步转角变化率约束。")
    add_table(doc, ["工作项", "实现或验证内容"], [
        ["控制器实现", "PP、运动学 LQR、动力学 LQR、线性 MPC 的学生版 TODO 全部完成"],
        ["公平性设置", "同一路线、同一车辆、同一速度档与同一采样周期下比较算法"],
        ["创新设计", "约束 MPC 第一个实际执行转角与上一帧转角的变化率，并做严格边界投影"],
        ["三变量实验", "速度 3/5/7 m/s、最大转角 5/8/12/20 deg、轴距 2.2/2.5/2.8 m"],
        ["导航任务", "高德截图重建 868 m 参考路径；全控制器到达、PP 峰值误差定位与路线可视化"],
        ["可视化", "路线重建、轨迹、时序和多变量对比图采用莫兰迪配色"],
    ], widths=[3.2, 12.3], font_size=8.5)

    add_heading(doc, "2 理论模型与代码对应", 1)
    add_paragraph(doc, "在后轮不转向的运动学自行车模型中，前轮转角为 δf，侧偏角为 β，车辆质心位置为 (x, y)，航向角为 ψ，速度为 v。其状态更新关系如下：")
    add_equation(doc,
                 m_run("β = arctan("), m_frac([m_sub("l", "r")], [m_run("("), m_sub("l", "f"), m_run(" + "), m_sub("l", "r"), m_run(")")]),
                 m_run(" tan "), m_sub("δ", "f"), m_run(")"))
    add_equation(doc, m_frac([m_run("d"), m_run("x")], [m_run("d"), m_run("t")]), m_run(" = v cos(ψ + β)"))
    add_equation(doc, m_frac([m_run("d"), m_run("y")], [m_run("d"), m_run("t")]), m_run(" = v sin(ψ + β)"))
    add_equation(doc,
                 m_frac([m_run("d"), m_run("ψ")], [m_run("d"), m_run("t")]), m_run(" = "),
                 m_frac([m_run("v")], [m_run("("), m_sub("l", "f"), m_run(" + "), m_sub("l", "r"), m_run(")")]),
                 m_run(" tan "), m_sub("δ", "f"), m_run(" cos β"))
    image(doc, ROOT / "vdm_lab/KMLM.png", 11.2, "图 1 运动学自行车模型及主要几何量")
    add_paragraph(doc, "对半径为 R 的圆周运动，曲率、法向加速度与小侧偏稳态关系如下。本实验圆形路径半径 R = 12 m，默认轴距为 2.5 m。")
    add_equation(doc, m_run("κ = "), m_frac([m_run("1")], [m_run("R")]), m_run(",  "), m_sub("a", "n"), m_run(" = "), m_sup("v", "2"), m_run("κ"))
    add_equation(doc,
                 m_sub("δ", "f"), m_run(" ≈ arctan[("), m_sub("l", "f"), m_run(" + "), m_sub("l", "r"), m_run(")κ],  "),
                 m_frac([m_run("d"), m_run("ψ")], [m_run("d"), m_run("t")]), m_run(" ≈ vκ"))
    image(doc, ROOT / "vdm_lab/exp_cm.png", 10.6, "图 2 圆周运动中的曲率与法向加速度关系")
    add_table(doc, ["课程量", "代码与日志字段", "说明"], [
        ["前轮转角 δf", "ControlCommand.steer / trajectory.csv: steer", "转向控制输入"],
        ["侧偏角 β", "StepRecord.beta / trajectory.csv: beta", "自行车模型侧偏角"],
        ["横摆角速度 ψ_dot", "StepRecord.yaw_rate / trajectory.csv: yaw_rate", "车辆航向变化速度"],
        ["法向加速度 an", "StepRecord.normal_accel", "v²κ"],
    ], widths=[3.1, 7.0, 5.4], font_size=8)

    add_heading(doc, "3 实验设置", 1)
    add_table(doc, ["项目", "设置"], [
        ["车辆", "student_car；轴距 2.5 m；最大转角 35 deg；最大转角速度 45 deg/s"],
        ["仿真", "采样周期 0.1 s；车辆仅前进；速度由路径速度档设定"],
        ["任务 1", "双移线、直角弯、S 弯；统一中速档"],
        ["任务 2", "R = 12 m 圆形路径；低、中、高速分别为 3、5、7 m/s"],
        ["评价指标", "是否到达终点、平均/最大横向误差、最大转角、最大法向加速度、最大横摆角速度、平均绝对转角变化率"],
    ], widths=[3.2, 12.3], font_size=8.5)

    add_heading(doc, "4 任务一 多路径算法对比", 1)
    task1_rows = [[algorithm_label(row["algorithm"]), row["route"], "是" if row["reached_goal"] else "否", fmt(row["mean_lateral_error_m"]), fmt(row["max_lateral_error_m"]), fmt(row["max_steer_rad"]), fmt(row["max_normal_accel_mps2"]), fmt(row["mean_abs_steer_rate_radps"])] for row in data["task1"]]
    add_table(doc, ["算法", "路线", "到达", "平均误差 m", "最大误差 m", "最大转角 rad", "最大法向加速度", "平均 |dδ/dt|"], task1_rows, widths=[2.5, 2.7, 1.1, 2.0, 2.0, 2.0, 2.6, 2.4], font_size=6.8)
    add_paragraph(doc, "结果分析：MPC 在三条路线上的平均横向误差均为最小，其中双移线和直角弯的最大误差均为 0.472 m。PP 对前视距离敏感，在直角弯和 S 弯的最大误差分别为 1.106 m 和 1.171 m。运动学 LQR 在双移线和 S 弯中出现较大的转角变化率，且在双移线中触及 35 deg 转角上限；这说明高反馈增益能够压低误差，但会提高控制激烈程度。")

    add_heading(doc, "5 参数变化实验", 1)
    parameter_rows = [[algorithm_label(row["algorithm"]), f"{row['max_steer_limit_deg']:.0f}", fmt(row["mean_lateral_error_m"]), fmt(row["max_lateral_error_m"]), fmt(row["max_steer_rad"]), fmt(row["mean_abs_steer_rate_radps"])] for row in data["parameter_study"]]
    add_table(doc, ["算法", "转角上限 deg", "平均误差 m", "最大误差 m", "实际最大转角 rad", "平均 |dδ/dt|"], parameter_rows, widths=[3.0, 2.6, 2.5, 2.5, 3.0, 3.0], font_size=8)
    add_paragraph(doc, "当最大转角由 35 deg 降至 25 deg 时，PP 在该中速双移线工况的需求转角未触及新上限，因此指标几乎不变。运动学 LQR 的最大转角由 0.611 rad 降至 0.436 rad，平均转角变化率由 4.366 降至 3.020 rad/s，同时平均与最大横向误差略有下降。本组结果表明，该路线下原始 LQR 的 35 deg 饱和会引入控制突变；适度约束可改善其动态行为。")

    doc.add_page_break()
    add_heading(doc, "6 任务二 圆形路径速度验证", 1)
    theory_rows = [[speed_label(mode), fmt(value["speed_mps"], 1), fmt(value["delta_rad"]), fmt(value["yaw_rate_radps"]), fmt(value["normal_accel_mps2"])] for mode, value in data["theory_circle"].items()]
    add_table(doc, ["速度档", "v m/s", "理论 δf rad", "理论 ψ_dot rad/s", "理论 an m/s²"], theory_rows, widths=[3.0, 2.0, 3.0, 4.0, 3.5], font_size=8.5)
    task2_rows = [[algorithm_label(row["algorithm"]), speed_label(row["speed_mode"]), "是" if row["reached_goal"] else "否", fmt(row["mean_lateral_error_m"]), fmt(row["max_lateral_error_m"]), fmt(row["steady_mean_steer_rad"]), fmt(row["steady_mean_yaw_rate_radps"]), fmt(row["steady_mean_normal_accel_mps2"])] for row in data["task2"]]
    add_table(doc, ["算法", "速度", "到达", "平均误差", "最大误差", "稳态转角", "稳态横摆角速度", "稳态法向加速度"], task2_rows, widths=[2.5, 1.7, 1.0, 2.0, 2.0, 2.1, 3.0, 3.0], font_size=6.8)
    add_paragraph(doc, "验证结果：三种算法和三档速度均到达终点。稳态转角基本保持在 0.20 至 0.215 rad，与理论值 0.205 rad 接近，说明固定曲率下几何所需转角与速度近似无关。MPC 在中速时的稳态横摆角速度为 0.416 rad/s，与理论 0.417 rad/s 基本一致；高速时稳态法向加速度为 4.014 m/s²，接近理论 4.083 m/s²。法向加速度随速度平方增加：从 3 m/s 的约 0.75 m/s² 提升到 7 m/s 的约 4.0 m/s²。")
    add_paragraph(doc, "高速下，运动学 LQR 最大转角达到 0.611 rad 并且平均转角变化率为 4.643 rad/s，明显高于 PP 和 MPC，反映出固定采样周期内修正时间缩短、转向饱和和相位滞后的共同影响。MPC 利用预测与约束，在三种速度下的平均横向误差均最低或接近最低。")

    add_heading(doc, "7 三个关键变量的敏感性实验", 1)
    speed_rows = [row for row in navigation_rows if row["factor"] == "speed"]
    speed_rows.sort(key=lambda row: (row["algorithm"], row["level"]))
    add_paragraph(doc, "速度升高会使单位时间内可用于消除横向和航向误差的距离缩短；对于给定曲率，法向加速度又随速度平方增加。因此，执行器带宽、转角上限和预测距离不足时，控制器更容易出现滞后、饱和或高频修正。该结论是“通常更难”的机理说明，而不是声称每一个控制器在每一条路线上的误差都必然单调增大。")
    speed_table = [[algorithm_label(row["algorithm"]), fmt(row["level"], 0), "是" if row["reached_goal"] else "否", fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["mean_abs_steer_rate_radps"]), fmt(row["max_abs_steer_rad"])] for row in speed_rows]
    add_table(doc, ["算法", "速度 m/s", "到达", "平均误差 m", "最大误差 m", "平均 |dδ/dt|", "最大转角 rad"], speed_table, widths=[2.7, 2.1, 1.2, 2.4, 2.4, 3.0, 2.6], font_size=7.5)
    add_paragraph(doc, "实验中，运动学 LQR 的平均误差由 3 m/s 时的 0.030 m 增至 7 m/s 时的 0.221 m，平均绝对转角变化率由 0.142 增至 10.929 rad/s，并触及 0.611 rad 转角上限。PP 与动力学 LQR 在该平滑路线中仍保持稳定，说明速度本身并非充分条件；路线曲率、模型失配和控制约束共同决定难度。")
    steer_rows = sorted((row for row in navigation_rows if row["factor"] == "max_steer"), key=lambda row: row["level"])
    wheel_rows = sorted((row for row in navigation_rows if row["factor"] == "wheelbase"), key=lambda row: row["level"])
    sensitivity_rows = [["最大前轮转角", fmt(row["level"], 0) + " deg", algorithm_label(row["algorithm"]), fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["max_abs_steer_rad"])] for row in steer_rows]
    sensitivity_rows += [["轴距", fmt(row["level"], 1) + " m", algorithm_label(row["algorithm"]), fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["max_abs_steer_rad"])] for row in wheel_rows]
    add_table(doc, ["变量", "取值", "代表控制器", "平均误差 m", "最大误差 m", "实际最大转角 rad"], sensitivity_rows, widths=[3.0, 2.4, 3.0, 2.7, 2.7, 3.0], font_size=7.5)
    add_paragraph(doc, "最大转角从 20 deg 收紧到 5 deg 时，PP 的平均误差由约 0.049 m 增至 0.201 m，最大误差增至 4.919 m，表明该路线的弯道需要超过 5 deg 的转向能力。轴距在 2.2 至 2.8 m 变化时，动力学 LQR 平均误差保持约 0.010 m，但实际最大转角随轴距增加而增大，符合更长轴距需要更大几何转角的预期。")
    image(doc, navigation_dir / "three_variable_sweeps.png", 15.5, "图 3 速度、最大转角和轴距三个变量的敏感性实验")

    add_heading(doc, "8 车辆模型重建与模型对比", 1)
    add_paragraph(doc, "运动学模型将车辆视为无侧偏的刚体，适合低速或几何关系主导的跟踪；动力学 LQR 以横向误差、横向速度误差、航向误差和横摆角速度误差为状态，并显式使用车辆质量、转动惯量、前后轮侧偏刚度和速度构建时变线性系统：")
    add_equation(doc, m_run("ė = A(v)e + B "), m_sub("δ", "f"))
    add_paragraph(doc, "因此，该模型可反映速度变化对轮胎侧偏与横摆响应的影响。两类 LQR 均使用同一路径、同一速度上限、同一采样周期和同一车辆几何参数。")
    model_rows = [[fmt(row["level"], 0), algorithm_label(row["algorithm"]), "是" if row["reached_goal"] else "否", fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["mean_abs_steer_rate_radps"])] for row in speed_rows if row["algorithm"] in {"lqr_kinematic", "lqr_dynamic"}]
    add_table(doc, ["速度 m/s", "模型控制器", "到达", "平均误差 m", "最大误差 m", "平均 |dδ/dt|"], model_rows, widths=[2.3, 3.4, 1.2, 2.7, 2.7, 3.2], font_size=7.8)
    add_paragraph(doc, "在 3、5、7 m/s 下，动力学 LQR 的平均误差分别为 0.008、0.010、0.013 m，而运动学 LQR 在 7 m/s 时升至 0.221 m 并发生转角饱和。该结果支持在较高速度、曲率连续变化的路线上引入侧偏刚度与横摆动力学；但它来自确定性仿真，不能替代真实轮胎和路面参数的实车标定。")
    image(doc, navigation_dir / "kinematic_dynamic_comparison.png", 15.5, "图 4 运动学与动力学 LQR 的速度敏感性和中速轨迹对比")

    add_heading(doc, "9 创新设计 执行器感知 MPC", 1)
    add_paragraph(doc, "标准线性 MPC 通常约束预测时域中相邻控制量的变化，但首个即将执行的控制量可能与上一帧实际转角不连续。本组对 QP 首步控制量加入如下执行器连续性约束，并在求解后将第一步动作投影到同一边界：")
    add_equation(doc,
                 m_run("|"), m_sub("δ", "0"), m_run(" − "), m_sub("δ", "previous"), m_run("| ≤ "),
                 m_sub("δ̇", "max"), m_run(" Δt"))
    add_paragraph(doc, "该约束直接对应转向执行器的连续性要求。")
    innovation = data["innovation"]
    innovation_rows = [["基准 MPC" if row["condition"] == "baseline_mpc" else "执行器感知 MPC", speed_label(row["speed_mode"]), fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["mean_abs_steer_rate_radps"]), fmt(row["max_abs_steer_rate_radps"])] for row in innovation]
    add_table(doc, ["方案", "速度", "平均误差 m", "最大误差 m", "平均 |dδ/dt|", "最大 |dδ/dt|"], innovation_rows, widths=[3.6, 2.0, 2.3, 2.3, 3.0, 3.0], font_size=7.5)
    image(doc, ROOT / "outputs/mpc_rate_limit_highlight/mpc_rate_limit_dashboard.png", 15.0, "图 5 高速圆形路径下的轨迹与控制时序对比")
    image(doc, ROOT / "outputs/mpc_rate_limit_highlight/mpc_rate_limit_tradeoff.png", 15.0, "图 6 精度与平滑性权衡的多视图比较")
    add_paragraph(doc, "高速工况中，执行器感知 MPC 的平均绝对转角变化率由 0.665 降至 0.357 rad/s，下降 46.2%；最大变化率由 1.735 降至 0.785 rad/s，下降 54.7%。平均横向误差由 0.168 m 降至 0.166 m，最大横向误差仅从 0.348 m 变化到 0.349 m。结果说明所加约束抑制了转向突变，并未显著牺牲跟踪精度。")

    add_heading(doc, "10 任务三 寝室至教学楼导航任务", 1)
    add_paragraph(doc, f"用户提供的高德步行导航图显示，寝室至教学楼路线为 868 m、预计步行 12 分钟。根据截图中的路线折线提取 9 个局部路点，并以连续样条生成 {navigation_metadata['reference_length_m']:.2f} m 的东—北（ENU）参考路径。该路径用于控制仿真，不宣称为高精度 GPS 或真实道路中心线。")
    image(doc, navigation_dir / "amap_route_reconstruction.png", 15.5, "图 7 高德路线依据与 868 m 本地导航参考路径重建")
    navigation_baseline = []
    for algorithm in ("pp", "lqr_kinematic", "lqr_dynamic"):
        navigation_baseline.append(next(row for row in navigation_rows if row["factor"] == "speed" and row["level"] == 5.0 and row["algorithm"] == algorithm))
    navigation_baseline.append(next(row for row in navigation_rows if row["factor"] == "baseline" and row["algorithm"] == "mpc"))
    navigation_table = [[algorithm_label(row["algorithm"]), "是" if row["reached_goal"] else "否", fmt(row["route_length_m"], 2), fmt(row["duration_s"], 1), fmt(row["mean_abs_lateral_error_m"]), fmt(row["max_abs_lateral_error_m"]), fmt(row["peak_time_s"], 1)] for row in navigation_baseline]
    add_table(doc, ["算法", "到达", "参考长度 m", "仿真用时 s", "平均误差 m", "最大误差 m", "最大误差时刻 s"], navigation_table, widths=[2.7, 1.2, 2.6, 2.5, 2.4, 2.4, 2.8], font_size=7.3)
    pp_navigation = next(row for row in navigation_baseline if row["algorithm"] == "pp")
    add_paragraph(doc, f"四种控制器均到达终点。以 PP 为例，最大横向误差为 {pp_navigation['max_abs_lateral_error_m']:.3f} m，发生在 t={pp_navigation['peak_time_s']:.1f} s 的连续转弯段，已在图中以红点和时序虚线定位。5 m/s 的仿真用时约 180 s，不能与高德 12 分钟步行预估直接比较：前者是受路线限速的理想化车辆仿真，后者包含步行速度、过路和实际导航规则。")
    image(doc, navigation_dir / "pp_navigation_tracking.png", 15.5, "图 8 PP 在寝室—教学楼路线上的跟踪轨迹与最大误差定位")

    add_heading(doc, "11 结论", 1)
    add_paragraph(doc, "第一，速度提高会通过法向加速度平方增长、可纠偏时间缩短和执行器饱和放大路径跟踪难度；本次路线中运动学 LQR 的高速结果给出了直接例证。第二，速度、最大转角和轴距三项变量均已进行定量扫描，结果显示转角裕度不足是最明确的性能瓶颈。第三，重建的动力学 LQR 在三档速度下保持更低的误差，说明侧偏和横摆动力学对高速跟踪具有解释价值。第四，基于高德截图重建的 868 m 寝室—教学楼路线已完成参考路径生成、PP 跟踪可视化、峰值误差定位及四类控制器的到达性对比。")
    add_paragraph(doc, "本报告的数值结论来自确定性仿真，不应解释为统计显著性；截图重建路线也不替代测绘级 GPS。实际道路通勤还受到交通规则、其他道路参与者、限速和路径规划质量影响。")

    add_heading(doc, "参考资料", 1)
    add_paragraph(doc, "[1] VehicleDynamicsMobility_01_BicycleModel.pdf，课程讲义。")
    add_paragraph(doc, "[2] vdm_lab/tasks/README.md，课程任务书。")
    add_paragraph(doc, "[3] 项目仿真日志与本报告生成的 comparison_metrics.csv、report_data.json、navigation_sweep_metrics.csv。")

    doc.core_properties.title = "车辆路径跟踪控制实验报告"
    doc.core_properties.author = "NullPointer"
    doc.core_properties.subject = "VDM Path Tracking Lab"
    doc.save(args.output)
    print(f"output={args.output}")


if __name__ == "__main__":
    main()
