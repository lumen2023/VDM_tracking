"""
A-2: PP lookahead 参数敏感性实验。
用法:
    python analysis/run_pp_param.py 1.5
    python analysis/run_pp_param.py 3.0
    python analysis/run_pp_param.py 5.0
"""
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

if len(sys.argv) < 2:
    print("用法: python analysis/run_pp_param.py <lookahead_value>")
    sys.exit(1)

LOOKAHEAD = float(sys.argv[1])

# ---- 方式1：改 dataclass 的 __init__ 默认值 ----
from vdm_lab.common.types import ControllerConfig
import dataclasses

# 拿到当前字段的默认值列表
field_defaults = list(ControllerConfig.__dataclass_fields__.values())
# 找到 pp_base_lookahead 字段在 __init__ 中的位置
# dataclass 的 __init__ 签名里，pp_base_lookahead 的默认值来自 field.default
# 直接改字段对象的 default
ControllerConfig.__dataclass_fields__["pp_base_lookahead"].default = LOOKAHEAD
# 同时刷新 __init__ 的 __defaults__
if ControllerConfig.__init__.__defaults__ is not None:
    defaults = list(ControllerConfig.__init__.__defaults__)
    # pp_base_lookahead 是第几个有默认值的参数？
    # 顺序 kp_speed, pp_base_lookahead, pp_speed_gain, lqr_q, ...
    # __defaults__ 只包含有默认值的参数，从最后一个开始
    # kp_speed 是第一个有默认值的，所以 index 0
    defaults[1] = LOOKAHEAD   # pp_base_lookahead 是第二个
    ControllerConfig.__init__.__defaults__ = tuple(defaults)

# ---- 方式2：monkey patch __init__ 双保险 ----
_orig_init = ControllerConfig.__init__
def _patched_init(self, *args, **kwargs):
    _orig_init(self, *args, **kwargs)
    self.pp_base_lookahead = LOOKAHEAD
ControllerConfig.__init__ = _patched_init

print(f"[A-2] pp_base_lookahead forced to {LOOKAHEAD}")

# ---- 构造 argv，调用 run_experiment ----
sys.argv = [
    "run_experiment.py",
    "--algo", "pp",
    "--version", "student",
    "--route", "s_curve",
    "--speed-mode", "medium",
    "--vehicle", "student_car",
    "--save-log",
    "--save-fig",
]

import run_experiment
run_experiment.main()

# ---- 写 marker ----
import glob
import json
OUTPUTS_DIR = os.path.join(PROJECT_ROOT, "outputs")
cands = sorted(glob.glob(os.path.join(OUTPUTS_DIR, "*_pp_s_curve_medium")),
               key=os.path.getmtime)
if cands:
    marker = os.path.join(cands[-1], "lookahead_value.json")
    with open(marker, "w", encoding="utf-8") as f:
        json.dump({"pp_base_lookahead": LOOKAHEAD}, f)
    print(f"[A-2] marker 写入: {marker}")