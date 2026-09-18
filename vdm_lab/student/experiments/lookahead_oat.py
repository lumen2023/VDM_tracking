"""Pure-pursuit lookahead one-at-a-time sweep."""

from vdm_lab.student.experiments.runner import run_many


SPEEDS = (3.0, 5.0, 7.0)
LOOKAHEADS = (1.5, 3.0, 6.0)
ROUTES = ("circle", "s_curve", "right_angle")


def _label(route, speed, lookahead):
    speed_tag = str(speed).replace(".", "p")
    ld_tag = str(lookahead).replace(".", "p")
    return f"lookahead_{route}_v{speed_tag}_ld{ld_tag}"


def main():
    jobs = [
        {
            "algo": "pp",
            "label": _label(route, speed, lookahead),
            "route": route,
            "speed_mode": "medium",
            "target_speed": speed,
            "lookahead": lookahead,
            "extra": {"oat": "grid"},
        }
        for route in ROUTES
        for speed in SPEEDS
        for lookahead in LOOKAHEADS
    ]
    jobs.append(
        {
            "algo": "pp",
            "label": "lookahead_s_curve_high_ld1p5",
            "route": "s_curve",
            "speed_mode": "high",
            "lookahead": 1.5,
            "extra": {"oat": "high_x_ld1.5"},
        }
    )
    return run_many(jobs)


if __name__ == "__main__":
    main()
