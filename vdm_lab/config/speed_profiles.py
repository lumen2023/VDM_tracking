SPEED_MODE_TITLES = {
    "low": "低速",
    "medium": "中速",
    "high": "高速",
}

DEFAULT_SPEED_MODE = "low"


def available_speed_modes():
    return tuple(SPEED_MODE_TITLES.keys())


def resolve_target_speed(route, speed_mode, override_speed=None):
    if override_speed is not None:
        return float(override_speed)
    try:
        return float(route.target_speeds[speed_mode])
    except KeyError as exc:
        choices = ", ".join(available_speed_modes())
        raise ValueError(f"未知速度档位 {speed_mode}，可选值为: {choices}") from exc
