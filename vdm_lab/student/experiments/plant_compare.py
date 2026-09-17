"""Same PP / circle / medium on kinematic vs student dynamic plants."""

from vdm_lab.student.experiments.runner import run_many


def main():
    return run_many(
        {
            "algo": "pp",
            "label": f"plant_pp_circle_medium_{plant}",
            "route": "circle",
            "speed_mode": "medium",
            "plant": plant,
        }
        for plant in ("kinematic", "dynamic")
    )


if __name__ == "__main__":
    main()
