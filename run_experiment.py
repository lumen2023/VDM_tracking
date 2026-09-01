import argparse

from vdm_lab.common.logging import (
    compute_metrics,
    create_output_dir,
    save_metrics,
    save_predictions,
    save_records,
)
from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import LabConfig
from vdm_lab.common.visualization import save_gif, save_summary
from vdm_lab.config.routes import DEFAULT_ROUTE_NAME, available_route_names
from vdm_lab.config.speed_profiles import DEFAULT_SPEED_MODE, available_speed_modes
from vdm_lab.config.vehicle_params import DEFAULT_VEHICLE_NAME, available_vehicle_names, make_vehicle_config


def parse_args():
    parser = argparse.ArgumentParser(description="VDM 学生路径跟踪仿真实验")
    parser.add_argument("--algo", choices=["pp", "lqr_kinematic", "lqr_dynamic", "mpc"], required=True)
    parser.add_argument("--version", choices=["solution", "student"], default="solution")
    parser.add_argument("--route", choices=available_route_names(), default=DEFAULT_ROUTE_NAME, help="选择跟踪路线")
    parser.add_argument("--vehicle", choices=available_vehicle_names(), default=DEFAULT_VEHICLE_NAME, help="选择车辆参数组")
    parser.add_argument("--speed-mode", choices=available_speed_modes(), default=DEFAULT_SPEED_MODE, help="目标速度档位，默认 low 低速")
    parser.add_argument("--target-speed", type=float, default=None, help="手动覆盖速度档位，单位 m/s")
    parser.add_argument("--animate", action="store_true", help="显示实时动画")
    parser.add_argument("--save-log", action="store_true", help="保存 trajectory.csv 和 metrics.json")
    parser.add_argument("--save-fig", action="store_true", help="保存 summary.png")
    parser.add_argument("--save-gif", action="store_true", help="保存 animation.gif 教学演示动图")
    parser.add_argument(
        "--history-ghosts",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="在实时动画和 GIF 中显示历史车辆姿态虚影，默认开启；关闭用 --no-history-ghosts",
    )
    parser.add_argument("--ghost-stride", type=int, default=12, help="历史虚影采样间隔，单位为仿真步")
    parser.add_argument("--ghost-count", type=int, default=0, help="最多显示的历史虚影数量；0 表示从起点开始一直保留")
    return parser.parse_args()


def main():
    args = parse_args()
    controller = load_controller(args.algo, args.version)
    output_dir = None
    if args.save_log or args.save_fig or args.save_gif:
        output_dir = create_output_dir(f"{args.algo}_{args.route}_{args.speed_mode}")
    config = LabConfig(vehicle=make_vehicle_config(args.vehicle))
    config.sim.route_name = args.route
    config.sim.speed_mode = args.speed_mode
    config.sim.show_history_ghosts = args.history_ghosts
    config.sim.history_ghost_stride = args.ghost_stride
    config.sim.history_ghost_count = args.ghost_count
    if args.target_speed is not None:
        config.sim.target_speed = args.target_speed

    live_gif_path = output_dir / "animation.gif" if args.animate and args.save_gif else None
    path, records, predictions = run_simulation(
        controller,
        config=config,
        animate=args.animate,
        gif_path=live_gif_path,
    )
    metrics = compute_metrics(path, records)
    if args.save_log:
        save_records(output_dir, records)
        save_predictions(output_dir, predictions)
        save_metrics(output_dir, metrics)
    if args.save_fig:
        save_summary(path, records, output_dir / "summary.png", getattr(controller, "NAME", args.algo))
    if args.save_gif and live_gif_path is None:
        save_gif(
            path,
            records,
            predictions,
            output_dir / "animation.gif",
            getattr(controller, "NAME", args.algo),
            config.vehicle,
            show_history_ghosts=config.sim.show_history_ghosts,
            ghost_stride=config.sim.history_ghost_stride,
            ghost_count=config.sim.history_ghost_count,
        )

    print(f"algo={args.algo}, version={args.version}")
    print(f"route={args.route}, speed_mode={args.speed_mode}, vehicle={args.vehicle}")
    print(f"steps={metrics['steps']}, reached_goal={metrics['reached_goal']}")
    print(f"mean_lateral_error={metrics['mean_lateral_error_m']:.3f} m")
    print(f"max_lateral_error={metrics['max_lateral_error_m']:.3f} m")
    print(f"finish_error={metrics['finish_error_m']:.3f} m")
    print(f"min_speed={metrics['min_speed_mps']:.3f} m/s")
    if output_dir is not None:
        print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
