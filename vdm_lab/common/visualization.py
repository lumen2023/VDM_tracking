import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from vdm_lab.common.types import ControlCommand


def _vehicle_polygon(state, vehicle):
    length_front = vehicle.wheelbase + vehicle.front_overhang
    length_rear = vehicle.rear_overhang
    half_width = vehicle.width / 2.0
    body = np.array(
        [
            [length_front, half_width],
            [length_front, -half_width],
            [-length_rear, -half_width],
            [-length_rear, half_width],
            [length_front, half_width],
        ]
    )
    rot = np.array(
        [
            [math.cos(state.yaw), -math.sin(state.yaw)],
            [math.sin(state.yaw), math.cos(state.yaw)],
        ]
    )
    return body @ rot.T + np.array([state.x, state.y])


def _rotate(points, yaw):
    rot = np.array(
        [
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ]
    )
    return points @ rot.T


def _wheel_polygon(cx, cy, yaw, vehicle):
    half_l = vehicle.tire_radius
    half_w = vehicle.tire_width / 2.0
    wheel = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
            [half_l, half_w],
        ]
    )
    return _rotate(wheel, yaw) + np.array([cx, cy])


def draw_vehicle(ax, state, command, vehicle):
    body = _vehicle_polygon(state, vehicle)
    ax.plot(body[:, 0], body[:, 1], color="#111827", linewidth=1.6)
    axle_points = [
        (0.0, vehicle.wheel_track / 2.0, state.yaw),
        (0.0, -vehicle.wheel_track / 2.0, state.yaw),
        (vehicle.wheelbase, vehicle.wheel_track / 2.0, state.yaw + command.steer),
        (vehicle.wheelbase, -vehicle.wheel_track / 2.0, state.yaw + command.steer),
    ]
    for local_x, local_y, wheel_yaw in axle_points:
        wx = state.x + local_x * math.cos(state.yaw) - local_y * math.sin(state.yaw)
        wy = state.y + local_x * math.sin(state.yaw) + local_y * math.cos(state.yaw)
        wheel = _wheel_polygon(wx, wy, wheel_yaw, vehicle)
        ax.plot(wheel[:, 0], wheel[:, 1], color="#111827", linewidth=1.2)

    arrow_len = vehicle.wheelbase * 0.7
    ax.arrow(state.x, state.y, arrow_len * math.cos(state.yaw), arrow_len * math.sin(state.yaw),
             color="#111827", width=0.04, head_width=0.45, length_includes_head=True)
    front_x = state.x + vehicle.wheelbase * math.cos(state.yaw)
    front_y = state.y + vehicle.wheelbase * math.sin(state.yaw)
    steer_yaw = state.yaw + command.steer
    ax.arrow(front_x, front_y, 0.9 * math.cos(steer_yaw), 0.9 * math.sin(steer_yaw),
             color="#dc2626", width=0.025, head_width=0.35, length_includes_head=True)


class LiveRenderer:
    def __init__(self, path, vehicle, title, gif_path=None, fps=12):
        self.path = path
        self.vehicle = vehicle
        self.title = title
        self.gif_path = gif_path
        self.fps = fps
        self.frames = []
        self.x_margin = max(6.0, 0.08 * (path.x.max() - path.x.min()))
        self.y_margin = max(6.0, 0.18 * (path.y.max() - path.y.min()))
        self.fig, self.ax = plt.subplots(figsize=(10, 6))

    def draw(self, state, records, reference, command, prediction=None):
        self.ax.clear()
        self.ax.plot(self.path.x, self.path.y, color="#6b7280", linewidth=2.0, label="reference")
        self.ax.plot([r.x for r in records], [r.y for r in records], color="#2563eb", linewidth=2.0, label="vehicle")
        self.ax.scatter(
            self.path.x[reference.nearest_index],
            self.path.y[reference.nearest_index],
            color="#dc2626",
            s=35,
            label="nearest target",
        )
        if prediction is not None:
            self.ax.plot(prediction[0], prediction[1], color="#9333ea", marker=".", linewidth=1.2, label="MPC prediction")
        draw_vehicle(self.ax, state, command, self.vehicle)
        self.ax.set_xlim(self.path.x.min() - self.x_margin, self.path.x.max() + self.x_margin)
        self.ax.set_ylim(self.path.y.min() - self.y_margin, self.path.y.max() + self.y_margin)
        self.ax.set_title(
            f"{self.title} | v={state.v:.2f} m/s | e_y={reference.lateral_error:.2f} m | e_yaw={reference.heading_error:.2f} rad"
        )
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper left")
        plt.pause(0.001)
        if self.gif_path is not None:
            self._capture_frame()

    def finish(self):
        if self.gif_path is not None and self.frames:
            duration_ms = int(1000 / self.fps)
            self.frames[0].save(
                self.gif_path,
                save_all=True,
                append_images=self.frames[1:],
                duration=duration_ms,
                loop=0,
            )
        plt.show()

    def _capture_frame(self):
        """Capture current canvas frame safely across Windows/Linux HiDPI."""
        self.fig.canvas.draw()

        rgba = np.asarray(self.fig.canvas.buffer_rgba())

        if rgba.ndim != 3 or rgba.shape[2] != 4:
            raise RuntimeError(
                f"Unexpected canvas buffer shape: {rgba.shape}"
            )

        rgb = np.ascontiguousarray(rgba[:, :, :3])

        self.frames.append(Image.fromarray(rgb))

def save_summary(path, records, output_path, title):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes[0, 0]
    ax.plot(path.x, path.y, color="#6b7280", linewidth=2.0, label="reference")
    ax.plot([r.x for r in records], [r.y for r in records], color="#2563eb", linewidth=2.0, label="vehicle")
    ax.set_title("Trajectory")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    times = [r.time for r in records]
    axes[0, 1].plot(times, [r.lateral_error for r in records], color="#dc2626")
    axes[0, 1].set_title("Lateral error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("error [m]")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(times, [r.speed for r in records], color="#16a34a", label="speed")
    axes[1, 0].plot(times, [r.target_speed for r in records], color="#6b7280", linestyle="--", label="target")
    axes[1, 0].set_title("Speed")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("m/s")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(times, [r.steer for r in records], color="#9333ea", label="steer")
    axes[1, 1].plot(times, [r.acceleration for r in records], color="#ea580c", label="acceleration")
    axes[1, 1].set_title("Control input")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def save_gif(path, records, predictions, output_path, title, vehicle, fps=12, max_frames=180):
    if not records:
        raise ValueError("没有仿真记录，无法生成 GIF。")

    frame_indices = _sample_frame_indices(len(records), max_frames)
    prediction_by_time = {round(time, 6): prediction for time, prediction in predictions}
    x_margin = max(6.0, 0.08 * (path.x.max() - path.x.min()))
    y_margin = max(6.0, 0.18 * (path.y.max() - path.y.min()))

    fig, ax = plt.subplots(figsize=(10, 6))

    def draw_frame(record_index):
        record = records[record_index]
        ax.clear()
        ax.plot(path.x, path.y, color="#6b7280", linewidth=2.0, label="reference")
        ax.plot(
            [r.x for r in records[: record_index + 1]],
            [r.y for r in records[: record_index + 1]],
            color="#2563eb",
            linewidth=2.2,
            label="vehicle",
        )
        ax.scatter(path.x[record.target_index], path.y[record.target_index], color="#dc2626", s=35, label="target")

        prediction = prediction_by_time.get(round(record.time, 6))
        if prediction is not None:
            ax.plot(prediction[0], prediction[1], color="#9333ea", marker=".", linewidth=1.2, label="MPC prediction")

        state = _state_from_record(record)
        command = ControlCommand(acceleration=record.acceleration, steer=record.steer)
        draw_vehicle(ax, state, command, vehicle)
        ax.set_xlim(path.x.min() - x_margin, path.x.max() + x_margin)
        ax.set_ylim(path.y.min() - y_margin, path.y.max() + y_margin)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")
        ax.set_title(
            f"{title} | t={record.time:.1f}s | v={record.speed:.2f}m/s | e_y={record.lateral_error:.2f}m"
        )

    gif = animation.FuncAnimation(fig, draw_frame, frames=frame_indices, interval=1000 / fps)
    gif.save(output_path, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    return output_path


def _sample_frame_indices(length, max_frames):
    if length <= max_frames:
        return list(range(length))
    return np.linspace(0, length - 1, max_frames, dtype=int).tolist()


def _state_from_record(record):
    class State:
        pass

    state = State()
    state.x = record.x
    state.y = record.y
    state.yaw = record.yaw
    state.v = record.speed
    return state
