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


def draw_vehicle(
    ax,
    state,
    command,
    vehicle,
    color="#111827",
    alpha=1.0,
    linewidth=1.6,
    show_steer=True,
):
    body = _vehicle_polygon(state, vehicle)
    ax.plot(
        body[:, 0],
        body[:, 1],
        color=color,
        linewidth=linewidth,
        alpha=alpha,
    )
    axle_points = [
        (0.0, vehicle.wheel_track / 2.0, state.yaw),
        (0.0, -vehicle.wheel_track / 2.0, state.yaw),
        (
            vehicle.wheelbase,
            vehicle.wheel_track / 2.0,
            state.yaw + command.steer,
        ),
        (
            vehicle.wheelbase,
            -vehicle.wheel_track / 2.0,
            state.yaw + command.steer,
        ),
    ]
    for local_x, local_y, wheel_yaw in axle_points:
        wx = (
            state.x
            + local_x * math.cos(state.yaw)
            - local_y * math.sin(state.yaw)
        )
        wy = (
            state.y
            + local_x * math.sin(state.yaw)
            + local_y * math.cos(state.yaw)
        )
        wheel = _wheel_polygon(wx, wy, wheel_yaw, vehicle)
        ax.plot(
            wheel[:, 0],
            wheel[:, 1],
            color=color,
            linewidth=max(0.8, linewidth * 0.75),
            alpha=alpha,
        )

    arrow_len = vehicle.wheelbase * 0.7
    ax.arrow(
        state.x,
        state.y,
        arrow_len * math.cos(state.yaw),
        arrow_len * math.sin(state.yaw),
        color=color,
        width=0.04,
        head_width=0.45,
        length_includes_head=True,
        alpha=alpha,
    )
    if not show_steer:
        return

    front_x = state.x + vehicle.wheelbase * math.cos(state.yaw)
    front_y = state.y + vehicle.wheelbase * math.sin(state.yaw)
    steer_yaw = state.yaw + command.steer
    ax.arrow(
        front_x,
        front_y,
        0.9 * math.cos(steer_yaw),
        0.9 * math.sin(steer_yaw),
        color="#dc2626",
        width=0.025,
        head_width=0.35,
        length_includes_head=True,
    )


def draw_history_ghosts(ax, records, vehicle, stride=12, count=0):
    if not records or stride <= 0:
        return

    history = records[:-1]
    if not history:
        return

    sampled = history[::stride]
    if count > 0:
        sampled = sampled[-count:]
    if not sampled:
        return

    for i, record in enumerate(sampled):
        alpha = 0.08 + 0.18 * (i + 1) / len(sampled)
        state = _state_from_record(record)
        command = ControlCommand(
            acceleration=record.acceleration,
            steer=record.steer,
        )
        draw_vehicle(
            ax,
            state,
            command,
            vehicle,
            color="#64748b",
            alpha=alpha,
            linewidth=1.0,
            show_steer=False,
        )


def _resolve_view_mode(path, view_mode, follow_radius):
    if view_mode not in {"auto", "full", "follow"}:
        raise ValueError("view_mode 必须是 auto / full / follow")

    if view_mode != "auto":
        return view_mode

    route_span = max(
        float(np.ptp(path.x)),
        float(np.ptp(path.y)),
    )

    # Whole-route view is fine for the original tens-of-metres teaching routes.
    # Kilometre-scale routes automatically switch to a local following camera.
    return "follow" if route_span > 4.0 * follow_radius else "full"


def _set_full_limits(ax, path):
    x_margin = max(6.0, 0.08 * float(np.ptp(path.x)))
    y_margin = max(6.0, 0.18 * float(np.ptp(path.y)))
    ax.set_xlim(path.x.min() - x_margin, path.x.max() + x_margin)
    ax.set_ylim(path.y.min() - y_margin, path.y.max() + y_margin)


def draw_basemap(ax, basemap, opacity=0.72, show_attribution=True):
    """Draw a cached geographic raster in the simulation's local frame."""
    if basemap is None:
        return
    ax.imshow(
        basemap.image,
        extent=basemap.extent,
        origin="upper",
        alpha=opacity,
        interpolation="bilinear",
        zorder=-100,
    )
    if show_attribution:
        ax.text(
            0.995,
            0.005,
            basemap.attribution,
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=7,
            color="#111827",
            bbox={
                "facecolor": "white",
                "edgecolor": "none",
                "alpha": 0.72,
                "pad": 1.5,
            },
            zorder=100,
        )


def _draw_overview(
    ax,
    path,
    records,
    state,
    basemap=None,
    basemap_opacity=0.72,
):
    ax.clear()
    draw_basemap(
        ax,
        basemap,
        opacity=basemap_opacity,
        show_attribution=False,
    )
    ax.plot(path.x, path.y, color="#9ca3af", linewidth=1.0)
    if records:
        ax.plot(
            [r.x for r in records],
            [r.y for r in records],
            color="#2563eb",
            linewidth=1.2,
        )
    ax.scatter([state.x], [state.y], color="#dc2626", s=22, zorder=3)
    ax.scatter([path.x[0]], [path.y[0]], color="#16a34a", s=18, zorder=3)
    ax.scatter([path.x[-1]], [path.y[-1]], color="#111827", s=18, zorder=3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Whole route", fontsize=9)
    ax.grid(True, alpha=0.18)


class LiveRenderer:
    """
    Real-time visualization.

    full:
        Original whole-route display.

    follow:
        Main axes follow the vehicle at a fixed metric radius. A small inset
        keeps the whole route visible, which makes kilometre-scale GPX routes
        practical while keeping the vehicle itself readable.

    auto:
        Chooses follow for long routes and full for small teaching routes.
    """

    def __init__(
        self,
        path,
        vehicle,
        title,
        gif_path=None,
        fps=12,
        show_history_ghosts=True,
        ghost_stride=12,
        ghost_count=0,
        view_mode="auto",
        follow_radius=45.0,
        basemap=None,
        basemap_opacity=0.72,
    ):
        self.path = path
        self.vehicle = vehicle
        self.title = title
        self.gif_path = gif_path
        self.fps = fps
        self.show_history_ghosts = show_history_ghosts
        self.ghost_stride = ghost_stride
        self.ghost_count = ghost_count
        self.follow_radius = float(follow_radius)
        self.basemap = basemap
        self.basemap_opacity = float(basemap_opacity)
        if self.follow_radius <= 0:
            raise ValueError("follow_radius 必须大于 0。")
        if not 0.0 <= self.basemap_opacity <= 1.0:
            raise ValueError("basemap_opacity 必须在 0..1 之间。")

        self.view_mode = _resolve_view_mode(
            path,
            view_mode,
            self.follow_radius,
        )
        self.frames = []

        if self.view_mode == "follow":
            self.fig, self.ax = plt.subplots(figsize=(10, 7))
            self.overview_ax = self.fig.add_axes([0.70, 0.66, 0.26, 0.25])
        else:
            self.fig, self.ax = plt.subplots(figsize=(10, 6))
            self.overview_ax = None

    def draw(self, state, records, reference, command, prediction=None):
        self.ax.clear()

        draw_basemap(
            self.ax,
            self.basemap,
            opacity=self.basemap_opacity,
        )

        self.ax.plot(
            self.path.x,
            self.path.y,
            color="#6b7280",
            linewidth=1.6,
            label="reference",
        )
        self.ax.plot(
            [r.x for r in records],
            [r.y for r in records],
            color="#2563eb",
            linewidth=2.0,
            label="vehicle",
        )
        self.ax.scatter(
            self.path.x[reference.nearest_index],
            self.path.y[reference.nearest_index],
            color="#dc2626",
            s=35,
            label="nearest target",
        )
        if prediction is not None:
            self.ax.plot(
                prediction[0],
                prediction[1],
                color="#9333ea",
                marker=".",
                linewidth=1.2,
                label="MPC prediction",
            )

        if self.show_history_ghosts:
            draw_history_ghosts(
                self.ax,
                records,
                self.vehicle,
                self.ghost_stride,
                self.ghost_count,
            )

        draw_vehicle(self.ax, state, command, self.vehicle)

        if self.view_mode == "follow":
            radius = self.follow_radius
            self.ax.set_xlim(state.x - radius, state.x + radius)
            self.ax.set_ylim(state.y - radius, state.y + radius)
            _draw_overview(
                self.overview_ax,
                self.path,
                records,
                state,
                basemap=self.basemap,
                basemap_opacity=self.basemap_opacity,
            )
        else:
            _set_full_limits(self.ax, self.path)

        self.ax.set_title(
            f"{self.title} | v={state.v:.2f} m/s | "
            f"e_y={reference.lateral_error:.2f} m | "
            f"e_yaw={reference.heading_error:.2f} rad"
        )
        self.ax.set_xlabel("x / East [m]")
        self.ax.set_ylabel("y / North [m]")
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


def save_summary(
    path,
    records,
    output_path,
    title,
    basemap=None,
    basemap_opacity=0.72,
):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes[0, 0]
    draw_basemap(ax, basemap, opacity=basemap_opacity)
    ax.plot(
        path.x,
        path.y,
        color="#6b7280",
        linewidth=1.7,
        label="reference",
    )
    ax.plot(
        [r.x for r in records],
        [r.y for r in records],
        color="#2563eb",
        linewidth=1.7,
        label="vehicle",
    )
    ax.set_title("Trajectory")
    ax.set_xlabel("x / East [m]")
    ax.set_ylabel("y / North [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    times = [r.time for r in records]
    axes[0, 1].plot(
        times,
        [r.lateral_error for r in records],
        color="#dc2626",
    )
    axes[0, 1].set_title("Lateral error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("error [m]")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(
        times,
        [r.speed for r in records],
        color="#16a34a",
        label="speed",
    )
    axes[1, 0].plot(
        times,
        [r.target_speed for r in records],
        color="#6b7280",
        linestyle="--",
        label="target",
    )
    axes[1, 0].set_title("Speed")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("m/s")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(
        times,
        [r.steer for r in records],
        color="#9333ea",
        label="steer [rad]",
    )
    axes[1, 1].plot(
        times,
        [r.beta for r in records],
        color="#0f766e",
        label="beta [rad]",
    )
    axes[1, 1].plot(
        times,
        [r.normal_accel for r in records],
        color="#ea580c",
        label="normal accel [m/s^2]",
    )
    axes[1, 1].set_title("Control and bicycle-model terms")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def save_gpx_overview(
    path,
    output_path,
    basemap=None,
    basemap_opacity=0.72,
):
    """
    Save a GPX-specific overview.

    Left: local metric coordinates used by controllers.
    Right: original geographic latitude/longitude.
    """
    if getattr(path, "lat", None) is None or getattr(path, "lon", None) is None:
        return None

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    draw_basemap(axes[0], basemap, opacity=basemap_opacity)
    axes[0].plot(path.x, path.y, linewidth=1.5)
    axes[0].scatter(path.x[0], path.y[0], s=35, label="start")
    axes[0].scatter(path.x[-1], path.y[-1], s=35, label="goal")
    axes[0].set_title("Local metric route")
    axes[0].set_xlabel("East x [m]")
    axes[0].set_ylabel("North y [m]")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(path.lon, path.lat, linewidth=1.5)
    axes[1].scatter(path.lon[0], path.lat[0], s=35, label="start")
    axes[1].scatter(path.lon[-1], path.lat[-1], s=35, label="goal")
    axes[1].set_title("Original GPX coordinates")
    axes[1].set_xlabel("longitude [deg]")
    axes[1].set_ylabel("latitude [deg]")
    axes[1].set_aspect("equal", adjustable="datalim")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def save_gif(
    path,
    records,
    predictions,
    output_path,
    title,
    vehicle,
    fps=12,
    max_frames=180,
    show_history_ghosts=True,
    ghost_stride=12,
    ghost_count=0,
    view_mode="auto",
    follow_radius=45.0,
    basemap=None,
    basemap_opacity=0.72,
):
    if not records:
        raise ValueError("没有仿真记录，无法生成 GIF。")

    frame_indices = _sample_frame_indices(len(records), max_frames)
    prediction_by_time = {
        round(time, 6): prediction
        for time, prediction in predictions
    }

    resolved_view = _resolve_view_mode(path, view_mode, follow_radius)

    if resolved_view == "follow":
        fig, ax = plt.subplots(figsize=(10, 7))
        overview_ax = fig.add_axes([0.70, 0.66, 0.26, 0.25])
    else:
        fig, ax = plt.subplots(figsize=(10, 6))
        overview_ax = None

    def draw_frame(record_index):
        record = records[record_index]
        ax.clear()

        draw_basemap(ax, basemap, opacity=basemap_opacity)

        ax.plot(
            path.x,
            path.y,
            color="#6b7280",
            linewidth=1.6,
            label="reference",
        )
        ax.plot(
            [r.x for r in records[: record_index + 1]],
            [r.y for r in records[: record_index + 1]],
            color="#2563eb",
            linewidth=2.0,
            label="vehicle",
        )
        ax.scatter(
            path.x[record.target_index],
            path.y[record.target_index],
            color="#dc2626",
            s=35,
            label="target",
        )

        prediction = prediction_by_time.get(round(record.time, 6))
        if prediction is not None:
            ax.plot(
                prediction[0],
                prediction[1],
                color="#9333ea",
                marker=".",
                linewidth=1.2,
                label="MPC prediction",
            )

        if show_history_ghosts:
            draw_history_ghosts(
                ax,
                records[: record_index + 1],
                vehicle,
                ghost_stride,
                ghost_count,
            )

        state = _state_from_record(record)
        command = ControlCommand(
            acceleration=record.acceleration,
            steer=record.steer,
        )
        draw_vehicle(ax, state, command, vehicle)

        if resolved_view == "follow":
            ax.set_xlim(
                record.x - follow_radius,
                record.x + follow_radius,
            )
            ax.set_ylim(
                record.y - follow_radius,
                record.y + follow_radius,
            )
            _draw_overview(
                overview_ax,
                path,
                records[: record_index + 1],
                state,
                basemap=basemap,
                basemap_opacity=basemap_opacity,
            )
        else:
            _set_full_limits(ax, path)

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x / East [m]")
        ax.set_ylabel("y / North [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")
        ax.set_title(
            f"{title} | t={record.time:.1f}s | "
            f"v={record.speed:.2f}m/s | "
            f"e_y={record.lateral_error:.2f}m"
        )

    gif = animation.FuncAnimation(
        fig,
        draw_frame,
        frames=frame_indices,
        interval=1000 / fps,
    )
    gif.save(
        output_path,
        writer=animation.PillowWriter(fps=fps),
    )
    plt.close(fig)
    return output_path


def _sample_frame_indices(length, max_frames):
    if length <= max_frames:
        return list(range(length))
    return np.linspace(
        0,
        length - 1,
        max_frames,
        dtype=int,
    ).tolist()


def _state_from_record(record):
    class State:
        pass

    state = State()
    state.x = record.x
    state.y = record.y
    state.yaw = record.yaw
    state.v = record.speed
    return state
