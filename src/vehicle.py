import numpy as np
import dearpygui.dearpygui as dpg
from config import LENGTH, WIDTH, BACKTOWHEEL, WHEEL_LEN, WHEEL_WIDTH, TREAD, WB


def _compute_vehicle_geometry(x, y, yaw, steer=0.0):
    """
    Compute vehicle outline and wheel positions after rotation and translation.
    Returns (outline, fr_wheel, rr_wheel, fl_wheel, rl_wheel) as numpy arrays.
    Adapted from: https://github.com/AtsushiSakai/PythonRobotics
    """
def _compute_vehicle_geometry(x, y, yaw, steer=0.0):
    """
    Compute modern vehicle hull, cabin and wheel positions.
    Returns (hull, cabin, fr_wheel, rr_wheel, fl_wheel, rl_wheel)
    """
    # Aerodynamic Hull
    f, b = LENGTH - BACKTOWHEEL, -BACKTOWHEEL
    w = WIDTH / 2
    hull = np.array([
        [b, f-0.2, f, f, f-0.2, b, b-0.1, b],
        [w-0.1, w, w-0.1, -w+0.1, -w, -w+0.1, 0, w-0.1]
    ])
    
    # Cabin/Windshield
    cabin = np.array([
        [b+0.4, f-0.4, f-0.4, b+0.4, b+0.4],
        [w-0.2, w-0.3, -w+0.3, -w+0.2, w-0.2]
    ])

    fr_wheel = np.array([
        [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
        [-WHEEL_WIDTH/2, -WHEEL_WIDTH/2, WHEEL_WIDTH/2, WHEEL_WIDTH/2, -WHEEL_WIDTH/2],
    ])
    fr_wheel[1, :] -= TREAD/2

    rr_wheel = np.copy(fr_wheel)
    fl_wheel = np.copy(fr_wheel); fl_wheel[1, :] += TREAD
    rl_wheel = np.copy(rr_wheel); rl_wheel[1, :] += TREAD

    rot_yaw = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    rot_steer = np.array([[np.cos(steer), np.sin(steer)], [-np.sin(steer), np.cos(steer)]])

    # Steering to front wheels
    fr_wheel = (fr_wheel.T.dot(rot_steer)).T
    fl_wheel = (fl_wheel.T.dot(rot_steer)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    # Apply yaw rotation and translation
    parts = [hull, cabin, fr_wheel, rr_wheel, fl_wheel, rl_wheel]
    results = []
    for p in parts:
        p_rot = (p.T.dot(rot_yaw)).T
        p_rot[0, :] += x
        p_rot[1, :] += y
        results.append(p_rot)

    return tuple(results)


def plotVehicle(x, y, yaw, steer=0.0):
    """Create initial vehicle plot series (called once)."""
    hull, cabin, fr, rr, fl, rl = _compute_vehicle_geometry(x, y, yaw, steer)

    parts = [
        ("hull", hull), ("cabin", cabin),
        ("fr_wheel", fr), ("rr_wheel", rr), ("fl_wheel", fl), ("rl_wheel", rl),
    ]
    for tag, data in parts:
        dpg.add_line_series(
            np.array(data[0, :]).flatten(),
            np.array(data[1, :]).flatten(),
            parent="yaxis", tag=tag,
        )
        dpg.bind_item_theme(tag, "vehicle_theme")

    dpg.add_scatter_series([x], [y], parent="yaxis", tag="origin_marker")
    dpg.bind_item_theme("origin_marker", "origin_marker_theme")


def update_vehicle(x, y, yaw, steer=0.0):
    """Update existing vehicle plot series positions."""
    hull, cabin, fr, rr, fl, rl = _compute_vehicle_geometry(x, y, yaw, steer)

    parts = [
        ("hull", hull), ("cabin", cabin),
        ("fr_wheel", fr), ("rr_wheel", rr), ("fl_wheel", fl), ("rl_wheel", rl),
    ]
    for tag, data in parts:
        dpg.configure_item(tag, x=np.array(data[0, :]).flatten(), y=np.array(data[1, :]).flatten())

    dpg.configure_item("origin_marker", x=[x], y=[y])