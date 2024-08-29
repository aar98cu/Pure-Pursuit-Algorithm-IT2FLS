import numpy as np
from config import *
import dearpygui.dearpygui as dpg

def plotVehicle(x, y, yaw, steer=0.0):
    """
    The function is to plot the vehicle
    it is copied and modified from https://github.com/AtsushiSakai/PythonRobotics/blob/187b6aa35f3cbdeca587c0abdb177adddefc5c2a/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L109
    """
    outline = np.array(
        [
            [
                -BACKTOWHEEL,
                (LENGTH - BACKTOWHEEL),
                (LENGTH - BACKTOWHEEL),
                -BACKTOWHEEL,
                -BACKTOWHEEL,
            ],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    fr_wheel = np.array(
        [
            [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [
                -WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
            ],
        ]
    )

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    Rot2 = np.array(
        [[np.cos(steer), np.sin(steer)], [-np.sin(steer), np.cos(steer)]]
    )

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    dpg.add_line_series(
        np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(),
        parent="yaxis",
        tag="vehicle"
    )
    dpg.add_line_series(
        np.array(fr_wheel[0, :]).flatten(),
        np.array(fr_wheel[1, :]).flatten(),
        parent="yaxis",
        tag="fr_wheel"
    )
    dpg.add_line_series(
        np.array(rr_wheel[0, :]).flatten(),
        np.array(rr_wheel[1, :]).flatten(),
        parent="yaxis",
        tag="rr_wheel"
    )
    dpg.add_line_series(
        np.array(fl_wheel[0, :]).flatten(),
        np.array(fl_wheel[1, :]).flatten(),
        parent="yaxis",
        tag="fl_wheel"
    )
    dpg.add_line_series(
        np.array(rl_wheel[0, :]).flatten(),
        np.array(rl_wheel[1, :]).flatten(),
        parent="yaxis",
        tag="rl_wheel"
    )
    dpg.draw_circle(center=(x, y), radius=0.15, color=(255, 0, 0, 255), fill=(255, 0, 0, 255), parent="overlay_drawlist")


    dpg.bind_item_theme("vehicle", "vehicle_theme")
    dpg.bind_item_theme("fr_wheel", "vehicle_theme")
    dpg.bind_item_theme("rr_wheel", "vehicle_theme")
    dpg.bind_item_theme("fl_wheel", "vehicle_theme")
    dpg.bind_item_theme("rl_wheel", "vehicle_theme")

def update_vehicle(x, y, yaw, steer=0.0):
    outline = np.array(
        [
            [
                -BACKTOWHEEL,
                (LENGTH - BACKTOWHEEL),
                (LENGTH - BACKTOWHEEL),
                -BACKTOWHEEL,
                -BACKTOWHEEL,
            ],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    fr_wheel = np.array(
        [
            [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [
                -WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
            ],
        ]
    )

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    Rot2 = np.array(
        [[np.cos(steer), np.sin(steer)], [-np.sin(steer), np.cos(steer)]]
    )

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    dpg.configure_item(
        "vehicle",
        x = np.array(outline[0, :]).flatten(),
        y = np.array(outline[1, :]).flatten(),
    )
    dpg.configure_item(
        "fr_wheel",
        x = np.array(fr_wheel[0, :]).flatten(),
        y = np.array(fr_wheel[1, :]).flatten(),
    )
    dpg.configure_item(
        "rr_wheel",
        x = np.array(rr_wheel[0, :]).flatten(),
        y = np.array(rr_wheel[1, :]).flatten(),
    )
    dpg.configure_item(
        "fl_wheel",
        x = np.array(fl_wheel[0, :]).flatten(),
        y = np.array(fl_wheel[1, :]).flatten(),
    )
    dpg.configure_item(
        "rl_wheel",
        x = np.array(rl_wheel[0, :]).flatten(),
        y = np.array(rl_wheel[1, :]).flatten(),
    )