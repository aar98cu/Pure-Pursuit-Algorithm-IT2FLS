import math
import os
import time
import threading
import csv
import numpy as np
import dearpygui.dearpygui as dpg
import pyautogui

import config
from config import (
    dt, look_ahead, WB,
    NOISE_MATRIX_SIZE, noise_matrix,
    params, paths, list_paths,
    steering_control, velocity_control, steering_mode,
    MF_parameters_it2mf2_delta, C_parameters_it2mf2_delta, Rules_it2mf2,
    MF_parameters_it2mf3_delta, C_parameters_it2mf3_delta, Rules_it2mf3,
    MF_parameters_t1mf2_delta, C_parameters_t1mf2_delta, Rules_t1mf2,
    MF_parameters_it2mf2_vel, C_parameters_it2mf2_vel,
    MF_parameters_t1mf2_steering, C_parameters_t1mf2_steering,
)
from vehicle import update_vehicle

# ---------------------------------------------------------------------------
# Module-level mutable state (replaces scattered globals)
# ---------------------------------------------------------------------------
_steering_control_func = None
_velocity_control_func = None
_steering_mode_func = None
_previous_delta_error = 0.0


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------
def getDistance(p1, p2):
    """Euclidean distance between two 2D points."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def abbreviate_name(name):
    """Create a short abbreviation from a control method name."""
    words = name.split()
    parts = []
    for word in words:
        if word.isalnum() and any(ch.isdigit() for ch in word):
            parts.append(word)
        elif word.lower() == "with":
            parts.append("W")
        elif word.lower() == "pi":
            parts.append("PI")
        else:
            parts.append(word[0].upper())
    return "".join(parts)


def _generalized_cauchy_mf(x, a, b, c, h=1.0):
    """Compute generalized Cauchy membership function: h / (1 + ((x-c)/a)^(2b))."""
    if a == 0.0: return h
    tmp = (x - c) / a
    if tmp == 0.0: return h
    return h / (1.0 + (tmp * tmp)**b)

def get_cauchy_points(a, b, c, h=1.0, range_val=1.0):
    """Generate X and Y points for plotting a Cauchy MF."""
    xs = np.linspace(c - range_val, c + range_val, 100)
    ys = np.array([_generalized_cauchy_mf(x, a, b, c, h) for x in xs])
    return xs, ys

def plot_mfs(controller, input_idx=0, plot_tag="fuzzy_plot_1"):
    """Update DPG fuzzy plot with controller membership functions."""
    axis_tag = f"{plot_tag}_yaxis"
    if not dpg.does_item_exist(axis_tag): return
    dpg.delete_item(axis_tag, children_only=True)
    
    colors = [(137, 180, 250), (243, 139, 168), (166, 227, 161), (250, 179, 135)]
    mf_start = input_idx * controller.n_mf
    
    # Check if IT2 (5 params: al, au, b, c, h) or T1 (3 params: a, b, c)
    is_it2 = (controller.MF_params.shape[1] == 5)
    
    if is_it2:
        centers = [controller.MF_params[mf_start + i][3] for i in range(controller.n_mf)]
        widths = [max(controller.MF_params[mf_start + i][0], controller.MF_params[mf_start + i][1]) for i in range(controller.n_mf)]
    else:
        centers = [controller.MF_params[mf_start + i][2] for i in range(controller.n_mf)]
        widths = [controller.MF_params[mf_start + i][0] for i in range(controller.n_mf)]

    x_min = min(centers) - 1.5 * max(widths)
    x_max = max(centers) + 1.5 * max(widths)
    
    xaxis_tag = f"{plot_tag}_xaxis"
    if dpg.does_item_exist(xaxis_tag):
        dpg.set_axis_limits(xaxis_tag, x_min, x_max)
    if dpg.does_item_exist(axis_tag):
        dpg.set_axis_limits(axis_tag, 0, 1.1)

    for i in range(controller.n_mf):
        p = controller.MF_params[mf_start + i]
        if is_it2:
            # p: al, au, b, c, h
            w_p = max(p[0], p[1])
            rv = 2.5 * w_p
            xs, y_low = get_cauchy_points(p[0], p[2], p[3], p[4], rv)
            _, y_high = get_cauchy_points(p[1], p[2], p[3], 1.0, rv)
            dpg.add_line_series(xs, y_high, parent=axis_tag)
            dpg.add_shade_series(xs, y_low, y2=[0]*len(xs), parent=axis_tag)
        else:
            # p: a, b, c
            rv = 2.5 * p[0]
            xs, ys = get_cauchy_points(p[0], p[1], p[2], 1.0, rv)
            dpg.add_line_series(xs, ys, parent=axis_tag)
            
        dpg.add_scatter_series([0], [0], parent=axis_tag, tag=f"{plot_tag}_firing_{i}")


# ---------------------------------------------------------------------------
# PI Controller
# ---------------------------------------------------------------------------
class PI:
    def __init__(self, kp=1.0, ki=0.1):
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        self.Pterm = self.kp * error
        self.Iterm += error * dt
        self.last_error = error
        return self.Pterm + self.ki * self.Iterm


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------
class Trajectory:
    def __init__(self, traj_x, traj_y):
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < look_ahead and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx), curr_dist, self.last_idx


# ---------------------------------------------------------------------------
# IT2FLS — Interval Type-2 Fuzzy Logic System
# ---------------------------------------------------------------------------
class IT2FLS:
    def __init__(self, n_inputs, n_mf, n_rules, mf_p, rules, c_p):
        self.n_inputs = n_inputs
        self.n_mf = n_mf
        self.n_rules = n_rules
        self.MF_params = mf_p
        self.Rules = rules
        self.C_params = c_p
        # Pre-allocate nodes scratch space
        total_nodes = n_inputs + n_inputs * n_mf + 3 * n_rules + 1
        self._nodes = np.zeros((total_nodes, 2))

    def model(self, inputs, return_firing=False):
        nodes = self._nodes
        nodes[:] = 0.0  # Reset scratch space
        nodes[0:self.n_inputs, 0] = inputs

        # Layer 1: compute membership function values (lower & upper)
        firing_vals = []
        for i in range(self.n_inputs):
            for j in range(self.n_mf):
                idx = self.n_inputs + i * self.n_mf + j
                mf_row = i * self.n_mf + j
                x = inputs[i]
                al, au = self.MF_params[mf_row, 0], self.MF_params[mf_row, 1]
                b, c, h = self.MF_params[mf_row, 2], self.MF_params[mf_row, 3], self.MF_params[mf_row, 4]

                nodes[idx, 0] = _generalized_cauchy_mf(x, al, b, c, h)
                nodes[idx, 1] = _generalized_cauchy_mf(x, au, b, c, 1.0)
                if return_firing:
                    firing_vals.append((nodes[idx, 0], nodes[idx, 1]))

        # Layer 2: rule firing strengths
        st = self.n_inputs + self.n_inputs * self.n_mf
        for i in range(self.n_rules):
            rule_indices = self.Rules[i, :]
            nodes[st + i, 0] = np.prod(nodes[rule_indices, 0])
            nodes[st + i, 1] = np.prod(nodes[rule_indices, 1])

        # Layer 3: type reduction (KM algorithm)
        wi = nodes[st:st + self.n_rules, 0].copy()
        ws = nodes[st:st + self.n_rules, 1].copy()
        wi_orig = wi.copy()
        wi, ws = np.sort(wi), np.sort(ws)
        order = wi_orig.argsort()

        wn = (wi + ws) / 2.0
        wn_sum = np.sum(wn)
        if wn_sum == 0: return (0.0, firing_vals) if return_firing else 0.0

        yi, ys = np.sum(wi * wn) / wn_sum, np.sum(ws * wn) / wn_sum

        left_idx = 0; right_idx = 0
        for i in range(self.n_rules - 1):
            if wi[i] <= yi <= wi[i + 1]: left_idx = i
            if ws[i] <= ys <= ws[i + 1]: right_idx = i

        Xl_parts = np.concatenate((ws[0:left_idx + 1], wi[left_idx + 1:]))
        Xr_parts = np.concatenate((wi[0:right_idx + 1], ws[right_idx + 1:]))
        Xl_sum, Xr_sum = np.sum(Xl_parts), np.sum(Xr_parts)
        Xl = Xl_parts / Xl_sum if Xl_sum != 0 else Xl_parts
        Xr = Xr_parts / Xr_sum if Xr_sum != 0 else Xr_parts
        X = ((Xl + Xr) / 2.0)[order]

        # Layer 4: consequent computation
        st2 = self.n_inputs + self.n_inputs * self.n_mf + 2 * self.n_rules
        for i in range(self.n_rules):
            nodes[i + st2, 0] = X[i] * (np.sum(self.C_params[i, 0:-1] * inputs) + self.C_params[i, -1])

        res = np.sum(nodes[st2:st2 + self.n_rules, 0])
        return (res, firing_vals) if return_firing else res


# ---------------------------------------------------------------------------
# T1FLS — Type-1 Fuzzy Logic System
# ---------------------------------------------------------------------------
class T1FLS:
    def __init__(self, n_inputs, n_mf, n_rules, mf_p, rules, c_p):
        self.n_inputs = n_inputs
        self.n_mf = n_mf
        self.n_rules = n_rules
        self.MF_params = mf_p
        self.Rules = rules
        self.C_params = c_p
        total_nodes = n_inputs * n_mf + 3 * n_rules
        self._nodes = np.zeros((total_nodes, 1))

    def model(self, inputs, return_firing=False):
        nodes = self._nodes
        nodes[:] = 0.0
        firing_vals = []

        # Layer 1: membership functions
        for i in range(self.n_inputs):
            for j in range(self.n_mf):
                idx = i * self.n_mf + j
                mf_row = i * self.n_mf + j
                x = inputs[i]
                a = self.MF_params[mf_row, 0]
                b = self.MF_params[mf_row, 1]
                c = self.MF_params[mf_row, 2]
                nodes[idx, 0] = _generalized_cauchy_mf(x, a, b, c, 1.0)
                if return_firing:
                    firing_vals.append((nodes[idx, 0], nodes[idx, 0])) # T1 has upper=lower

        # Layer 2: rule firing
        st = self.n_inputs * self.n_mf
        for i in range(self.n_rules):
            rule_indices = self.Rules[i, :]
            nodes[st + i, 0] = np.prod(nodes[rule_indices, 0])

        # Layer 3: normalization
        firing_sum = np.sum(nodes[st:st + self.n_rules, 0])
        st_norm = st + self.n_rules
        if firing_sum != 0:
            for i in range(self.n_rules):
                nodes[st_norm + i, 0] = nodes[st + i, 0] / firing_sum

        # Layer 4: consequent
        st_out = self.n_inputs * self.n_mf + 2 * self.n_rules
        for i in range(self.n_rules):
            nodes[st_out + i, 0] = nodes[st_norm + i, 0] * (
                np.sum(self.C_params[i, 0:-1] * inputs) + self.C_params[i, -1]
            )

        return (np.sum(nodes[st_out:st_out + self.n_rules, 0]), firing_vals) if return_firing else np.sum(nodes[st_out:st_out + self.n_rules, 0])


# ---------------------------------------------------------------------------
# Vehicle mathematical model
# ---------------------------------------------------------------------------
class Mathematic_Model:
    def __init__(self, x, y, yaw, vel=0, delta=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        self.delta = delta
        self.noise_index = 0

    def get_noise(self):
        noise_value = noise_matrix[self.noise_index]
        self.noise_index = (self.noise_index + 1) % NOISE_MATRIX_SIZE
        return noise_value

    def update(self, velocity, delta):
        if dpg.get_value("noise_checkbox"):
            self.delta = _steering_mode_func(self.delta, delta)
            self.x += self.vel * math.cos(self.yaw) * dt + self.get_noise()
            self.y += self.vel * math.sin(self.yaw) * dt + self.get_noise()
            self.yaw += self.vel * math.tan(self.delta) / WB * dt

            delta_speed = velocity - self.vel
            if abs(delta_speed) > config.max_delta_speed:
                delta_speed = config.max_delta_speed if delta_speed > 0 else -config.max_delta_speed
            self.vel += delta_speed
        else:
            self.delta = delta
            self.x += self.vel * math.cos(self.yaw) * dt
            self.y += self.vel * math.sin(self.yaw) * dt
            self.yaw += self.vel * math.tan(self.delta) / WB * dt
            self.vel = velocity


# ---------------------------------------------------------------------------
# Pre-instantiated fuzzy controllers (avoids re-creation every step)
# ---------------------------------------------------------------------------
_it2fls_2mf_steering = IT2FLS(
    n_inputs=2, n_mf=2, n_rules=4,
    mf_p=MF_parameters_it2mf2_delta, rules=Rules_it2mf2, c_p=C_parameters_it2mf2_delta,
)
_it2fls_3mf_steering = IT2FLS(
    n_inputs=2, n_mf=3, n_rules=9,
    mf_p=MF_parameters_it2mf3_delta, rules=Rules_it2mf3, c_p=C_parameters_it2mf3_delta,
)
_t1fls_2mf_steering = T1FLS(
    n_inputs=2, n_mf=2, n_rules=4,
    mf_p=MF_parameters_t1mf2_delta, rules=Rules_t1mf2, c_p=C_parameters_t1mf2_delta,
)
_it2fls_2mf_velocity = IT2FLS(
    n_inputs=2, n_mf=2, n_rules=4,
    mf_p=MF_parameters_it2mf2_vel, rules=Rules_it2mf2, c_p=C_parameters_it2mf2_vel,
)
_t1fls_steering_model = T1FLS(
    n_inputs=2, n_mf=2, n_rules=4,
    mf_p=MF_parameters_t1mf2_steering, rules=Rules_t1mf2, c_p=C_parameters_t1mf2_steering,
)


# ---------------------------------------------------------------------------
# Steering control functions
# ---------------------------------------------------------------------------
def steering_geometric_lateral_control():
    return math.atan2((2 * WB * math.sin(params["yaw_error"])) / params["ld"], 1)


def steering_based_on_error():
    return params["yaw_error"]


def steering_it2fls_2mf():
    return _it2fls_2mf_steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))


def steering_it2fls_3mf():
    return _it2fls_3mf_steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))


def steering_t1fls_2mf():
    return _t1fls_2mf_steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))


steering_control_functions = {
    "Geometric Lateral Control": steering_geometric_lateral_control,
    "Error-based controller": steering_based_on_error,
    "T1FLC with 2MF": steering_t1fls_2mf,
    "IT2FLC with 2MF": steering_it2fls_2mf,
    "IT2FLC with 3MF": steering_it2fls_3mf,
}


# ---------------------------------------------------------------------------
# Velocity control functions
# ---------------------------------------------------------------------------
def velocity_pi_control(vel, pi_ctrl):
    vel_err = params["ld"] - look_ahead
    velocity = np.clip(pi_ctrl.control(vel_err), 0, dpg.get_value("speed_slider"))
    return velocity, vel_err


def velocity_pi_control_2(vel, pi_ctrl):
    vel_err = dpg.get_value("speed_slider") - vel
    velocity = np.clip(vel + pi_ctrl.control(vel_err) * dt, 0, dpg.get_value("speed_slider"))
    return velocity, vel_err


def velocity_it2fls_2mf(vel, pi_ctrl):
    vel_err = params["ld"] - look_ahead
    velocity = np.clip(
        _it2fls_2mf_velocity.model(np.array([vel_err, params["d_velocity_error"]])),
        0, dpg.get_value("speed_slider"),
    )
    return velocity, vel_err


velocity_control_functions = {
    "PI Distance Control": velocity_pi_control,
    "PI Velocity Control": velocity_pi_control_2,
    "IT2FLC with 2MF": velocity_it2fls_2mf,
}


# ---------------------------------------------------------------------------
# Steering mode functions
# ---------------------------------------------------------------------------
def mathematical_model(old_delta, new_delta):
    delta_steering = new_delta - old_delta
    if abs(delta_steering) > config.max_delta_steering:
        delta_steering = config.max_delta_steering if delta_steering > 0 else -config.max_delta_steering
    return old_delta + delta_steering


def transfer_function_model(old_delta, new_delta):
    current = old_delta
    for _ in range(5):
        delta_error = new_delta - current
        control_signal = np.clip(41.3006023567974 * delta_error, -255, 255)
        current = current * 0.9989 + 0.001911 * control_signal
    return current


def t1fls_steering_mode(old_delta, new_delta):
    global _previous_delta_error
    current = old_delta
    for _ in range(5):
        delta_error = new_delta - current
        control_signal = np.clip(
            0.8 * delta_error + 0.0001941 * ((delta_error - _previous_delta_error) / dt),
            -1, 1,
        )
        control_signal = np.interp(control_signal, [-1, 1], [-255, 255])
        _previous_delta_error = delta_error
        current = _t1fls_steering_model.model(np.array([current, control_signal]))
    return current


steering_mode_functions = {
    "Steering angle restriction": mathematical_model,
    "Transfer function model": transfer_function_model,
    "T1FLS": t1fls_steering_mode,
}


# ---------------------------------------------------------------------------
# UI callback functions
# ---------------------------------------------------------------------------
def fit_interface_size(sender, app_data):
    config.width = dpg.get_viewport_width()
    config.height = dpg.get_viewport_height()
    if dpg.does_item_exist("window"): dpg.configure_item("window", width=config.width, height=config.height)
    if dpg.does_item_exist("left_panel"): dpg.configure_item("left_panel", width=340)
    if dpg.does_item_exist("plot_container"): dpg.configure_item("plot_container", width=config.width - 370, height=config.height - 30)

def update_path(sender, app_data):
    selection = app_data
    x_data, y_data = zip(*paths[selection])
    if dpg.does_item_exist("path_plot"):
        dpg.set_value("path_plot", [x_data, y_data])
        dpg.bind_item_theme("path_plot", "path_theme")
        dpg.fit_axis_data("xaxis")
        dpg.fit_axis_data("yaxis")
    update_vehicle(x_data[0] - 2, y_data[0] - 2, 1.5708, 0)

def update_steering_control(sender, app_data):
    global _steering_control_func
    _steering_control_func = steering_control_functions[app_data]
    # Update Fuzzy Plots if applicable
    if "IT2FLC" in app_data:
        ctrl = _it2fls_3mf_steering if "3MF" in app_data else _it2fls_2mf_steering
        plot_mfs(ctrl, 0, "fuzzy_plot_1")
        plot_mfs(ctrl, 1, "fuzzy_plot_2")
    elif "T1FLC" in app_data:
        plot_mfs(_t1fls_2mf_steering, 0, "fuzzy_plot_1")
        plot_mfs(_t1fls_2mf_steering, 1, "fuzzy_plot_2")

def update_velocity_control(sender, app_data):
    global _velocity_control_func
    _velocity_control_func = velocity_control_functions[app_data]

def update_steering_mode(sender, app_data):
    global _steering_mode_func
    _steering_mode_func = steering_mode_functions[app_data]
    show = (app_data == "Steering angle restriction")
    if dpg.does_item_exist("max_steering_delta"):
        dpg.configure_item("max_steering_delta", show=show)
        dpg.configure_item("max_steering_delta_text", show=show)

def toggle_noise_settings(sender, app_data):
    val = dpg.get_value("noise_checkbox")
    items = ["max_speed_delta", "max_speed_delta_text", "steering"]
    for item in items:
        if dpg.does_item_exist(item): dpg.configure_item(item, show=val)
    if val and dpg.get_value("steering") == "Steering angle restriction":
        if dpg.does_item_exist("max_steering_delta"):
            dpg.configure_item("max_steering_delta", show=True)
            dpg.configure_item("max_steering_delta_text", show=True)
    elif not val:
        if dpg.does_item_exist("max_steering_delta"):
            dpg.configure_item("max_steering_delta", show=False)
            dpg.configure_item("max_steering_delta_text", show=False)


# ---------------------------------------------------------------------------
# Items to disable/enable during simulation
# ---------------------------------------------------------------------------
_SIM_UI_ITEMS = [
    "speed_slider", "noise_checkbox", "run_button", "path",
    "control", "max_steering_delta", "max_speed_delta", "speed", "steering",
]


def _set_sim_ui_enabled(enabled):
    fn = dpg.enable_item if enabled else dpg.disable_item
    for item in _SIM_UI_ITEMS:
        fn(item)


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
def run_simulation(return_errors=False, n_iterations=1):
    config.max_delta_steering = math.radians(dpg.get_value("max_steering_delta"))
    config.max_delta_speed = dpg.get_value("max_speed_delta")

    _set_sim_ui_enabled(False)

    x_data, y_data = dpg.get_value("path_plot")
    path = np.stack((x_data, y_data), axis=-1)
    traj = Trajectory(x_data, y_data)
    goal = traj.getPoint(len(x_data) - 1)

    ego = Mathematic_Model(
        x_data[0] - 3, y_data[0] - 3,
        math.atan2(y_data[1] - y_data[0], x_data[1] - x_data[0]),
    )
    pi_ctrl = PI()

    traj_ego_x, traj_ego_y = [], []
    lateral_error_list = []
    heading_error_list = []
    velocity_error_list = []
    mse_lateral = 0.0
    mse_heading = 0.0
    speed_list = []
    delta_list = []
    index = 0
    last_near_idx = 0  # For local nearest-point search
    tracking_started = False

    # Define file path to save results
    if not return_errors:
        subfolder = (
            f'{abbreviate_name(dpg.get_value("steering")) + "_" if dpg.get_value("noise_checkbox") else ""}'
            f'{abbreviate_name(dpg.get_value("speed"))}'
        )
        folder = f'./results/simple/{dpg.get_value("path")}{"WN" if dpg.get_value("noise_checkbox") else ""}/{subfolder}'
        txt_path = f'{folder}.txt'
        file_path = f'{folder}/{subfolder}_{abbreviate_name(dpg.get_value("control"))}.csv'
        image_path = f'{folder}/{subfolder}_{abbreviate_name(dpg.get_value("control"))}.png'
    else:
        subfolder = (
            f'{abbreviate_name(dpg.get_value("steering")) + "_" if dpg.get_value("noise_checkbox") else ""}'
            f'{abbreviate_name(dpg.get_value("speed"))}'
        )
        folder = f'./results/multi/{dpg.get_value("path")}{"WN" if dpg.get_value("noise_checkbox") else ""}/{subfolder}'
        txt_path = f'{folder}.txt'
        path_abbr = abbreviate_name(dpg.get_value("path"))
        ctrl_abbr = abbreviate_name(dpg.get_value("control"))
        file_path = f'{folder}/{path_abbr}_{subfolder}_{ctrl_abbr}.csv'
        image_path = f'{folder}/{path_abbr}_{subfolder}_{ctrl_abbr}.png'

    os.makedirs(folder, exist_ok=True)
    diverged = False
    
    # Initialize error metrics to 0 or "-" before the loop to prevent UnboundLocalError
    mse_lateral = 0.0; mae_lateral = 0.0; rmse_lateral = 0.0
    mse_heading = 0.0; mae_heading = 0.0; rmse_heading = 0.0

    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "theta", "v", "delta", "le", "mse_le", "mae_le", "rmse_le", "he", "mse_he", "mae_he", "rmse_he"])

        while getDistance([ego.x, ego.y], goal) > 1 or index < len(x_data) - 2:
            if abs(params["yaw_error"]) > 2:
                mse_lateral = mae_lateral = rmse_lateral = "-"
                mse_heading = mae_heading = rmse_heading = "-"
                diverged = True
                break

            step_start = time.perf_counter()
            target_point, current, index = traj.getTargetPoint([ego.x, ego.y])
            params["ld"] = current

            # Velocity control
            velocity, vel_err = _velocity_control_func(ego.vel, pi_ctrl)
            velocity_error_list.append(vel_err)
            params["velocity_error"] = velocity_error_list[-1]
            params["d_velocity_error"] = ((velocity_error_list[-1] - (velocity_error_list[-2] if len(velocity_error_list) > 1 else 0)) / dt)

            # Nearest point search
            search_start = max(0, last_near_idx - 5)
            search_end = min(len(path), last_near_idx + 20)
            local_distances = np.sqrt((path[search_start:search_end, 0] - ego.x) ** 2 + (path[search_start:search_end, 1] - ego.y) ** 2)
            index_near_point = search_start + np.argmin(local_distances)
            last_near_idx = index_near_point
            nearest_dist = local_distances[index_near_point - search_start]

            # Calculate Errors
            yaw_err_mp = math.atan2(path[index_near_point, 1] - ego.y, path[index_near_point, 0] - ego.x) - ego.yaw
            yaw_err_mp = np.arctan2(np.sin(yaw_err_mp), np.cos(yaw_err_mp))
            l_err = nearest_dist * math.sin(yaw_err_mp)
            
            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

            # 🚀 Logic: Start recording errors only when within tracking zone (<0.5m)
            if not tracking_started and nearest_dist < 0.5:
                tracking_started = True
            
            if tracking_started:
                lateral_error_list.append(abs(l_err))
                heading_error_list.append(abs(yaw_err))
                
                # Lateral Metrics (m)
                mse_lateral = np.mean(np.square(lateral_error_list))
                mae_lateral = np.mean(lateral_error_list)
                rmse_lateral = math.sqrt(mse_lateral)
                
                # Heading Metrics (deg) - More precise: compute in rad then convert
                mse_heading_rad = np.mean(np.square(heading_error_list))
                mse_heading = math.degrees(mse_heading_rad) # Keep for legacy display if needed
                mae_heading = math.degrees(np.mean(heading_error_list))
                rmse_heading = math.degrees(math.sqrt(mse_heading_rad))

            params["yaw_error"] = yaw_err
            params["d_yaw_error"] = ((yaw_err - (heading_error_list[-2] if len(heading_error_list) > 1 else 0)) / dt)

            # Steering Control & Firing Capture
            ctrl_val = dpg.get_value("control")
            if "IT2FLC" in ctrl_val or "T1FLC" in ctrl_val:
                if "IT2" in ctrl_val:
                    ctrl_obj = _it2fls_3mf_steering if "3MF" in ctrl_val else _it2fls_2mf_steering
                else:
                    ctrl_obj = _t1fls_2mf_steering
                delta, firing = ctrl_obj.model(np.array([params["yaw_error"], params["d_yaw_error"]]), return_firing=True)
            else:
                delta = np.clip(_steering_control_func(), -0.698132, 0.698132)
                firing = None

            ego.update(velocity, delta)

            speed_list.append(ego.vel)
            delta_list.append(math.degrees(ego.delta))
            traj_ego_x.append(ego.x)
            traj_ego_y.append(ego.y)

            writer.writerow([
                ego.x, ego.y, math.degrees(ego.yaw), ego.vel, math.degrees(ego.delta),
                abs(l_err), 
                f'{mse_lateral:.6f}' if not isinstance(mse_lateral, str) else mse_lateral,
                f'{mae_lateral:.6f}' if not isinstance(mae_lateral, str) else mae_lateral,
                f'{rmse_lateral:.6f}' if not isinstance(rmse_lateral, str) else rmse_lateral,
                math.degrees(abs(yaw_err)),
                f'{mse_heading:.6f}' if not isinstance(mse_heading, str) else mse_heading,
                f'{mae_heading:.6f}' if not isinstance(mae_heading, str) else mae_heading,
                f'{rmse_heading:.6f}' if not isinstance(rmse_heading, str) else rmse_heading
            ])

            # Update interface display
            def safe_set(tag, val):
                if dpg.does_item_exist(tag): dpg.set_value(tag, val)

            safe_set("x", f"{ego.x:.2f} m")
            safe_set("y", f"{ego.y:.2f} m")
            safe_set("theta", f"{math.degrees(ego.yaw):.2f}°")
            safe_set("v", f"{ego.vel:.2f} m/s")
            safe_set("delta", f"{math.degrees(ego.delta):.2f}°")
            safe_set("le", f"{abs(l_err):.4f} m")
            safe_set("msele", f"{mse_lateral:.4f}" if not isinstance(mse_lateral, str) else f"{mse_lateral}")
            safe_set("he", f"{math.degrees(abs(yaw_err)):.2f}°")
            safe_set("msehe", f"{mse_heading:.4f}" if not isinstance(mse_heading, str) else f"{mse_heading}")

            # Update Fuzzy Plots
            if firing is not None:
                # Plot 1: Input 0 (Yaw Error), offset 0
                # Plot 2: Input 1 (d_Yaw Error), offset = n_mf
                n_mf_current = 3 if "3MF" in dpg.get_value("control") else 2
                for p_idx, input_val, offset in [("1", params["yaw_error"], 0), ("2", params["d_yaw_error"], n_mf_current)]:
                    for i in range(n_mf_current):
                        f_tag = f"fuzzy_plot_{p_idx}_firing_{i}"
                        if dpg.does_item_exist(f_tag):
                            # Ensure we don't exceed firing list size
                            if (offset + i) < len(firing):
                                # IT2 has [lower, upper], T1 has [val, val]
                                avg_f = (firing[offset+i][0] + firing[offset+i][1]) / 2.0
                                dpg.set_value(f_tag, [[input_val], [avg_f]])

            update_vehicle(ego.x, ego.y, ego.yaw, ego.delta)
            if dpg.does_item_exist("reference_plot"):
                dpg.set_value("reference_plot", [traj_ego_x, traj_ego_y])
            dpg.bind_item_theme("reference_plot", "reference_theme")
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")

            # Accurate timing: account for computation time
            elapsed = time.perf_counter() - step_start
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # Reset control parameters
    params["yaw_error"] = 0.0
    params["d_yaw_error"] = 0.0
    params["velocity_error"] = 0.0
    params["d_velocity_error"] = 0.0
    params["ld"] = 0.0

    _set_sim_ui_enabled(True)

    screenshot = pyautogui.screenshot(region=(420, 185, 1150, 800))
    screenshot.save(image_path)

    if return_errors:
        return (
            (mse_lateral, mse_heading),
            (mae_lateral, mae_heading),
            (rmse_lateral, rmse_heading)
        ), folder, txt_path


def generate_thesis_reports(results, path_name, folder_path, metric_type="RMSE"):
    """Generates an Excel-friendly CSV and a LaTeX table code for the results."""
    # Mappings for professional names
    MAP_VEL = {"PI Distance Control": "PIDC", "PI Velocity Control": "PIVC", "IT2FLC with 2MF": "IT2FLC 2MF"}
    MAP_MODE = {"Steering angle restriction": "Restricción del ángulo de dirección", "Transfer function model": "Modelo en función de transferencia", "T1FLS": "T1FLS"}
    MAP_CTRL = {"Geometric Lateral Control": "GLC", "Error-based controller": "EBC", "T1FLC with 2MF": "T1FLC 2MF", "IT2FLC with 2MF": "IT2FLC 2MF", "IT2FLC with 3MF": "IT2FLC 3MF"}
    
    csv_path = os.path.join(folder_path, f"Tabla_Resultados_{metric_type}_{path_name}.csv")
    tex_path = os.path.join(folder_path, f"Tabla_Resultados_{metric_type}_{path_name}.tex")
    
    # --- Generate CSV ---
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        header = ["Modelo del volante", "Control de velocidad", f"Métrica: {metric_type}"] + list(MAP_CTRL.values())
        writer.writerow(header)
        for mode in steering_mode:
            for i, vel in enumerate(velocity_control):
                row = [MAP_MODE[mode] if i == 0 else "", MAP_VEL[vel], ""]
                for ctrl in steering_control:
                    val = results.get((mode, vel, ctrl), ("-", "-"))
                    row.append(f"{val[0]} / {val[1]}")
                writer.writerow(row)

    # --- Generate LaTeX ---
    with open(tex_path, 'w', encoding='utf-8') as f:
        f.write("% Requiere: \\usepackage{multirow}, \\usepackage{booktabs}, \\usepackage[table]{xcolor}\n")
        f.write("\\begin{table}[ht]\n\\centering\n")
        f.write(f"\\caption{{{metric_type} del error lateral y de cabeceo en la {path_name} bajo condiciones de ruido}}\n")
        f.write("\\small\n\\begin{tabular}{ll" + "c" * len(steering_control) + "}\n\\toprule\n")
        f.write("\\multirow{2}{*}{Modelo del volante} & \\multirow{2}{*}{Control de velocidad} & \\multicolumn{" + str(len(steering_control)) + "}{c}{Control del volante} \\\\ \\cmidrule{3-" + str(len(steering_control)+2) + "}\n")
        f.write(" & & " + " & ".join(MAP_CTRL.values()) + " \\\\\n\\midrule\n")
        
        for mode in steering_mode:
            f.write(f"\\multirow{{3}}{{*}}{{{MAP_MODE[mode]}}}")
            for i, vel in enumerate(velocity_control):
                f.write(f" & {MAP_VEL[vel]}")
                for ctrl in steering_control:
                    v = results.get((mode, vel, ctrl), ("-", "-"))
                    if v[0] == "-": f.write(" & -")
                    else: 
                        unit_l = "m" if metric_type != "MSE" else "m^2"
                        unit_h = "^\\circ" if metric_type != "MSE" else "^\\circ^2"
                        f.write(f" & \\begin{{tabular}}[c]{{@{{}}c@{{}}}}{v[0]} {unit_l} \\\\ {v[1]}{unit_h}\\end{{tabular}}")
                f.write(" \\\\\n")
                if i < len(velocity_control) - 1: f.write(" & ")
            f.write("\\midrule\n")
        f.write("\\bottomrule\n\\end{tabular}\n\\end{table}")


def run_multiple_simulations():
    dpg.set_value("noise_checkbox", True)
    
    for i in range(len(list_paths)):
        path_name = list_paths[i]
        dpg.set_value("path", path_name)
        update_path("", path_name)
        
        mse_res, mae_res, rmse_res = {}, {}, {}
        last_folder = ""
        
        for j in range(len(steering_mode)):
            mode_name = steering_mode[j]
            dpg.set_value("steering", mode_name)
            update_steering_mode("", mode_name)
            
            for k in range(len(velocity_control)):
                vel_name = velocity_control[k]
                dpg.set_value("speed", vel_name)
                update_velocity_control("", vel_name)
                
                for l_idx in range(len(steering_control)):
                    ctrl_name = steering_control[l_idx]
                    dpg.set_value("control", ctrl_name)
                    update_steering_control("", ctrl_name)
                    
                    # Capture triple metric set
                    errs_set, folder, _ = run_simulation(return_errors=True, n_iterations=i)
                    last_folder = folder
                    
                    key = (mode_name, vel_name, ctrl_name)
                    
                    # Store MSE (errs_set[0]), MAE (errs_set[1]), RMSE (errs_set[2])
                    for metrics, storage in zip(errs_set, [mse_res, mae_res, rmse_res]):
                        if isinstance(metrics[0], (int, float)):
                            storage[key] = (f"{metrics[0]:.4f}", f"{metrics[1]:.4f}")
                        else:
                            storage[key] = ("-", "-")
        
        # Build 3 reports per path
        if last_folder:
            generate_thesis_reports(mse_res, path_name, last_folder, "MSE")
            generate_thesis_reports(mae_res, path_name, last_folder, "MAE")
            generate_thesis_reports(rmse_res, path_name, last_folder, "RMSE")


# ---------------------------------------------------------------------------
# Threaded wrappers (keeps GUI responsive during simulation)
# ---------------------------------------------------------------------------
def run_simulation_callback(sender=None, app_data=None):
    """Button callback — runs simulation in a background thread."""
    thread = threading.Thread(target=run_simulation, daemon=True)
    thread.start()


def run_multiple_callback(sender=None, app_data=None):
    """Button callback — runs multiple simulations in a background thread."""
    thread = threading.Thread(target=run_multiple_simulations, daemon=True)
    thread.start()