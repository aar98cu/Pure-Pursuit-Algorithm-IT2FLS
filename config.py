import numpy as np
import dearpygui.dearpygui as dpg

# Interface parameters
# These parameters define the size of the application window.
width = 1500 # Window width in pixels
height = 900 # Window height in pixels

# Vehicle parameters in meters
# These parameters define the physical characteristics of the vehicle model.
LENGTH = 2.2           # Length of the vehicle
WIDTH = 1.2              # Width of the vehicle
BACKTOWHEEL = 0.5      # Distance from the back of the vehicle to the rear wheel
WHEEL_LEN = 0.3        # Length of the wheel
WHEEL_WIDTH = 0.2      # Width of the wheel
TREAD = 0.4            # Distance between the left and right wheels (track width)
WB = 1.3               # Wheelbase (distance between the front and rear axles)

# Simulation parameters
# Parameters controlling the behavior of the simulation.
look_ahead = 1.3               # Look-ahead distance for path tracking
dt = 0.1                       # Time step for simulation updates (in seconds)
noise_std_dev = 0.02           # Standard deviation of the Gaussian noise added to measurements
max_delta_speed = 0.05         # Maximum change in speed per time step (in meters per second)
max_delta_steering = 0.0523599 # Maximum change in steering angle per time step (in radians)
noise_matrix = np.round(np.random.uniform(-noise_std_dev, noise_std_dev, 100000), 4)

# Steering control variables
# Lists available steering control algorithms and initializes the selected control function to None.
steering_control = ["Geometric Lateral Control", "Error-based controller", "T1FLC with 2MF", "IT2FLC with 2MF", "IT2FLC with 3MF"]
steering_control_func = None  # Function pointer for the selected steering control method

# Velocity control variables
# Lists available velocity control algorithms and initializes the selected control function to None.
velocity_control = ["PI Distance Control", "PI Velocity Control", "IT2FLC with 2MF"]
velocity_control_func = None  # Function pointer for the selected velocity control method

# Control parameters
# This dictionary holds the state variables for the control algorithms.
params = {
    "yaw_error": 0.0,           # Yaw angle error
    "d_yaw_error": 0.0,         # Derivative of yaw angle error
    "velocity_error": 0.0,      # Velocity error
    "d_velocity_error": 0.0,    # Derivative of velocity error
    "ld": 0.0,                  # Look-ahead distance (dynamic during simulation)
}

# Interface color parameters
# Defines the colors and sizes for different elements in the simulation interface.
PATH_THEME_COLOR = (0, 0, 255)              # Color for the vehicle's planned path (blue)
REFERENCE_THEME_COLOR = (255, 0, 0)         # Color for the reference path (red)
VEHICLE_THEME_COLOR = (0, 0, 0)             # Color for the vehicle representation (black)
ORIGIN_MARKER_SIZE = 15                     # Size of the origin marker
ORIGIN_MARKER_COLOR = (255, 0, 0)           # Color of the origin marker (red)

# x, y coordinates of the different paths
# These lists define various pre-set paths that the vehicle can follow during the simulation.
path_1 = [(-1.618, 4.475), (-1.425, 4.980), (-1.233, 5.486), (-1.040, 5.992), (-0.848, 6.497), (-0.656, 7.003), (-0.463, 7.509), (-0.271, 8.014), (-0.079, 8.520), (0.113, 9.026), (0.305, 9.531), (0.498, 10.037), (0.690, 10.543), (0.882, 11.048), (1.075, 11.554), (1.267, 12.060), (1.459, 12.565), (1.652, 13.071), (1.844, 13.577), (2.036, 14.082), (2.229, 14.588), (2.421, 15.094), (2.614, 15.600)]
path_2 = [(-1.618000e+00, 4.475000e+00), (-1.219813e+00, 5.512312e+00), (-8.216268e-01, 6.549623e+00), (-4.234402e-01, 7.586935e+00), (-2.525356e-02, 8.624246e+00), (3.729331e-01, 9.661558e+00), (7.711197e-01, 1.069887e+01), (1.169306e+00, 1.173618e+01), (1.567493e+00, 1.277349e+01), (1.965679e+00, 1.381080e+01), (2.363866e+00, 1.484812e+01), (2.562668e+00, 1.587086e+01), (2.399681e+00, 1.689992e+01), (1.894563e+00, 1.781118e+01), (1.108239e+00, 1.849472e+01), (1.355518e-01, 1.886810e+01), (-9.061786e-01, 1.888628e+01), (-1.891304e+00, 1.854708e+01), (-2.701004e+00, 1.789140e+01), (-3.237616e+00, 1.699832e+01), (-3.237616e+00, 1.699832e+01), (-3.635803e+00, 1.596101e+01), (-4.033990e+00, 1.492370e+01), (-4.432176e+00, 1.388639e+01), (-4.830363e+00, 1.284908e+01), (-5.228550e+00, 1.181177e+01), (-5.626736e+00, 1.077445e+01), (-6.024923e+00, 9.737142e+00), (-6.423109e+00, 8.699831e+00), (-6.821296e+00, 7.662519e+00), (-7.219483e+00, 6.625208e+00), (-7.756095e+00, 5.732134e+00), (-8.565795e+00, 5.076452e+00), (-9.550920e+00, 4.737247e+00), (-1.059265e+01, 4.755430e+00), (-1.156534e+01, 5.128810e+00), (-1.235166e+01, 5.812350e+00), (-1.285678e+01, 6.723607e+00), (-1.301977e+01, 7.752669e+00), (-1.282097e+01, 8.775415e+00), (-1.282097e+01, 8.775415e+00), (-1.242278e+01, 9.812727e+00), (-1.202459e+01, 1.085004e+01), (-1.162641e+01, 1.188735e+01), (-1.122822e+01, 1.292466e+01), (-1.083003e+01, 1.396197e+01), (-1.043185e+01, 1.499928e+01), (-1.003366e+01, 1.603660e+01), (-9.635472e+00, 1.707391e+01), (-9.237286e+00, 1.811122e+01), (-8.839099e+00, 1.914853e+01), (-8.640297e+00, 2.017128e+01), (-8.803285e+00, 2.120034e+01), (-9.308402e+00, 2.211160e+01), (-1.009473e+01, 2.279514e+01), (-1.106741e+01, 2.316852e+01), (-1.210914e+01, 2.318670e+01), (-1.309427e+01, 2.284749e+01), (-1.390397e+01, 2.219181e+01), (-1.444058e+01, 2.129874e+01)]
path_3 = [(-1.618000e+00, 4.475000e+00), (-1.106046e+00, 5.808686e+00), (-5.940916e-01, 7.142373e+00), (-8.213736e-02, 8.476059e+00), (4.298169e-01, 9.809745e+00), (9.417711e-01, 1.114343e+01), (1.453725e+00, 1.247712e+01), (1.965679e+00, 1.381080e+01), (2.477634e+00, 1.514449e+01), (2.989588e+00, 1.647818e+01), (3.501542e+00, 1.781186e+01), (4.013496e+00, 1.914555e+01), (4.525451e+00, 2.047924e+01), (5.037405e+00, 2.181292e+01), (5.549359e+00, 2.314661e+01), (6.061313e+00, 2.448029e+01), (6.273368e+00, 2.557122e+01), (6.099515e+00, 2.666889e+01), (5.560723e+00, 2.764090e+01), (4.721978e+00, 2.837001e+01), (3.684445e+00, 2.876828e+01), (2.573266e+00, 2.878767e+01), (1.522465e+00, 2.842586e+01), (6.587852e-01, 2.772646e+01), (8.639847e-02, 2.677385e+01), (8.639847e-02, 2.677385e+01), (-4.255557e-01, 2.544016e+01), (-9.375100e-01, 2.410648e+01), (-1.449464e+00, 2.277279e+01), (-1.961418e+00, 2.143910e+01), (-2.473373e+00, 2.010542e+01), (-2.985327e+00, 1.877173e+01), (-3.497281e+00, 1.743805e+01), (-4.009235e+00, 1.610436e+01), (-4.521189e+00, 1.477067e+01), (-5.033144e+00, 1.343699e+01), (-5.545098e+00, 1.210330e+01), (-6.057052e+00, 1.076961e+01), (-6.569006e+00, 9.435928e+00), (-7.080961e+00, 8.102241e+00), (-7.592915e+00, 6.768555e+00), (-8.165301e+00, 5.815943e+00), (-9.028981e+00, 5.116549e+00), (-1.007978e+01, 4.754730e+00), (-1.119096e+01, 4.774125e+00), (-1.222849e+01, 5.172397e+00), (-1.306724e+01, 5.901507e+00), (-1.360603e+01, 6.873514e+00), (-1.377988e+01, 7.971180e+00), (-1.356783e+01, 9.062110e+00), (-1.356783e+01, 9.062110e+00), (-1.305588e+01, 1.039580e+01), (-1.254392e+01, 1.172948e+01), (-1.203197e+01, 1.306317e+01), (-1.152001e+01, 1.439686e+01), (-1.100806e+01, 1.573054e+01), (-1.049610e+01, 1.706423e+01), (-9.984150e+00, 1.839791e+01), (-9.472196e+00, 1.973160e+01), (-8.960242e+00, 2.106529e+01), (-8.448287e+00, 2.239897e+01), (-7.936333e+00, 2.373266e+01), (-7.424379e+00, 2.506635e+01), (-6.912425e+00, 2.640003e+01), (-6.400470e+00, 2.773372e+01), (-5.888516e+00, 2.906740e+01), (-5.676461e+00, 3.015833e+01), (-5.850314e+00, 3.125600e+01), (-6.389107e+00, 3.222801e+01), (-7.227852e+00, 3.295712e+01), (-8.265385e+00, 3.335539e+01), (-9.376564e+00, 3.337478e+01), (-1.042736e+01, 3.301297e+01), (-1.129104e+01, 3.231357e+01), (-1.186343e+01, 3.136096e+01)]
path_4 = [(-5.625688e+00, 3.653988e+00), (-3.725041e+00, 3.804284e+00), (-2.036216e+00, 4.625236e+00), (-7.655605e-01, 5.935859e+00), (1.153492e-02, 7.512786e+00), (3.257931e-01, 9.167440e+00), (2.617824e-01, 1.077757e+01), (-8.496859e-02, 1.228265e+01), (-6.305375e-01, 1.366549e+01), (-1.309578e+00, 1.493431e+01), (-2.074219e+00, 1.611029e+01), (-2.890030e+00, 1.722033e+01), (-3.731504e+00, 1.829340e+01), (-4.577879e+00, 1.935907e+01), (-5.409320e+00, 2.044694e+01), (-6.203189e+00, 2.158640e+01), (-6.930067e+00, 2.280568e+01), (-7.549370e+00, 2.412944e+01), (-8.004895e+00, 2.557361e+01), (-8.221799e+00, 2.713560e+01), (-8.108696e+00, 2.877905e+01), (-7.571454e+00, 3.041447e+01), (-6.545930e+00, 3.188530e+01), (-5.048499e+00, 3.297860e+01), (-3.220564e+00, 3.348041e+01), (-1.322001e+00, 3.326616e+01), (3.478029e-01, 3.236955e+01), (1.563877e+00, 3.096999e+01), (2.235770e+00, 2.930758e+01), (2.394978e+00, 2.759356e+01), (2.138928e+00, 2.596624e+01), (1.579329e+00, 2.449189e+01), (8.133731e-01, 2.318559e+01), (-8.540448e-02, 2.203316e+01), (-1.065760e+00, 2.100657e+01), (-2.093633e+00, 2.007272e+01), (-3.146636e+00, 1.919753e+01), (-4.209245e+00, 1.834732e+01), (-5.268727e+00, 1.748880e+01), (-6.311432e+00, 1.658852e+01), (-7.318951e+00, 1.561256e+01), (-8.263716e+00, 1.452701e+01), (-9.103781e+00, 1.330056e+01), (-9.777278e+00, 1.191062e+01), (-1.019856e+01, 1.035527e+01), (-1.026101e+01, 8.672193e+00), (-9.854947e+00, 6.961729e+00), (-8.908381e+00, 5.400860e+00), (-7.445023e+00, 4.224139e+00), (-5.625688e+00, 3.653988e+00)]

# Dictionary of the different types of path
# Maps path names to their corresponding coordinate lists.
paths = {
    "Path 1": path_1,
    "Path 2": path_2,
    "Path 3": path_3,
    "Path 4": path_4,
}
list_paths = ["Path 1", "Path 2", "Path 3", "Path 4"]  # List of available paths for selection

# Steering mode variables
# Lists available steering mode and initializes the selected control function to None.
steering_mode = ["Steering angle restriction", "Transfer function model", "T1FLS"]
steering_mode_func = None  # Function pointer for the selected steering control method
previous_delta_error = 0

# Constants for an interval-type 2 fuzzy steering wheel controller with 2 membership functions
# Parameters for an interval-type 2 fuzzy logic system (IT2FLS) with 2 membership functions.
MF_parameters_it2mf2_delta = np.array([[0.091044, 0.03161, 1.0488, -0.21707, 0.15446],
    [0.076239, 0.18334, 1.0264, 0.37033, 0.78535],
    [1.3177, 1.5725, 1.0069, -4.618, 0.45283],
    [1.3091, 1.577, 1.0135, 1.9458, 0.46793]])

C_parameters_it2mf2_delta = np.array([[-0.50804, 0.12629, -0.44446],
    [1.8885, -0.18666, 0.070893],
    [1.8154, -0.1777, 0.19092],
    [0.79655, 0.081774, -0.049842]])

Rules_it2mf2 = np.array([[2, 4],
    [2, 5],
    [3, 4],
    [3, 5]])

# Constants for the IT2 fuzzy steering wheel controller with 3 membership functions
# Parameters for an IT2FLS with 3 membership functions.
MF_parameters_it2mf3_delta = np.array([[0.16207, 0.18457, 1.0085, -0.43013, 0.37617],
    [0.072534, 0.091516, 1.0262, -0.02401, 0.3882],
    [0.022101, 0.02803, 1.0275, 0.36549, 0.50255],
    [1.3126, 1.5771, 0.99528, -4.6208, 0.54202],
    [1.278, 1.5463, 1.0527, -1.3363, 0.40656],
    [1.2884, 1.5522, 1.0617, 1.9235, 0.42364]])

C_parameters_it2mf3_delta = np.array([[1.6165, 0.043378, 0.3826],
    [1.0109, -0.0075928, 0.047905],
    [1.3223, -0.032349, -0.0010639],
    [3.7929, -0.072392, -0.40528],
    [3.0834, -0.36098, -0.67003],
    [1.0144, -0.70076, -0.11468],
    [2.4935, 0.10978, -0.61202],
    [2.9009, -0.90953, -0.95173],
    [0.26377, 0.26037, 0.53858]])

Rules_it2mf3 = np.array([[2, 5],
    [2, 6],
    [2, 7],
    [3, 5],
    [3, 6],
    [3, 7],
    [4, 5],
    [4, 6],
    [4, 7]])

# Constants for the type 1 fuzzy steering wheel controller with 2 membership functions
# Parameters for a Type-1 Fuzzy Logic System (T1FLS) with 2 membership functions.
MF_parameters_t1mf2_delta = np.array([[0.5167, 1.985, -0.3764],
    [0.4749, 2.015, 0.2903],
    [3.288, 2.005, -4.614],
    [3.287, 1.994, 1.952]])

C_parameters_t1mf2_delta = np.array([[-12.44, 0.6913, -6.652],
    [-1.47, -0.1517, -1.5],
    [-9.214, -3.533, 2.268],
    [-1.3, -0.0705, 2.163]])

Rules_t1mf2 = np.array([[0, 2],
    [0, 3],
    [1, 2],
    [1, 3],])


# Constants for the IT2 fuzzy velocity controller with 2 membership functions for velocity control
# Parameters for an IT2FLS used specifically for velocity control with 2 membership functions.
MF_parameters_it2mf2_vel = np.array([[0.72631, 0.87158, 1, -0.17305, 0.5],
    [0.72631, 0.87158, 1, 3.4585, 0.5],
    [7.317, 8.7805, 1, -2, 0.5],
    [7.31705, 8.78046, 1, 34.5852, 0.5]])

C_parameters_it2mf2_vel = np.array([[-1.17582, -2.04875, -12.8161],
    [-23.4196, -2.02625, 87.2486],
    [-2.9969, 3.11659, 26.9546],
    [-38.3781, 3.18461, 19.6162]])


# Constants for the type 1 fuzzy steering wheel model with 2 membership functions
# Parameters for a Type-1 Fuzzy Logic System (T1FLS) with 2 membership functions.
MF_parameters_t1mf2_steering = np.array([[0.5167, 1.985, -0.3764],
    [0.4749, 2.015, 0.2903],
    [242.5, 2.17, -243],
    [242.5, 1.654, 242]])

C_parameters_t1mf2_steering = np.array([[0.998, 0.001682, 0.179],
    [0.9973, 0.002515, -0.3191],
    [1.01, 0.004261, 0.2992],
    [0.9853, 0.003636, -0.206]])


# Theme for the vehicle
# Sets the visual style for the vehicle's representation in the plot.
with dpg.theme(tag="vehicle_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, VEHICLE_THEME_COLOR, category=dpg.mvThemeCat_Plots)
        dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 8, category=dpg.mvThemeCat_Plots)

# Theme for the path
# Sets the visual style for the path that the vehicle will follow.
with dpg.theme(tag="path_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, PATH_THEME_COLOR, category=dpg.mvThemeCat_Plots)
        dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 8, category=dpg.mvThemeCat_Plots)

# Theme for the reference path
# Sets the visual style for the reference path.
with dpg.theme(tag="reference_theme"):
    with dpg.theme_component(dpg.mvLineSeries):
        dpg.add_theme_color(dpg.mvPlotCol_Line, REFERENCE_THEME_COLOR, category=dpg.mvThemeCat_Plots)
        dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 8, category=dpg.mvThemeCat_Plots)

# General interface theme
# Defines the color scheme and style for the application interface.
with dpg.theme(tag="interface_theme"):
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (255, 255, 255, 255))
        dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_Border, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_TitleBg, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (0, 0, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (0, 0, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_BorderShadow, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (255, 255, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 255, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 255, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_Button, (150, 150, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (255, 255, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_CheckMark, (0, 0, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (200, 200, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (255, 255, 0, 255))
        dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (255, 255, 0, 255))
        dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 10, 10)
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
        dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 10, 10)

with dpg.theme(tag="plot_theme"):
    with dpg.theme_component(dpg.mvPlot):
        dpg.add_theme_color(dpg.mvPlotCol_FrameBg, (255, 255, 255, 255), category=dpg.mvThemeCat_Plots)
        dpg.add_theme_color(dpg.mvPlotCol_PlotBg, (255, 255, 255, 255), category=dpg.mvThemeCat_Plots)
        dpg.add_theme_color(dpg.mvPlotCol_PlotBorder, (0, 0, 0, 255), category=dpg.mvThemeCat_Plots)

# Global font scale
# Sets the global font scale to 1.5 for all text elements in the interface.
dpg.set_global_font_scale(1.5)