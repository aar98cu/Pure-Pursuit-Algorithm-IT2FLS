import dearpygui.dearpygui as dpg
import numpy as np
import math

paths = ["Trayectoria 1", "Trayectoria 2", "Trayectoria 3"]
control = ["Control 1", "Control 2", "Control 3"]
width = 1200
height = 800

# Vehicle parameters (m)
LENGTH = 2.2 #4.5
WIDTH = 1 #2.0
BACKTOWHEEL = 0.5 #1.0
WHEEL_LEN = 0.3 #0.3
WHEEL_WIDTH = 0.2 #0.2
TREAD = 0.4 #0.1
WB = 1.3 #2.5

def plotVehicle(x, y, yaw, steer=0.0):
    """
    The function is to plot the vehicle
    it is copied from https://github.com/AtsushiSakai/PythonRobotics/blob/187b6aa35f3cbdeca587c0abdb177adddefc5c2a/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L109
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

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array(
        [[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]]
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
    dpg.add_scatter_series([x], [y], parent="yaxis", tag="origin")

    dpg.bind_item_theme("vehicle", "plot_theme")
    dpg.bind_item_theme("fr_wheel", "plot_theme")
    dpg.bind_item_theme("rr_wheel", "plot_theme")
    dpg.bind_item_theme("fl_wheel", "plot_theme")
    dpg.bind_item_theme("rl_wheel", "plot_theme")
    dpg.bind_item_theme("origin", "plot_theme")

def fit_interface_size(sender, app_data):
    global width
    global height
    
    width = dpg.get_viewport_width()
    height = dpg.get_viewport_height()
    
    # Ajustar el tamaño de la ventana principal
    dpg.configure_item("window", width=width, height=height)
    
    # Ajustar los tamaños de los paneles
    dpg.configure_item("left_panel", width=int(width * 0.2), height=height)
    dpg.configure_item("right_panel", width=int(width * 0.77), height=int(height*0.9))

dpg.create_context()

with dpg.window(label="Simulation", tag="window"):

    with dpg.theme(tag="plot_theme"):
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 255, 255), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 8, category=dpg.mvThemeCat_Plots)

        with dpg.theme_component(dpg.mvScatterSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Circle, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 15, category=dpg.mvThemeCat_Plots)

    with dpg.group(horizontal=True):
        # Panel izquierdo
        with dpg.group(horizontal=False, tag="left_panel", width=int(0.2 * 1200), height=800):
            dpg.add_text("Path")
            dpg.add_combo(paths, default_value=paths[0], width=-1)
        
            dpg.add_text("Control")
            dpg.add_combo(control, default_value=control[0], width=-1)

            dpg.add_text("x: ?", tag="x")
            dpg.add_text("y: ?", tag="y")
            dpg.add_text("theta: ?", tag="theta")
            dpg.add_text("v: ?", tag="v")
            dpg.add_text("el: ?", tag="el")
            dpg.add_text("et: ?", tag="et")
        
        # Panel derecho - Área de simulación
        with dpg.plot(tag="right_panel", width=int(0.77 * 1200), height=int(800*0.9), equal_aspects=True):
            dpg.add_plot_axis(dpg.mvXAxis, label="Norte", tag="xaxis")
            dpg.add_plot_axis(dpg.mvYAxis, label="Este", tag="yaxis")
            plotVehicle(0,0,0,0)
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")

dpg.create_viewport(title='Pure Pursuit Algorithm', width=1200, height=800)
dpg.setup_dearpygui()
dpg.set_viewport_resize_callback(fit_interface_size)
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
