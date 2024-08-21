import dearpygui.dearpygui as dpg
dpg.create_context()
from config import *
from vehicle import *
from functions import *

with dpg.window(label="Simulation", tag="window"):
    dpg.bind_item_theme("window", "interface_theme")
    with dpg.group(horizontal=True):
        with dpg.group(horizontal=False, tag="left_panel", width=int(0.2 * width), height=height):
            dpg.add_text("Path")
            dpg.add_combo(paths, default_value=paths[0], width=-1, callback=update_trayectoria)
        
            dpg.add_text("Control")
            dpg.add_combo(control, default_value=control[0], width=-1, callback=update_control)

            dpg.add_text("x: ?", tag="x")
            dpg.add_text("y: ?", tag="y")
            dpg.add_text("theta: ?", tag="theta")
            dpg.add_text("v: ?", tag="v")
            dpg.add_text("el: ?", tag="el")
            dpg.add_text("et: ?", tag="et")

            dpg.add_button(label="Comenzar Simulación", callback=run_simulation, width=150, height=50)

        with dpg.plot(tag="right_panel", width=int(0.77 * 1200), height=int(800*0.9), equal_aspects=True):
            dpg.add_plot_axis(dpg.mvXAxis, label="Norte", tag="xaxis")
            dpg.add_plot_axis(dpg.mvYAxis, label="Este", tag="yaxis")
            plotVehicle(0,0,1.5708,0)
            dpg.add_line_series([], [], parent="yaxis", tag="trayectoria_plot")
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")

dpg.create_viewport(title='Pure Pursuit Algorithm', width=1200, height=800)
dpg.setup_dearpygui()
dpg.set_viewport_resize_callback(fit_interface_size)
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
