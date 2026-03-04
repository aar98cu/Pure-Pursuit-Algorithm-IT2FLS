import math, os, sys
import dearpygui.dearpygui as dpg

# Adjust path for src module
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

dpg.create_context()

from themes import setup_themes
setup_themes()

import config
from config import (
    list_paths, steering_control, velocity_control, steering_mode,
    max_delta_steering, max_delta_speed, TRANSLATIONS,
)
from vehicle import plotVehicle
from functions import (
    update_path, update_steering_control, update_velocity_control,
    update_steering_mode, toggle_noise_settings,
    fit_interface_size,
    run_simulation_callback, run_multiple_callback,
    get_cauchy_points, plot_mfs, _it2fls_2mf_steering,
)

# ── Language State ──────────────────────────────────────────────────────────
_lang = "ES"

def update_lang(sender, app_data):
    global _lang
    _lang = app_data
    L = TRANSLATIONS[_lang]
    # Header and Titles (using set_value for text items)
    dpg.set_value("path_header", L["trajectory"])
    dpg.set_value("control_header", L["control"])
    dpg.set_value("noise_header", L["uncertainty"])
    dpg.set_value("limit_header", L["limits"])
    dpg.set_value("speed_slider_text", L["setpoint"])
    dpg.set_value("fuzzy_header", L["active_inf"])
    
    # Buttons and Checkboxes (using set_item_label for buttons)
    dpg.set_item_label("run_button", L["start"])
    dpg.set_item_label("batch_button", L["batch"])
    dpg.set_item_label("noise_checkbox", L["noise"])
    
    # Metric Labels
    dpg.set_value("v_lbl", f'{L["vel"]}:')
    dpg.set_value("delta_lbl", "DELTA:") # Delta is usually universal in Spanish context too
    dpg.set_value("msele_lbl", f'{L["mse_lat"]}:')
    dpg.set_value("msehe_lbl", f'{L["mse_head"]}:')
    
    # Fuzzy Plot Labels
    dpg.set_item_label("fuzzy_plot_1", f'Input 1 ({L["yaw_error"]})')
    dpg.set_item_label("fuzzy_plot_2", f'Input 2 ({L["dyaw_error"]})')

# ── Theme Toggle Logic ───────────────────────────────────────────────────────
_is_dark = True
def toggle_theme():
    global _is_dark
    _is_dark = not _is_dark
    interface = "interface_theme" if _is_dark else "light_interface_theme"
    plot = "plot_theme" if _is_dark else "light_plot_theme"
    
    dpg.bind_item_theme("window", interface)
    dpg.bind_item_theme("right_panel", plot)
    dpg.set_viewport_clear_color((24, 24, 37, 255) if _is_dark else (239, 241, 245, 255))

# ── Interface setup ───────────────────────────────────────────────────────────
with dpg.window(label="Pure Pursuit Framework", tag="window", no_title_bar=True):
    dpg.bind_item_theme("window", "interface_theme")

    with dpg.group(horizontal=True):
        # Left panel (Scrollable Child Window)
        with dpg.child_window(tag="left_panel", width=340, border=False):
            dpg.add_spacer(height=10)
            with dpg.group(horizontal=True):
                dpg.add_text("⚡ RESEARCH SUITE", color=(137, 180, 250))
                dpg.add_spacer(width=20)
                dpg.add_button(label="THEME", callback=toggle_theme, width=80, small=True)
                dpg.add_combo(["ES", "EN"], default_value="ES", callback=update_lang, width=60)
            
            dpg.add_spacer(height=5)
            dpg.add_separator()
            
            dpg.add_text("Trajectory", tag="path_header")
            dpg.add_combo(list_paths, default_value="Path 1", width=-1, callback=update_path, tag="path")
            
            dpg.add_text("Control Architecture", tag="control_header")
            dpg.add_combo(steering_control, default_value=steering_control[0], width=-1, callback=update_steering_control, tag="control")
            dpg.add_combo(velocity_control, default_value=velocity_control[0], width=-1, callback=update_velocity_control, tag="speed")

            dpg.add_text("Uncertainty", tag="noise_header")
            dpg.add_checkbox(tag="noise_checkbox", default_value=False, callback=toggle_noise_settings)
            dpg.add_combo(steering_mode, default_value=steering_mode[0], width=-1, callback=update_steering_mode, tag="steering", show=False)
            
            dpg.add_text("Max Steering Delta (°)", tag="max_steering_delta_text", show=False)
            dpg.add_input_float(tag="max_steering_delta", default_value=3.0, width=-1, show=False)
            dpg.add_text("Max Speed Delta (m/s)", tag="max_speed_delta_text", show=False)
            dpg.add_input_float(tag="max_speed_delta", default_value=0.05, width=-1, show=False)

            dpg.add_text("Operational Limits", tag="limit_header")
            dpg.add_text("Set Point", tag="speed_slider_text", color=(137, 180, 250))
            dpg.add_slider_float(tag="speed_slider", default_value=2.0, min_value=0.1, max_value=10.0, width=-1)

            dpg.add_separator()
            
            # Metrics
            met_cols = [("VELOCITY", "v_lbl", "v"), ("DELTA", "delta_lbl", "delta"), ("MSE LATERAL", "msele_lbl", "msele"), ("MSE HEADING", "msehe_lbl", "msehe"), ("X", "x_lbl", "x"), ("Y", "y_lbl", "y"), ("THETA", "theta_lbl", "theta"), ("L ERR", "le_lbl", "le"), ("H ERR", "he_lbl", "he")]
            for label, lbl_tag, tag in met_cols:
                with dpg.group(horizontal=True):
                    dpg.add_text(f"{label}:", tag=lbl_tag, color=(108, 112, 134))
                    dpg.add_text("0.0", tag=tag, color=(166, 227, 161))

            dpg.add_button(label="START SIMULATION", callback=run_simulation_callback, width=-1, height=40, tag="run_button")
            dpg.add_button(label="BATCH RUN", callback=run_multiple_callback, width=-1, height=30, tag="batch_button")

            dpg.add_spacer(height=10)
            dpg.add_text("Inference Logic", tag="fuzzy_header", color=(137, 180, 250))
            with dpg.plot(label="Input 1 (Yaw Error)", height=130, width=-1, tag="fuzzy_plot_1"):
                dpg.add_plot_axis(dpg.mvXAxis, no_gridlines=True, tag="fuzzy_plot_1_xaxis")
                dpg.add_plot_axis(dpg.mvYAxis, no_gridlines=True, tag="fuzzy_plot_1_yaxis")
            with dpg.plot(label="Input 2 (d_Yaw Error)", height=130, width=-1, tag="fuzzy_plot_2"):
                dpg.add_plot_axis(dpg.mvXAxis, no_gridlines=True, tag="fuzzy_plot_2_xaxis")
                dpg.add_plot_axis(dpg.mvYAxis, no_gridlines=True, tag="fuzzy_plot_2_yaxis")
            
            dpg.add_spacer(height=20) # Bottom padding for scroll

        # Right panel for plots
        with dpg.group(tag="plot_container"):
            with dpg.plot(tag="right_panel", width=-1, height=-1, equal_aspects=True):
                dpg.add_plot_axis(dpg.mvXAxis, label="X (m)", tag="xaxis")
                dpg.add_plot_axis(dpg.mvYAxis, label="Y (m)", tag="yaxis")
                plotVehicle(0, 0, 1.5708, 0)
                dpg.add_line_series([], [], parent="yaxis", tag="path_plot")
                dpg.add_line_series([], [], parent="yaxis", tag="reference_plot")
                dpg.fit_axis_data("xaxis")
                dpg.fit_axis_data("yaxis")

from functions import _it2fls_2mf_steering
plot_mfs(_it2fls_2mf_steering, 0, "fuzzy_plot_1")
plot_mfs(_it2fls_2mf_steering, 1, "fuzzy_plot_2")

# Apply initial language
update_lang(None, "ES")

dpg.bind_item_theme("right_panel", "plot_theme")

dpg.create_viewport(title='Pure Pursuit Algorithm IT2FLS', width=1200, height=800)
dpg.maximize_viewport()
dpg.setup_dearpygui()
dpg.set_viewport_resize_callback(fit_interface_size)
dpg.show_viewport()
dpg.set_primary_window("window", True)
dpg.start_dearpygui()
dpg.destroy_context()
