import dearpygui.dearpygui as dpg
dpg.create_context()
from config import *
from vehicle import *
from functions import *

# Interface setup
with dpg.window(label="Pure Pursuit Algorithm Simulation", tag="window"):
    # Apply the custom interface theme to the main window
    dpg.bind_item_theme("window", "interface_theme")

    # Create a horizontal group to manage layout
    with dpg.group(horizontal=True):
        # Left panel for controls and information display
        with dpg.group(horizontal=False, tag="left_panel", width=int(0.2 * width), height=height):
            # Dropdown for selecting the path
            dpg.add_combo(list_paths, default_value="Select path", width=-1, callback=update_trayectoria, tag="path")
        
            # Dropdown for selecting the steering control algorithm
            dpg.add_combo(steering_control, default_value="Select steering control", width=-1, callback=update_steering_control, tag="control")

            # Dropdown for selecting the velocity control algorithm
            dpg.add_combo(velocity_control, default_value="Select speed control", width=-1, callback=update_velocity_control)

            # Slider for adjusting maximum speed
            dpg.add_text("Maximum speed")
            dpg.add_slider_float(tag="speed_slider", default_value=2.0, min_value=0.1, max_value=10.0, width=200, track_offset=0.1, format='%.1f')

            # Input for setting the maximum steering delta in degrees
            dpg.add_text("Max steering delta in degrees")
            dpg.add_input_float(tag="max_steering_delta", default_value=math.degrees(max_delta_steering), format="%.0f", width=200, max_value=35, min_value=1, min_clamped=True, max_clamped=True, step=1)

            # Input for setting the maximum speed delta in meters per second
            dpg.add_text("Max speed delta in m/s")
            dpg.add_input_float(tag="max_speed_delta", default_value=max_delta_speed, format="%.2f", width=200, max_value=10, min_value=0.01, min_clamped=True, max_clamped=True, step=0.05)

            # Checkbox to toggle noise simulation
            dpg.add_checkbox(label="Simulate with noise", tag="noise_checkbox", default_value=False)

            # Text elements to display vehicle state and errors
            dpg.add_text("X: ?", tag="x")
            dpg.add_text("Y: ?", tag="y")
            dpg.add_text("Theta: ?", tag="theta")
            dpg.add_text("V: ?", tag="v")
            dpg.add_text("Delta: ?", tag="delta")
            dpg.add_text("Lateral error: ?", tag="le")
            dpg.add_text("MSE lateral: ?", tag="msele")
            dpg.add_text("Heading error: ?", tag="he")
            dpg.add_text("MSE heading: ?", tag="msehe")

            # Button to start the simulation
            dpg.add_button(label="Run Simulation", callback=run_simulation, width=150, height=50, tag="run_button")

        # Right panel for plotting the simulation results
        with dpg.plot(tag="right_panel", width=int(0.77 * width), height=int(height*0.9), equal_aspects=True):
            # X-axis configuration
            dpg.add_plot_axis(dpg.mvXAxis, label="West", tag="xaxis")
            # Y-axis configuration
            dpg.add_plot_axis(dpg.mvYAxis, label="Nort", tag="yaxis")
            # Initial vehicle plot
            plotVehicle(0,0,1.5708,0)

            # Plot for the selected path
            dpg.add_line_series([], [], parent="yaxis", tag="trayectoria_plot")
            # Plot for the vehicle's reference trajectory
            dpg.add_line_series([], [], parent="yaxis", tag="reference_plot")

            # Fit the plot axes to the data
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")

# Create and setup the viewport (application window)
dpg.create_viewport(title='Pure Pursuit Algorithm', width=width, height=height)
dpg.setup_dearpygui()
dpg.set_viewport_resize_callback(fit_interface_size)
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
