import dearpygui.dearpygui as dpg

def ajustar_tamano_interfaz(sender, app_data):
    width = dpg.get_viewport_width()
    height = dpg.get_viewport_height()
    dpg.configure_item("window", width=width, height=height)

dpg.create_context()

with dpg.window(label="Simulation", tag="window"):
    dpg.add_text("Simulación de Vehículo Siguiendo una Trayectoria")

dpg.create_viewport(title='Pure Pursuit Algorithm', width=800, height=600)
dpg.setup_dearpygui()

# Ajustar la interfaz cuando se redimensiona la ventana
dpg.set_viewport_resize_callback(ajustar_tamano_interfaz)

dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
