"""Modern dark theme for the Pure Pursuit simulator."""
import dearpygui.dearpygui as dpg

# ─── Color Palette (Catppuccin Mocha-inspired) ───────────────────────────────
BG_BASE = (24, 24, 37, 255)
BG_SURFACE = (30, 30, 46, 255)
BG_OVERLAY = (49, 50, 68, 255)
BG_HIGHLIGHT = (59, 60, 78, 255)
BG_ELEVATED = (69, 71, 90, 255)

ACCENT = (137, 180, 250, 255)          # Blue
ACCENT_HOVER = (160, 195, 252, 255)
ACCENT_ACTIVE = (110, 160, 245, 255)
ACCENT_DIM = (80, 120, 200, 255)

SUCCESS = (166, 227, 161, 255)         # Green
WARNING = (250, 179, 135, 255)         # Peach
ERROR = (243, 139, 168, 255)           # Red/Pink

TEXT = (205, 214, 244, 255)
TEXT_DIM = (147, 153, 178, 255)
BORDER = (59, 60, 78, 255)
TRANSPARENT = (0, 0, 0, 0)

# ─── Plot Palette ────────────────────────────────────────────────────────────
PLOT_BG = (30, 30, 46, 255)
PLOT_FRAME = (24, 24, 37, 255)
PLOT_GRID = (49, 50, 68, 100)
PLOT_BORDER = (69, 71, 90, 255)

PATH_LINE = (0, 190, 255)              # Cyan — reference path
TRAJ_LINE = (166, 227, 161)            # Green — vehicle trajectory
VEHICLE_LINE = (205, 214, 244)         # Light — vehicle body
MARKER_COLOR = (250, 179, 135, 255)    # Peach — origin dot


def setup_themes():
    """Create all DPG themes. Must be called after dpg.create_context()."""

    # ── Main interface theme ─────────────────────────────────────────────────
    with dpg.theme(tag="interface_theme"):
        with dpg.theme_component(dpg.mvAll):
            # Backgrounds
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, BG_BASE)
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, BG_BASE)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, BG_SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_MenuBarBg, BG_SURFACE)

            # Frames (inputs, combos, sliders)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, BG_ELEVATED)

            # Text
            dpg.add_theme_color(dpg.mvThemeCol_Text, TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, TEXT_DIM)

            # Title bar
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg, BG_SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, BG_SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed, BG_SURFACE)

            # Buttons
            dpg.add_theme_color(dpg.mvThemeCol_Button, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, BG_ELEVATED)

            # Headers (collapsing sections)
            dpg.add_theme_color(dpg.mvThemeCol_Header, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, BG_ELEVATED)

            # Borders
            dpg.add_theme_color(dpg.mvThemeCol_Border, BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_BorderShadow, TRANSPARENT)

            # Slider / Check
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark, ACCENT)

            # Separator
            dpg.add_theme_color(dpg.mvThemeCol_Separator, BORDER)

            # Tab
            dpg.add_theme_color(dpg.mvThemeCol_Tab, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, BG_ELEVATED)

            # Scrollbar
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, BG_BASE)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive, BG_ELEVATED)

            # Styles
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_GrabRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 12, 12)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 5)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ItemInnerSpacing, 6, 4)
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 0)

    # ── Plot themes ──────────────────────────────────────────────────────────
    with dpg.theme(tag="plot_theme"):
        with dpg.theme_component(dpg.mvPlot):
            dpg.add_theme_color(dpg.mvPlotCol_FrameBg, PLOT_FRAME, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBg, PLOT_BG, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBorder, PLOT_BORDER, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBg, BG_SURFACE, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBorder, BORDER, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendText, TEXT, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_AxisText, TEXT_DIM, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_AxisGrid, PLOT_GRID, category=dpg.mvThemeCat_Plots)

    with dpg.theme(tag="vehicle_theme"):
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, VEHICLE_LINE, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 3, category=dpg.mvThemeCat_Plots)

    with dpg.theme(tag="path_theme"):
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, PATH_LINE, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 3, category=dpg.mvThemeCat_Plots)

    with dpg.theme(tag="reference_theme"):
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, TRAJ_LINE, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 3, category=dpg.mvThemeCat_Plots)

    with dpg.theme(tag="origin_marker_theme"):
        with dpg.theme_component(dpg.mvScatterSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, MARKER_COLOR, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 5, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Circle, category=dpg.mvThemeCat_Plots)

    # ── Accent button theme (for Run buttons) ────────────────────────────────
    with dpg.theme(tag="accent_button_theme"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, ACCENT_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, ACCENT_ACTIVE)
            dpg.add_theme_color(dpg.mvThemeCol_Text, BG_BASE)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 16, 10)

    with dpg.theme(tag="secondary_button_theme"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, BG_OVERLAY)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, BG_HIGHLIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, BG_ELEVATED)
            dpg.add_theme_color(dpg.mvThemeCol_Text, TEXT)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 16, 10)

    # ── Metric label theme (dimmed text) ─────────────────────────────────────
    with dpg.theme(tag="metric_label_theme"):
        with dpg.theme_component(dpg.mvText):
            dpg.add_theme_color(dpg.mvThemeCol_Text, TEXT_DIM)

    with dpg.theme(tag="metric_value_theme"):
        with dpg.theme_component(dpg.mvText):
            dpg.add_theme_color(dpg.mvThemeCol_Text, ACCENT)

    with dpg.theme(tag="title_theme"):
        with dpg.theme_component(dpg.mvText):
            dpg.add_theme_color(dpg.mvThemeCol_Text, ACCENT)

    # ── Light interface theme ────────────────────────────────────────────────
    with dpg.theme(tag="light_interface_theme"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (239, 241, 245, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (239, 241, 245, 255))
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (230, 233, 239, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (204, 208, 218, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (188, 192, 204, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (172, 176, 190, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (76, 79, 105, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (140, 143, 161, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg, (230, 233, 239, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (230, 233, 239, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Button, (204, 208, 218, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (188, 192, 204, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (172, 176, 190, 255))
            
            # Use same accent colors for interactive elements but lightened if needed
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark, (30, 102, 245, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (30, 102, 245, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (4, 165, 229, 255))
            
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 12, 12)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 5)

    # ── Light Plot theme ─────────────────────────────────────────────────────
    with dpg.theme(tag="light_plot_theme"):
        with dpg.theme_component(dpg.mvPlot):
            dpg.add_theme_color(dpg.mvPlotCol_FrameBg, (239, 241, 245, 255), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBg, (255, 255, 255, 255), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBorder, (204, 208, 218, 255), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_AxisText, (76, 79, 105, 255), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_AxisGrid, (204, 208, 218, 100), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBg, (230, 233, 239, 200), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendText, (76, 79, 105, 255), category=dpg.mvThemeCat_Plots)

    dpg.set_global_font_scale(1.25)

