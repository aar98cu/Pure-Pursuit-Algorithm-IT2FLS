"""
Modern web-based Pure Pursuit Simulator using NiceGUI + Plotly.
Optimized for Doctoral Thesis presentations with high-fidelity aesthetics.
"""
import math, os, csv, time, asyncio
import numpy as np
import plotly.graph_objects as go
from nicegui import ui, app

# Adjust path for src module
import sys, socket
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def ensure_port_free(port):
    """Attempt to close any existing connection on the given port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.1)
        if s.connect_ex(('localhost', port)) == 0:
            print(f"Port {port} in use. Attempting to clear...")
            # We can't easily kill the other process cross-platform without 'psutil',
            # but we can at least warn or try to bind. 
            # NiceGUI/Uvicorn will still fail if it's truly locked, 
            # but this check helps identify the conflict.

ensure_port_free(8082)

import config
from config import (
    dt, look_ahead, WB, NOISE_MATRIX_SIZE, noise_matrix,
    paths, list_paths, steering_control, velocity_control, steering_mode,
    TRANSLATIONS,
)
from functions import (
    IT2FLS, T1FLS, PI, Trajectory, getDistance, abbreviate_name,
    _it2fls_2mf_steering, _it2fls_3mf_steering, _t1fls_2mf_steering,
    _it2fls_2mf_velocity, _t1fls_steering_model,
    mathematical_model, transfer_function_model, get_cauchy_points,
)
from vehicle import _compute_vehicle_geometry

# ── Controller Descriptions (Thesis Info) ────────────────────────────────────
DESCRIPTIONS = {
    "Geometric Lateral Control": "Classical Pure Pursuit. Calculates steering based on look-ahead distance and heading error.",
    "Error-based controller": "Proportional control directly mapping yaw error to steering angle.",
    "T1FLC with 2MF": "Type-1 Fuzzy Logic Controller with 2 Gaussian Membership Functions per input.",
    "IT2FLC with 2MF": "Interval Type-2 Fuzzy Logic Controller. Robust to uncertainty with 2 MTFs.",
    "IT2FLC with 3MF": "Advanced Interval Type-2 FLC with 3 MTFs, providing higher resolution control logic.",
    "PI Distance Control": "Proportional-Integral control regulating the distance to the look-ahead point.",
    "PI Velocity Control": "Standard PI controller focused on maintaining constant set-point velocity.",
}

# ── Simulation State ────────────────────────────────────────────────────────
_p = {"yaw_error": 0.0, "d_yaw_error": 0.0, "velocity_error": 0.0, "d_velocity_error": 0.0, "ld": 0.0}
_prev_delta_err = 0.0
_running = False
L = TRANSLATIONS["ES"] # Initial Language Reference

class Vehicle:
    def __init__(self, x, y, yaw, vel=0, delta=0):
        self.x, self.y, self.yaw, self.vel, self.delta = x, y, yaw, vel, delta
        self._ni = 0
    def _noise(self):
        v = noise_matrix[self._ni]; self._ni = (self._ni + 1) % NOISE_MATRIX_SIZE; return v
    def update(self, velocity, delta, noise_on, steer_fn, max_dv):
        if noise_on and steer_fn:
            self.delta = steer_fn(self.delta, delta)
            self.x += self.vel * math.cos(self.yaw) * dt + self._noise()
            self.y += self.vel * math.sin(self.yaw) * dt + self._noise()
            self.yaw += self.vel * math.tan(self.delta) / WB * dt
            dv = velocity - self.vel
            if abs(dv) > max_dv: dv = max_dv if dv > 0 else -max_dv
            self.vel += dv
        else:
            self.delta = delta
            self.x += self.vel * math.cos(self.yaw) * dt
            self.y += self.vel * math.sin(self.yaw) * dt
            self.yaw += self.vel * math.tan(self.delta) / WB * dt
            self.vel = velocity

# ── Control Mapping ──────────────────────────────────────────────────────────
STEER_FNS = {
    "Geometric Lateral Control": lambda: math.atan2((2*WB*math.sin(_p["yaw_error"]))/_p["ld"], 1),
    "Error-based controller": lambda: _p["yaw_error"],
    "T1FLC with 2MF": lambda: _t1fls_2mf_steering.model(np.array([_p["yaw_error"], _p["d_yaw_error"]])),
    "IT2FLC with 2MF": lambda: _it2fls_2mf_steering.model(np.array([_p["yaw_error"], _p["d_yaw_error"]])),
    "IT2FLC with 3MF": lambda: _it2fls_3mf_steering.model(np.array([_p["yaw_error"], _p["d_yaw_error"]])),
}
def _vel_pi(v, pi, ms): e=_p["ld"]-look_ahead; return np.clip(pi.control(e),0,ms), e
def _vel_pi2(v, pi, ms): e=ms-v; return np.clip(v+pi.control(e)*dt,0,ms), e
def _vel_it2(v, pi, ms): e=_p["ld"]-look_ahead; return np.clip(_it2fls_2mf_velocity.model(np.array([e,_p["d_velocity_error"]])),0,ms), e
VEL_FNS = {"PI Distance Control": _vel_pi, "PI Velocity Control": _vel_pi2, "IT2FLC with 2MF": _vel_it2}

def _t1fls_mode(od, nd):
    global _prev_delta_err; c=od
    for _ in range(5):
        de=nd-c; cs=np.clip(0.8*de+0.0001941*((de-_prev_delta_err)/dt),-1,1)
        cs=np.interp(cs,[-1,1],[-255,255]); _prev_delta_err=de
        c=_t1fls_steering_model.model(np.array([c,cs]))
    return c
MODE_FNS = {"Steering angle restriction": mathematical_model, "Transfer function model": transfer_function_model, "T1FLS": _t1fls_mode}

# ── Plot Config ─────────────────────────────────────────────────────────────
def get_plot_theme(dark_mode):
    return 'plotly_dark' if dark_mode else 'plotly_white'

def make_fig(dark):
    fig = go.Figure()
    colors = {'path': '#00d4ff' if dark else '#0070f3',
              'traj': '#a6e3a1' if dark else '#228b22',
              'veh': '#cdd6f4' if dark else '#1e1e2e',
              'bg': '#11111b' if dark else '#f8f9fa',
              'plot': '#1e1e2e' if dark else '#ffffff',
              'grid': 'rgba(69,71,90,0.4)' if dark else 'rgba(0,0,0,0.1)'}
    
    fig.add_trace(go.Scatter(x=[], y=[], mode='lines', name='Reference Path', line=dict(color=colors['path'], width=3, dash='dot')))
    fig.add_trace(go.Scatter(x=[], y=[], mode='lines', name='Vehicle Trajectory', line=dict(color=colors['traj'], width=2.5)))
    fig.add_trace(go.Scatter(x=[], y=[], mode='lines', name='Vehicle Hull', line=dict(color=colors['veh'], width=2), showlegend=False))
    fig.add_trace(go.Scatter(x=[], y=[], mode='lines', name='Cabin', line=dict(color=colors['path'], width=1.5), showlegend=False))
    for _ in range(4):
        fig.add_trace(go.Scatter(x=[], y=[], mode='lines', line=dict(color='#89b4fa', width=2), showlegend=False))
    fig.add_trace(go.Scatter(x=[], y=[], mode='markers', marker=dict(color='#fab387', size=10, symbol='circle'), showlegend=False))
    
    fig.update_layout(
        template=get_plot_theme(dark),
        paper_bgcolor=colors['bg'], plot_bgcolor=colors['plot'],
        font=dict(family='Inter, sans-serif', color=colors['veh'], size=13),
        xaxis=dict(title='X (m)', gridcolor=colors['grid'], scaleanchor='y', scaleratio=1, zeroline=False),
        yaxis=dict(title='Y (m)', gridcolor=colors['grid'], zeroline=False),
        legend=dict(orientation='h', yanchor='bottom', y=1.02, xanchor='right', x=1),
        margin=dict(l=50, r=20, t=10, b=50),
    )
    return fig

def make_fuzzy_fig(is_dark, controller, input_idx=0, lang="ES"):
    fig = go.Figure()
    colors = ['#89b4fa', '#f38ba8', '#a6e3a1', '#fab387', '#f9e2af', '#cba6f7']
    bg, plot_bg = ('#11111b', '#1e1e2e') if is_dark else ('#f8f9fa', '#ffffff')
    txt = '#cdd6f4' if is_dark else '#1e1e2e'
    
    # Range based on typical errors
    r = 1.0 if input_idx == 0 else 0.5
    xlabel = TRANSLATIONS[lang]["yaw_error"] if input_idx == 0 else TRANSLATIONS[lang]["dyaw_error"]
    
    # Correct indexing for MFs based on input_idx
    mf_start = input_idx * controller.n_mf
    for i in range(controller.n_mf):
        p = controller.MF_params[mf_start + i]
        xs, y_low = get_cauchy_points(p[0], p[2], p[3], p[4], r)
        _, y_high = get_cauchy_points(p[1], p[2], p[3], 1.0, r)
        
        fig.add_trace(go.Scatter(x=xs, y=y_high, mode='lines', line=dict(width=1, color=colors[i]), name=f'MF {i+1} Upper', showlegend=False))
        fig.add_trace(go.Scatter(x=xs, y=y_low, mode='lines', fill='tonexty', line=dict(width=1, color=colors[i]), name=f'IT2 FOU', opacity=0.3))
        fig.add_trace(go.Scatter(x=[0], y=[0], mode='markers', marker=dict(size=10, color=colors[i], symbol='diamond'), name=f'Firing', showlegend=False))

    fig.update_layout(
        height=200, margin=dict(l=40, r=20, t=30, b=40),
        paper_bgcolor=bg, plot_bgcolor=plot_bg, font=dict(color=txt, size=9),
        xaxis=dict(title=xlabel, range=[-r, r], gridcolor='rgba(100,100,100,0.1)'),
        yaxis=dict(title=TRANSLATIONS[lang]["firing"], range=[0, 1.1], gridcolor='rgba(100,100,100,0.1)'),
        title=dict(text=f'{TRANSLATIONS[lang]["active_inf"]} - Input {input_idx+1}', font=dict(size=11)),
        legend=dict(orientation='h', font=dict(size=8))
    )
    return fig

# ── NiceGUI Construction ──────────────────────────────────────────────────────
@ui.page('/')
def main_page():
    global _running
    # Move UI State into page scope or use app storage if multi-page
    # NiceGUI Construction
    dark_mode_ctrl = ui.dark_mode(True)
    ui.colors(primary='#89b4fa', secondary='#a6e3a1', accent='#fab387', dark='#1e1e2e')
    
    current_lang = {"v": "ES"}
    def reload_texts():
        l = TRANSLATIONS[current_lang["v"]]
        header_title.text = l["title"]; header_sub.text = l["subtitle"]
        run_btn.text = l["start"]; multi_btn.text = l["batch"]; status_lbl.text = l["idle"] if not _running else l["simulating"]
        path_label.text = l["trajectory"]; path_sel.label = l["route"]
        ctrl_label.text = l["control"]; steer_sel.label = l["lateral"]; vel_sel.label = l["velocity"]
        noise_label.text = l["uncertainty"]; noise_cb.text = l["noise"]; mode_sel.label = l["dynamics"]
        s_delta_val.label = l["max_steer"]; v_delta_val.label = l["max_speed"]
        limit_label.text = l["limits"]; sp_label_prefix.text = l["setpoint"]
        inf_label.text = l["inference"]; fou_txt.content = l["fou_desc"]; en_txt.content = l["enabled_desc"]
        for k, v in m_labels.items(): m_titles[k].text = l[k.replace('msele','mse_lat').replace('msehe','mse_head')]
        
        # Update Fuzzy Layouts
        for i, f_ui in enumerate([fuzzy1_ui, fuzzy2_ui]):
            f_ui.model_dict['layout']['title']['text'] = f'{l["active_inf"]} - Input {i+1}'
            f_ui.model_dict['layout']['xaxis']['title']['text'] = l["yaw_error"] if i==0 else l["dyaw_error"]
            f_ui.model_dict['layout']['yaxis']['title']['text'] = l["firing"]
            f_ui.update()

    # Header
    with ui.header().classes('items-center justify-between px-6 py-2 bg-[#181825] border-b border-[#45475a]'):
        with ui.row().classes('items-center gap-3'):
            ui.icon('settings_input_antenna', color='#89b4fa').classes('text-3xl')
            with ui.column().classes('gap-0'):
                header_title = ui.label('').classes('text-lg font-bold text-white tracking-tight')
                header_sub = ui.label('').classes('text-xs text-[#6c7086]')
        
        with ui.row().classes('items-center gap-6'):
            ui.select(['ES', 'EN'], value='ES', label='Language').bind_value(current_lang, 'v').on_value_change(reload_texts).props('dense dark outlined width=80')
            ui.switch('Dark Mode', value=True).bind_value_to(dark_mode_ctrl, 'value').on_value_change(lambda e: update_theme(e.value))
            with ui.row().classes('gap-2'):
                run_btn = ui.button('', on_click=lambda: start_sim(False)).classes('px-6 font-bold shadow-lg')
                multi_btn = ui.button('', color='#313244', on_click=lambda: start_sim(True)).classes('px-4 font-semibold')
            status_lbl = ui.label('').classes('px-3 py-1 rounded bg-[#1e1e2e] text-[#a6e3a1] font-bold text-xs border border-[#a6e3a1]/20')

    # Sidebar
    with ui.left_drawer(value=True, fixed=True).classes('bg-[#11111b] border-r border-[#313244] p-4 gap-4').style('width: 320px'):
        path_label = ui.label('').classes('text-[10px] text-[#585b70] font-bold tracking-widest')
        path_sel = ui.select(list_paths, label='').classes('w-full').props('outlined dark dense')
        
        ctrl_label = ui.label('').classes('text-[10px] text-[#585b70] font-bold tracking-widest mt-4')
        with ui.column().classes('w-full gap-2'):
            steer_sel = ui.select(steering_control, label='').classes('w-full').props('outlined dark dense')
            vel_sel = ui.select(velocity_control, label='').classes('w-full').props('outlined dark dense')

        noise_label = ui.label('').classes('text-[10px] text-[#585b70] font-bold tracking-widest mt-4')
        noise_cb = ui.checkbox('').props('dark')
        with ui.column().classes('w-full gap-2').bind_visibility_from(noise_cb, 'value'):
            mode_sel = ui.select(steering_mode, label='').classes('w-full').props('outlined dark dense')
            s_delta_val = ui.number('', value=3, format='%.0f').classes('w-full').props('outlined dark dense')
            v_delta_val = ui.number('', value=0.05, format='%.2f').classes('w-full').props('outlined dark dense')

        limit_label = ui.label('').classes('text-[10px] text-[#585b70] font-bold tracking-widest mt-4')
        speed_sl = ui.slider(min=0.1, max=10.0, value=2.0).props('dark dense')
        with ui.row().classes('gap-1 items-center'):
            sp_label_prefix = ui.label('').classes('text-xs font-mono text-[#89b4fa]')
            ui.label('').classes('text-xs font-mono text-[#89b4fa]').bind_text_from(speed_sl, 'value', lambda v: f': {v:.1f} m/s')

    # Content
    with ui.row().classes('w-full h-full no-wrap'):
        with ui.column().classes('flex-grow p-4 gap-4'):
            with ui.row().classes('w-full gap-4 no-wrap'):
                plot_ui = ui.plotly(fig).classes('flex-grow rounded-xl overflow-hidden shadow-2xl').style('height: 60vh')
                with ui.column().classes('w-80 gap-2'):
                    fuzzy1_ui = ui.plotly(make_fuzzy_fig(True, _it2fls_2mf_steering, 0)).classes('w-full rounded-xl shadow-lg bg-[#1e1e2e]')
                    fuzzy2_ui = ui.plotly(make_fuzzy_fig(True, _it2fls_2mf_steering, 1)).classes('w-full rounded-xl shadow-lg bg-[#1e1e2e]')
                    
                    with ui.card().classes('w-full bg-[#1e1e2e] border-[#313244] p-3 gap-1'):
                        inf_label = ui.label('').classes('text-[9px] text-[#585b70] font-bold tracking-widest mb-1')
                        fou_txt = ui.markdown('').classes('text-[9px] text-[#cdd6f4] leading-tight')
                        en_txt = ui.markdown('').classes('text-[9px] text-[#a6e3a1] leading-tight')

            # Metrics
            m_titles = {}
            with ui.row().classes('w-full gap-2'):
                keys = [('MSE LATERAL','msele','m'),('MSE HEADING','msehe','°'),('X','x','m'),('Y','y','m'),('THETA','theta','°'),('VELOCITY','v','m/s')]
                for label, k, unit in keys:
                    with ui.card().classes('flex-grow bg-[#1e1e2e] border-[#313244] p-3 gap-0'):
                        m_titles[k] = ui.label(label).classes('text-[9px] text-[#6c7086] font-black tracking-tighter')
                        with ui.row().classes('items-baseline gap-1'):
                            l = ui.label('—').classes('text-lg font-bold text-[#89b4fa] font-mono')
                            ui.label(unit).classes('text-[9px] text-[#585b70]')
                            m_labels[k] = l
    
    reload_texts()

    # ── Functions ─────────────────────────────────────────────────────────────
    def update_theme(is_dark):
        fig.layout.template = get_plot_theme(is_dark)
        colors = {'path': '#00d4ff' if is_dark else '#0070f3',
                  'traj': '#a6e3a1' if is_dark else '#228b22',
                  'veh': '#cdd6f4' if is_dark else '#1e1e2e',
                  'bg': '#11111b' if is_dark else '#f8f9fa',
                  'plot': '#1e1e2e' if is_dark else '#ffffff',
                  'grid': 'rgba(69,71,90,0.4)' if is_dark else 'rgba(0,0,0,0.1)'}
        fig.layout.paper_bgcolor = colors['bg']
        fig.layout.plot_bgcolor = colors['plot']
        fig.layout.xaxis.gridcolor = colors['grid']
        fig.layout.yaxis.gridcolor = colors['grid']
        fig.data[0].line.color = colors['path']
        fig.data[1].line.color = colors['traj']
        fig.data[2].line.color = colors['veh']
        plot_ui.update()

    def on_path_change(e):
        if not e.value: return
        xd, yd = zip(*paths[e.value])
        fig.data[0].x, fig.data[0].y = list(xd), list(yd)
        fig.update_layout(xaxis=dict(range=[min(xd)-4, max(xd)+4]), yaxis=dict(range=[min(yd)-4, max(yd)+4]))
        plot_ui.update()
    path_sel.on_value_change(on_path_change)

    async def start_sim(multi):
        global _running, _prev_delta_err
        if _running: return
        _running = True; _prev_delta_err = 0.0
        status_lbl.text = 'SIMULATING...'; status_lbl.style('color: #fab387; border-color: #fab387')
        run_btn.disable()

        xd, yd = zip(*paths[path_sel.value])
        path_arr = np.stack((xd, yd), axis=-1)
        traj = Trajectory(xd, yd); goal = traj.getPoint(len(xd)-1)
        init_yaw = math.atan2(yd[1]-yd[0], xd[1]-xd[0])
        start_x = xd[0] - 3.0 * math.cos(init_yaw)
        start_y = yd[0] - 3.0 * math.sin(init_yaw)
        ego = Vehicle(start_x, start_y, init_yaw)
        pi_c = PI(); max_sp = speed_sl.value
        noise_on = noise_cb.value
        mode_fn = MODE_FNS.get(mode_sel.value) if noise_on else None
        steer_fn = STEER_FNS[steer_sel.value]; vel_fn = VEL_FNS[vel_sel.value]
        
        traj_x, traj_y, lat_errs, head_errs, vel_errs = [], [], [], [], []
        idx = near_idx = 0
        tracking_started = False
        
        while getDistance([ego.x, ego.y], goal) > 1 or idx < len(xd)-2:
            t0 = time.perf_counter()
            tp, cur, idx = traj.getTargetPoint([ego.x, ego.y]); _p["ld"] = cur
            
            # Calculate Velocity and its Error dynamics (CRITICAL FOR FLS)
            vel, ve = vel_fn(ego.vel, pi_c, max_sp)
            _p["d_velocity_error"] = (ve - (vel_errs[-1] if vel_errs else 0)) / dt
            _p["velocity_error"] = ve
            vel_errs.append(ve)
            
            ss, se = max(0,near_idx-5), min(len(path_arr),near_idx+20)
            ldists = np.sqrt((path_arr[ss:se,0]-ego.x)**2+(path_arr[ss:se,1]-ego.y)**2)
            near_idx = ss+np.argmin(ldists)
            
            # Lateral error calculation
            distance_to_path = np.sqrt((path_arr[near_idx,0]-ego.x)**2+(path_arr[near_idx,1]-ego.y)**2)
            ye = math.atan2(path_arr[near_idx,1]-ego.y, path_arr[near_idx,0]-ego.x)-ego.yaw
            ye = np.arctan2(np.sin(ye),np.cos(ye))
            l_err = distance_to_path * math.sin(ye)
            
            # Heading error calculation
            yaw_e = math.atan2(tp[1]-ego.y, tp[0]-ego.x)-ego.yaw
            yaw_e = np.arctan2(np.sin(yaw_e),np.cos(yaw_e))
            
            # 🚀 Logic: Start recording errors only when within tracking zone (<0.5m)
            if not tracking_started and distance_to_path < 0.5:
                tracking_started = True
            
            if tracking_started:
                lat_errs.append(abs(l_err))
                head_errs.append(abs(yaw_e))

            _p["d_yaw_error"] = (yaw_e - (head_errs[-2] if len(head_errs)>1 else 0)) / dt
            _p["yaw_error"] = yaw_e
            
            # Execute Control
            
            # Execute Control
            is_it2 = steer_sel.value.startswith("IT2")
            if is_it2:
                # Capture firing values for visualization
                ctrl_obj = _it2fls_3mf_steering if "3MF" in steer_sel.value else _it2fls_2mf_steering
                d, firing = ctrl_obj.model(np.array([_p["yaw_error"], _p["d_yaw_error"]]), return_firing=True)
            else:
                d = np.clip(steer_fn(), -0.698132, 0.698132)
                firing = None

            max_dv = v_delta_val.value if noise_on else 0.05
            ego.update(vel, d, noise_on, mode_fn, max_dv)
            traj_x.append(ego.x); traj_y.append(ego.y)

            # Update UI (throttle for smoothness)
            if len(traj_x) % 2 == 0:
                fig.data[1].x, fig.data[1].y = traj_x, traj_y
                v_parts = _compute_vehicle_geometry(ego.x, ego.y, ego.yaw, ego.delta)
                for i, p in enumerate(v_parts):
                    fig.data[2+i].x, fig.data[2+i].y = p[0,:].tolist(), p[1,:].tolist()
                fig.data[8].x, fig.data[8].y = [ego.x], [ego.y]
                
                # Update Fuzzy Graphs
                if is_it2 and firing:
                    # Update Fig 1 (Yaw Error) and Fig 2 (d_Yaw Error)
                    for f_ui, f_fig, input_val, f_offset in [(fuzzy1_ui, fuzzy1_ui.model_dict, _p["yaw_error"], 0), 
                                                             (fuzzy2_ui, fuzzy2_ui.model_dict, _p["d_yaw_error"], ctrl_obj.n_mf)]:
                        for i in range(ctrl_obj.n_mf):
                            f_idx = 3*i + 2
                            f_fig['data'][f_idx]['x'] = [input_val]
                            avg_f = (firing[f_offset+i][0] + firing[f_offset+i][1]) / 2.0
                            f_fig['data'][f_idx]['y'] = [avg_f]
                        f_ui.update()

                plot_ui.update()
                m_labels['x'].text=f"{ego.x:.2f}"; m_labels['y'].text=f"{ego.y:.2f}"
                m_labels['theta'].text=f"{math.degrees(ego.yaw):.1f}"; m_labels['v'].text=f"{ego.vel:.1f}"
                m_labels['msele'].text=f"{np.mean(np.square(lat_errs)):.4f}"
                m_labels['msehe'].text=f"{math.degrees(np.mean(np.square(head_errs))):.4f}"
                # Visual magnitude reporting for L ERR and H ERR
                m_labels['le'].text=f"{abs(l_err):.4f}"; m_labels['he'].text=f"{math.degrees(abs(yaw_e)):.2f}"

            # Real-time synchronization
            elapsed = time.perf_counter() - t0
            await asyncio.sleep(max(0.001, dt - elapsed))

        _running = False
        run_btn.enable()
        status_lbl.text = 'SUCCESS'; status_lbl.style('color: #a6e3a1; border-color: #a6e3a1')

ui.run(title='Thesis Framework — Pure Pursuit', show=True, port=8082, reload=False)
