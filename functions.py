import dearpygui.dearpygui as dpg
from config import *
from vehicle import *
import math
import numpy as np
import time
import matplotlib.pyplot as plt

# Adjust the size of the interface
def fit_interface_size(sender, app_data):
    global width
    global height
    
    width = dpg.get_viewport_width()
    height = dpg.get_viewport_height()
    
    dpg.configure_item("window", width=width, height=height)
    
    dpg.configure_item("left_panel", width=int(width * 0.2), height=40)
    dpg.configure_item("right_panel", width=int(width * 0.77), height=int(height*0.9))

# Select and display the selected path
def update_trayectoria(sender, app_data):
    seleccion = app_data
    x_data, y_data = zip(*trayectorias[seleccion])
    
    dpg.set_value("trayectoria_plot", [x_data, y_data])
    dpg.bind_item_theme("trayectoria_plot", "path_theme")
    dpg.fit_axis_data("xaxis")
    dpg.fit_axis_data("yaxis")

# Function for steering wheel calculation by geometric lateral control
def geometric_lateral_control():
    delta = math.atan2((2*1.3*math.sin(params["yaw_error"]))/params["ld"],1)
    return delta

# Function for steering wheel calculation by error
def based_on_error():
    delta = params["yaw_error"]
    return delta

# Function for steering wheel calculation by IT2FLS with 2 MF
def it2fls_2mf():
    steering = IT2FLS(n_entradas=2, n_mf=2, n_rules=4, nodos=np.zeros((19,2)), mf_p=MF_parameters_it2mf2, rules=Rules_it2mf2, c_p=C_parameters_it2mf2)
    delta = steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))
    return delta

# Function for steering wheel calculation by IT2FLS with 3 MF
def it2fls_3mf():
    steering = IT2FLS(n_entradas=2, n_mf=3, n_rules=9, nodos=np.zeros((36,2)), mf_p=MF_parameters_it2mf3, rules=Rules_it2mf3, c_p=C_parameters_it2mf3)
    delta = steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))
    return delta

# Function for steering wheel calculation by T1FLS with 2 MF
def t1fls():
    return

# Update the selected control type
def update_control(sender, app_data):
    global control_func
    selected_control = app_data
    
    control_func = control_functions[selected_control]

# Dictionary of the different types of control
control_functions = {
    "Geometric Lateral Control": geometric_lateral_control,
    "Based on error": based_on_error,
    "IT2FLS with 2MF": it2fls_2mf,
    "IT2FLS with 3MF": it2fls_3mf,
    "T1FLS": t1fls
}

# Function to run the simulation of the vehicle on the selected route with the chosen control    
def run_simulation():
    global max_delta_steering
    global max_delta_speed
    dpg.disable_item("speed_slider")
    dpg.disable_item("noise_checkbox")
    dpg.disable_item("run_button")
    dpg.disable_item("path")
    dpg.disable_item("control")
    dpg.disable_item("max_steering_delta")
    dpg.disable_item("max_speed_delta")

    max_delta_steering = math.radians(dpg.get_value("max_steering_delta"))
    max_delta_speed = dpg.get_value("max_speed_delta")

    ego =  Mathematic_Model(0,0,1.5708)

    x_data, y_data = dpg.get_value("trayectoria_plot")

    ruta = np.stack((x_data, y_data), axis=-1)
    traj = Trajectory(x_data, y_data)
    goal = traj.getPoint(len(x_data) - 1)

    PI_acc = PI()

    traj_ego_x = []
    traj_ego_y = []

    lateral_error_list = [0]
    heading_error_list = [0]
    speed_list = []
    delta_list = []
    index = 0

    file_path = f'./results/{dpg.get_value("control")} in {dpg.get_value("path")} {"with noise" if dpg.get_value("noise_checkbox") else "without noise"} and speed {dpg.get_value("speed_slider")}.txt'    

    with open(file_path, "w") as file:
        file.write("mse_lateral_error,mse_heading_error\n")
        
        while getDistance([ego.x, ego.y], goal) > 1 or index<len(x_data)-2:
            target_point, current, index = traj.getTargetPoint([ego.x, ego.y])

            vel_err = math.sqrt(math.pow(target_point[1] - ego.y,2)+math.pow(target_point[0] - ego.x,2))-look_ahead
            acc = np.clip(PI_acc.control(vel_err), 0, dpg.get_value("speed_slider"))

            distances = np.sqrt((ruta[:, 0] - ego.x)**2 + (ruta[:, 1] - ego.y)**2)
            index_near_point = np.argmin(distances)
            yaw_err_mp = math.atan2(ruta[index_near_point, 1] - ego.y, ruta[index_near_point, 0] - ego.x) - ego.yaw
            yaw_err_mp = np.arctan2(np.sin(yaw_err_mp), np.cos(yaw_err_mp))

            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            
            params["yaw_error"] = yaw_err
            params["d_yaw_error"] = (heading_error_list[-1] - (heading_error_list[-2] if len(heading_error_list) > 1 else 0)) / dt
            params["ld"] = current
            delta = np.clip(control_func(), -0.698132, 0.698132)
            
            ego.update(acc, delta)

            traj_ego_x.append(ego.x)
            traj_ego_y.append(ego.y)

            lateral_error_list.append(distances[index_near_point]*math.sin(yaw_err_mp))
            heading_error_list.append(yaw_err)
            speed_list.append(ego.vel)
            delta_list.append(math.degrees(ego.delta))
            file.write(f"{(np.mean(np.square(lateral_error_list)))},{(math.degrees(np.mean(np.square(heading_error_list))))}\n")

            #Consider a function to update the interface
            dpg.set_value("x", f"X: {ego.x:.2f}m")
            dpg.set_value("y", f"Y: {ego.y:.2f}m")
            dpg.set_value("theta", f"Theta: {math.degrees(ego.yaw):.2f}°")
            dpg.set_value("v", f"V: {ego.vel:.2f}m/s")
            dpg.set_value("delta", f"Delta: {math.degrees(delta):.2f}°")
            dpg.set_value("le", f"Lateral error: {lateral_error_list[-1]:.2f}m")
            dpg.set_value("msele", f"MSE lateral: {(np.mean(np.square(lateral_error_list))):.2f}m")
            dpg.set_value("he", f"Heading error: {math.degrees(yaw_err):.2f}°")
            dpg.set_value("msehe", f"MSE heading: {math.degrees(np.mean(np.square(heading_error_list))):.2f}°")
            
            update_vehicle(ego.x, ego.y, ego.yaw, delta)
            dpg.set_value("reference_plot", [traj_ego_x, traj_ego_y])
            dpg.bind_item_theme("reference_plot", "reference_theme")
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")
            time.sleep(dt)

    dpg.enable_item("speed_slider")
    dpg.enable_item("noise_checkbox")
    dpg.enable_item("run_button")
    dpg.enable_item("path")
    dpg.enable_item("control")
    dpg.enable_item("max_steering_delta")
    dpg.enable_item("max_speed_delta")

    #Create function to graph
    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(speed_list, label='Speed')
    plt.xlabel('Samples')
    plt.ylabel('Speed (m/s)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(delta_list, label='Steering')
    plt.xlabel('Samples')
    plt.ylabel('Steering (degrees)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(lateral_error_list, label='Lateral Error')
    plt.xlabel('Samples')
    plt.ylabel('Lateral Error (m)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot([math.degrees(angle) for angle in heading_error_list], label='Heading Error')
    plt.xlabel('Samples')
    plt.ylabel('Heading Error (degreess)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# Calculate the distance between two points
def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

# Functions of the Mathematical Bicycle Model
class Mathematic_Model:
    def __init__(self, x, y, yaw, vel=0, delta=0):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        :param delta: float, steering orientation
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        self.delta = delta

    def update(self, acc, delta):
        if dpg.get_value("noise_checkbox"):
            delta_steering = delta - self.delta
            if abs(delta_steering) > max_delta_steering:
                delta_steering = max_delta_steering if delta_steering > 0 else -max_delta_steering
            self.delta += delta_steering

            self.x += self.vel * math.cos(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)
            self.y += self.vel * math.sin(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)

            self.yaw += self.vel * math.tan(self.delta) / WB * dt

            delta_speed = acc - self.vel
            if abs(delta_speed) > max_delta_speed:
                delta_speed = max_delta_speed if delta_speed > 0 else -max_delta_speed
            self.vel += delta_speed
        else:
            self.delta=delta
            self.x += self.vel * math.cos(self.yaw) * dt
            self.y += self.vel * math.sin(self.yaw) * dt
            self.yaw += self.vel * math.tan(self.delta) / WB * dt
            self.vel = acc

        #self.vel += acc * dt
        #self.vel = 1

# PI controller functions
class PI:
    def __init__(self, kp=1.0, ki=0.1):
        """
        Define a PID controller class
        :param kp: float, kp coeff
        :param ki: float, ki coeff
        :param kd: float, kd coeff
        """
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        """
        PID main function, given an input, this function will output a control unit
        :param error: float, error term
        :return: float, output control
        """
        self.Pterm = self.kp * error
        self.Iterm += error * dt

        self.last_error = error
        output = self.Pterm + self.ki * self.Iterm
        return output

# Functions of vehicle trajectory calculation
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

# IT2 model adjustable to any number of inputs and membership functions  
class IT2FLS:
    def __init__(self, n_entradas=2, n_mf=2, n_rules=4, nodos=np.zeros((19,2)), mf_p=MF_parameters_it2mf2, rules=Rules_it2mf2, c_p=C_parameters_it2mf2):
        self.n_entradas = n_entradas
        self.n_mf = n_mf
        self.n_rules = n_rules
        self.nodos = nodos
        self.MF_parametros_it2 = mf_p
        self.Reglas_it2 = rules
        self.C_parametros_it2 = c_p

    def model(self,entradas):
        self.nodos[0:self.n_entradas,0]=entradas

        for i in range(0,self.n_entradas):
            for j in range(0,self.n_mf):
                ind=self.n_entradas+i*self.n_mf+j
                x = entradas[i]
                pb = self.MF_parametros_it2[i*self.n_mf+j,2]
                pc = self.MF_parametros_it2[i*self.n_mf+j,3]
                pai = self.MF_parametros_it2[i*self.n_mf+j,0]
                tmp1i = (x - pc)/pai
                if tmp1i == 0:
                    tmp2i=0
                else:
                    tmp2i = (tmp1i*tmp1i)**pb
                self.nodos[ind,0]=self.MF_parametros_it2[i*self.n_mf+j,4]/(1+tmp2i)
                pas = self.MF_parametros_it2[i*self.n_mf+j,1]
                tmp1s = (x - pc)/pas
                if tmp1s == 0:
                    tmp2s=0
                else:
                    tmp2s = (tmp1s*tmp1s)**pb
                self.nodos[ind,1]=1/(1+tmp2s)

        st=self.n_entradas+self.n_entradas*self.n_mf
        for i in range(st,st+self.n_rules):
            self.nodos[i,0]=np.cumprod(self.nodos[self.Reglas_it2[i-st,:],0])[-1]
            self.nodos[i,1]=np.cumprod(self.nodos[self.Reglas_it2[i-st,:],1])[-1]

        st=self.n_entradas+self.n_entradas*self.n_mf
        wi=self.nodos[st:st+self.n_rules,0]
        ws=self.nodos[st:st+self.n_rules,1]

        wi_orig = wi.copy()
        wi = np.sort(wi)
        ws = np.sort(ws)
        order = wi_orig.argsort()
        wn = (wi+ws)/2
        yi = np.sum(wi * wn)/np.sum(wn)
        ys = np.sum(ws * wn)/np.sum(wn)

        l=0
        r=0
        Xl=np.zeros(self.n_rules)
        Xr=np.zeros(self.n_rules)

        for i in range(0,self.n_rules-1):
            if wi[i]<=yi and yi<=wi[i+1]:
                l=i
            if ws[i]<=ys and ys<=ws[i+1]:
                r=i

        Xl = np.divide(np.concatenate((ws[0:l+1], wi[l+1:]), axis=0),np.sum(np.concatenate((ws[0:l+1], wi[l+1:]), axis=0)))
        Xr = np.divide(np.concatenate((wi[0:r+1], ws[r+1:]), axis=0),np.sum(np.concatenate((wi[0:r+1], ws[r+1:]), axis=0)))
        X = ((Xl+Xr)/2)[order]

        st=self.n_entradas+self.n_entradas*self.n_mf+2*self.n_rules
        for i in range(0,self.n_rules):
           self.nodos[i+st,0] = X[i]*(np.sum(self.C_parametros_it2[i,0:-1]*entradas)+self.C_parametros_it2[i,-1])

        y = np.sum(self.nodos[-self.n_rules-1:-1,0])
        return y