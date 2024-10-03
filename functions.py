import math
import time
import numpy as np
from config import *
from vehicle import *
import matplotlib.pyplot as plt
import dearpygui.dearpygui as dpg
import seaborn as sns

# Adjust the size of the interface to fit the current viewport dimensions
def fit_interface_size(sender, app_data):
    global width
    global height
    
    width = dpg.get_viewport_width() # Get the current width of the viewport
    height = dpg.get_viewport_height() # Get the current height of the viewport
    
    # Update the size of the main window and the panels
    dpg.configure_item("window", width=width, height=height)
    dpg.configure_item("left_panel", width=int(width * 0.2), height=40)
    dpg.configure_item("right_panel", width=int(width * 0.77), height=int(height*0.9))

# Update and display the selected path on the plot
def update_path(sender, app_data):
    seleccion = app_data # Selected path from dropdown
    x_data, y_data = zip(*paths[seleccion]) # Extract x and y data points
    
    # Update the plot with the new path data
    dpg.set_value("path_plot", [x_data, y_data])
    dpg.bind_item_theme("path_plot", "path_theme")  # Apply the path theme
    dpg.fit_axis_data("xaxis") # Adjust the x-axis to fit the new data
    dpg.fit_axis_data("yaxis") # Adjust the y-axis to fit the new data

    update_vehicle(x_data[0]-2,y_data[0]-2,1.5708,0)

# Calculate the steering angle using geometric lateral control
def steering_geometric_lateral_control():
    delta = math.atan2((2*1.3*math.sin(params["yaw_error"]))/params["ld"],1)
    return delta

# Calculate the steering angle based on yaw error
def steering_based_on_error():
    delta = params["yaw_error"]
    return delta

# Calculate the steering angle using IT2FLS with 2 membership functions
def steering_it2fls_2mf():
    steering = IT2FLS(n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((19,2)), mf_p=MF_parameters_it2mf2_delta, rules=Rules_it2mf2, c_p=C_parameters_it2mf2_delta)
    delta = steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))
    return delta

# Calculate the steering angle using IT2FLS with 3 membership functions
def steering_it2fls_3mf():
    steering = IT2FLS(n_inputs=2, n_mf=3, n_rules=9, nodes=np.zeros((36,2)), mf_p=MF_parameters_it2mf3_delta, rules=Rules_it2mf3, c_p=C_parameters_it2mf3_delta)
    delta = steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))
    return delta

# Calculate the steering angle using T1FLS with 2 membership functions
def steering_t1fls_2mf():
    steering = T1FLS(n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((16,1)), mf_p=MF_parameters_t1mf2_delta, rules=Rules_t1mf2, c_p=C_parameters_t1mf2_delta)
    delta = steering.model(np.array([params["yaw_error"], params["d_yaw_error"]]))
    return delta

# Update the selected steering control function based on user selection
def update_steering_control(sender, app_data):
    global steering_control_func
    steering_control_func = steering_control_functions[app_data] # Assign the selected control function

# Dictionary mapping steering control names to their corresponding functions
steering_control_functions = {
    "Geometric Lateral Control": steering_geometric_lateral_control,
    "Based on error": steering_based_on_error,
    "T1FLC with 2MF": steering_t1fls_2mf,
    "IT2FLC with 2MF": steering_it2fls_2mf,
    "IT2FLC with 3MF": steering_it2fls_3mf
}

# Calculate the vehicle speed using a PI controller from Robotics, Vision and Control book
def velocity_pi_control(vel, PI_acc):
    vel_err = params["ld"]-look_ahead
    velocity = np.clip(PI_acc.control(vel_err), 0, dpg.get_value("speed_slider"))
    return velocity, vel_err

# Calculate the vehicle speed using a PI controller
def velocity_pi_control_2(vel,Pi_acc):
    vel_err = dpg.get_value("speed_slider")-vel
    velocity = np.clip(vel+PI_acc.control(vel_err)*dt, 0, dpg.get_value("speed_slider"))
    return velocity, vel_err

# Calculate the vehicle speed using IT2FLS with 2 membership functions
def velocity_it2fls_2mf(vel, PI_acc):
    vel_err = params["ld"]-look_ahead
    acc = IT2FLS(n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((19,2)), mf_p=MF_parameters_it2mf2_vel, rules=Rules_it2mf2, c_p=C_parameters_it2mf2_vel)
    velocity = np.clip(acc.model(np.array([vel_err, params["d_velocity_error"]])), 0, dpg.get_value("speed_slider"))
    return velocity, vel_err

# Update the selected velocity control type
def update_velocity_control(sender, app_data):
    global velocity_control_func
    velocity_control_func = velocity_control_functions[app_data]

# Dictionary mapping control types to their respective functions
velocity_control_functions = {
    "PI Distance Control": velocity_pi_control,
    "PI Velocity Control": velocity_pi_control_2,
    "IT2FLC with 2MF": velocity_it2fls_2mf
}

# Toggles the visibility of noise-related settings based on the noise checkbox state.
def toggle_noise_settings(sender, app_data):
    if dpg.get_value("noise_checkbox"):
        dpg.show_item("max_speed_delta")
        dpg.show_item("max_speed_delta_text")
        dpg.show_item("steering")

        if dpg.get_value("steering") == "Mathematical model":
            dpg.show_item("max_steering_delta")
            dpg.show_item("max_steering_delta_text")
    else:
        dpg.hide_item("max_speed_delta")
        dpg.hide_item("max_speed_delta_text")
        dpg.hide_item("steering")
        dpg.hide_item("max_steering_delta")
        dpg.hide_item("max_steering_delta_text")

# Updates the steering angle using a mathematical model.
# This function calculates the steering change (delta_steering) between the new (n_delta) and old (o_delta) commands, ensuring it doesn't exceed the maximum limit (max_delta_steering).
def mathematical_model(o_delta, n_delta):
    delta_steering = n_delta - o_delta
    if abs(delta_steering) > max_delta_steering:
        delta_steering = max_delta_steering if delta_steering > 0 else -max_delta_steering
    o_delta += delta_steering
    return o_delta

# Updates the steering angle using a transfer function model.
# Simulates a real steering system using proportional control based on the error between the new (n_delta) and old (o_delta) steering angles. The control signal is calculated using a transfer function over 5 iterations, clipped to the range [-255, 255], with the old angle updated in each step.
def transfer_function_model(o_delta, n_delta):
    for i in range(5):
        delta_error = n_delta - o_delta
        control_signal = np.clip(41.3006023567974*delta_error, -255, 255)
        o_delta = o_delta*0.9989+0.001911*control_signal
    return o_delta

# Updates the steering angle using a Type-1 Fuzzy Logic System (T1FLS).
# The controller calculates the error between the new (n_delta) and old (o_delta) steering angles, applies a proportional-derivative (PD) control, and maps the control signal to the range [-255, 255].
def t1fm(o_delta, n_delta):
    global previous_delta_error
    steering = T1FLS(n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((16,1)), mf_p=MF_parameters_t1mf2_steering, rules=Rules_t1mf2, c_p=C_parameters_t1mf2_steering)

    for i in range(5):
        delta_error = n_delta - o_delta
        control_signal = np.clip(0.8*delta_error+0.0001941*((delta_error-previous_delta_error)/dt), -1, 1)
        control_signal = np.interp(control_signal, [-1, 1], [-255, 255])
        previous_delta_error = delta_error
        o_delta = steering.model(np.array([o_delta, control_signal]))
    return o_delta

# Dictionary mapping modes types to their respective functions
steering_mode_functions = {
    "Mathematical model": mathematical_model,
    "Transfer function model": transfer_function_model,
    "T1FLS": t1fm
}

# Update the selected steering mode
def update_steering_mode(sender, app_data):
    global steering_mode_func
    steering_mode_func = steering_mode_functions[app_data]

    selected_mode = app_data
    if selected_mode == "Mathematical model":
        dpg.show_item("max_steering_delta")
        dpg.show_item("max_steering_delta_text")
    else:
        dpg.hide_item("max_steering_delta")
        dpg.hide_item("max_steering_delta_text")

# Function to run the simulation on the selected path with the chosen control settings    
def run_simulation(return_errors=False):
    global max_delta_steering, max_delta_speed

    # Disable interface elements during the simulation
    dpg.disable_item("speed_slider")
    dpg.disable_item("noise_checkbox")
    dpg.disable_item("run_button")
    dpg.disable_item("path")
    dpg.disable_item("control")
    dpg.disable_item("max_steering_delta")
    dpg.disable_item("max_speed_delta")
    dpg.disable_item("speed")
    dpg.disable_item("steering")

    # Update maximum steering and speed delta values from the interface
    max_delta_steering = math.radians(dpg.get_value("max_steering_delta"))
    max_delta_speed = dpg.get_value("max_speed_delta")

    # Extract x and y coordinates from the selected path
    x_data, y_data = dpg.get_value("path_plot")
    path = np.stack((x_data, y_data), axis=-1)
    traj = Trajectory(x_data, y_data)
    goal = traj.getPoint(len(x_data) - 1)

    # Initialize vehicle model
    ego =  Mathematic_Model(x_data[0]-2,y_data[0]-2,1.5708)

    # Initialize PI controller
    PI_acc = PI()

    # Initialize lists for tracking simulation metrics
    traj_ego_x, traj_ego_y = [], []
    lateral_error_list = []
    heading_error_list = []
    velocity_error_list = []
    speed_list = []
    delta_list = []
    index = 0

    # Define file path to save results
    file_path = f'./results/{dpg.get_value("control")} in {dpg.get_value("path")} {"with noise" if dpg.get_value("noise_checkbox") else "without noise"} and speed {dpg.get_value("speed_slider")}.txt'    

    with open(file_path, "w") as file:
        file.write("mse_lateral_error,mse_heading_error\n")
        
        while getDistance([ego.x, ego.y], goal) > 1 or index<len(x_data)-2:
            # Calculate the target point and update parameters
            target_point, current, index = traj.getTargetPoint([ego.x, ego.y])
            params["ld"] = current

            # Update velocity control parameters
            velocity, vel_err = velocity_control_func(ego.vel, PI_acc)
            velocity_error_list.append(vel_err)
            params["velocity_error"] = velocity_error_list[-1]
            params["d_velocity_error"] = (velocity_error_list[-1] - (velocity_error_list[-2] if len(velocity_error_list) > 1 else 0)) / dt

            # Calculate nearest point and errors
            distances = np.sqrt((path[:, 0] - ego.x)**2 + (path[:, 1] - ego.y)**2)
            index_near_point = np.argmin(distances)
            yaw_err_mp = math.atan2(path[index_near_point, 1] - ego.y, path[index_near_point, 0] - ego.x) - ego.yaw
            yaw_err_mp = np.arctan2(np.sin(yaw_err_mp), np.cos(yaw_err_mp))
            lateral_error_list.append(distances[index_near_point]*math.sin(yaw_err_mp))

            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            heading_error_list.append(yaw_err)            
            params["yaw_error"] = heading_error_list[-1]
            params["d_yaw_error"] = (heading_error_list[-1] - (heading_error_list[-2] if len(heading_error_list) > 1 else 0)) / dt
            
            # Apply steering control and update vehicle state
            delta = np.clip(steering_control_func(), -0.698132, 0.698132)
            ego.update(velocity, delta)

            speed_list.append(ego.vel)
            delta_list.append(math.degrees(ego.delta))

            traj_ego_x.append(ego.x)
            traj_ego_y.append(ego.y)
            
            if return_errors!=True:
                file.write(f'{(np.mean(np.square(lateral_error_list)))},{(math.degrees(np.mean(np.square(heading_error_list))))}\n')

            # Update interface values for real-time display
            dpg.set_value("x", f"X: {ego.x:.2f}m")
            dpg.set_value("y", f"Y: {ego.y:.2f}m")
            dpg.set_value("theta", f"Theta: {math.degrees(ego.yaw):.2f}°")
            dpg.set_value("v", f"V: {ego.vel:.2f}m/s")
            dpg.set_value("delta", f"Delta: {math.degrees(ego.delta):.2f}°")
            dpg.set_value("le", f"Lateral error: {lateral_error_list[-1]:.2f}m")
            dpg.set_value("msele", f"MSE lateral: {(np.mean(np.square(lateral_error_list))):.2f}m")
            dpg.set_value("he", f"Heading error: {math.degrees(heading_error_list[-1]):.2f}°")
            dpg.set_value("msehe", f"MSE heading: {math.degrees(np.mean(np.square(heading_error_list))):.2f}°")
            
            update_vehicle(ego.x, ego.y, ego.yaw, ego.delta)
            dpg.set_value("reference_plot", [traj_ego_x, traj_ego_y])
            dpg.bind_item_theme("reference_plot", "reference_theme")
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")
            time.sleep(dt)

    # Re-enable interface elements after simulation ends
    dpg.enable_item("speed_slider")
    dpg.enable_item("noise_checkbox")
    dpg.enable_item("run_button")
    dpg.enable_item("path")
    dpg.enable_item("control")
    dpg.enable_item("max_steering_delta")
    dpg.enable_item("max_speed_delta")
    dpg.enable_item("speed")
    dpg.enable_item("steering")

    if return_errors==True:
        # Return the lateral error list and heading error list after running multiple simulations.
        return np.mean(np.square(lateral_error_list)), math.degrees(np.mean(np.square(heading_error_list)))
    else:
        # Plot simulation results
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

def run_multiple_simulations():
    mse_lateral_errors = []
    mse_heading_errors = []
    
    for _ in range(3):
        mse_lateral, mse_heading = run_simulation(return_errors=True)
        mse_lateral_errors.append(mse_lateral)
        mse_heading_errors.append(mse_heading)

    avg_mse_lateral = np.mean(mse_lateral_errors)
    avg_mse_heading = np.mean(mse_heading_errors)
    std_mse_lateral = np.std(mse_lateral_errors)
    std_mse_heading = np.std(mse_heading_errors)

    print(f"Average MSE Lateral Error: {avg_mse_lateral:.4f} ± {std_mse_lateral:.4f}")
    print(f"Average MSE Heading Error: {avg_mse_heading:.4f} ± {std_mse_heading:.4f}")

    # Gráfico de línea comparativa para cada corrida
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(range(1, len(mse_lateral_errors) + 1), mse_lateral_errors, marker='o', color='b', linestyle='-', markersize=6)
    plt.title('MSE Lateral Errors per Test')
    plt.xlabel('Test Number')
    plt.ylabel('MSE Lateral Error')

    plt.subplot(1, 2, 2)
    plt.plot(range(1, len(mse_heading_errors) + 1), mse_heading_errors, marker='o', color='r', linestyle='-', markersize=6)
    plt.title('MSE Heading Errors per Test')
    plt.xlabel('Test Number')
    plt.ylabel('MSE Heading Error')

    plt.tight_layout()
    plt.show()

    # Boxplot para mostrar la distribución de los errores
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    sns.boxplot(data=mse_lateral_errors, color='b')
    plt.title('MSE Lateral Errors (Boxplot)')
    plt.ylabel('MSE Lateral Error')

    plt.subplot(1, 2, 2)
    sns.boxplot(data=mse_heading_errors, color='r')
    plt.title('MSE Heading Errors (Boxplot)')
    plt.ylabel('MSE Heading Error')

    plt.tight_layout()
    plt.show()

# Calculate the distance between two points
def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

class Mathematic_Model:
    def __init__(self, x, y, yaw, vel=0, delta=0):
        """
        Initialize the vehicle's state.
        :param x: float, initial x position
        :param y: float, initial y position
        :param yaw: float, initial heading (yaw angle in radians)
        :param vel: float, initial velocity (default is 0)
        :param delta: float, initial steering angle (default is 0) 
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        self.delta = delta

    def update(self, velocity, delta):
        """
        Update the vehicle's state based on velocity and steering angle.
        :param velocity: float, the current velocity of the vehicle
        :param delta: float, the current steering angle (in radians)
        """
        if dpg.get_value("noise_checkbox"):
            # Apply steering and velocity constraints with noise

            self.delta = steering_mode_func(self.delta, delta)

            """delta_steering = delta - self.delta
            if abs(delta_steering) > max_delta_steering:
                delta_steering = max_delta_steering if delta_steering > 0 else -max_delta_steering
            self.delta += delta_steering"""

            self.x += self.vel * math.cos(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)
            self.y += self.vel * math.sin(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)

            self.yaw += self.vel * math.tan(self.delta) / WB * dt

            delta_speed = velocity - self.vel
            if abs(delta_speed) > max_delta_speed:
                delta_speed = max_delta_speed if delta_speed > 0 else -max_delta_speed
            self.vel += delta_speed
        else:
            # Update state without noise
            self.delta=delta
            self.x += self.vel * math.cos(self.yaw) * dt
            self.y += self.vel * math.sin(self.yaw) * dt
            self.yaw += self.vel * math.tan(self.delta) / WB * dt
            self.vel = velocity

        #self.vel += acc * dt
        #self.vel = 1

class PI:
    def __init__(self, kp=1.0, ki=0.1):
        """
        Initialize the PI controller.
        :param kp: float, proportional gain
        :param ki: float, integral gain 
        """
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        """
        Compute the control output based on the error.
        :param error: float, the current error value
        :return: float, the control output
        """
        self.Pterm = self.kp * error
        self.Iterm += error * dt

        self.last_error = error
        output = self.Pterm + self.ki * self.Iterm
        return output

# Update to best usage
PI_acc = PI()

# Functions of vehicle trajectory calculation
class Trajectory:
    """
    Initialize the trajectory with x and y coordinates.
    :param traj_x: list of floats, x coordinates of the trajectory
    :param traj_y: list of floats, y coordinates of the trajectory
    """
    def __init__(self, traj_x, traj_y):
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        """
        Get the trajectory point at a given index.
        :param idx: int, index of the desired point
        :return: list of float, [x, y] coordinates of the trajectory point
        """
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        """
        Get the target point along the trajectory based on the current position.
        :param pos: list of float, current [x, y] position of the vehicle
        :return: tuple, target [x, y] point, current distance to the point, and updated index
        """
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < look_ahead and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)
            
        self.last_idx = target_idx
        return self.getPoint(target_idx), curr_dist, self.last_idx

class IT2FLS:
    """
    Initialize an IT2FLS model.
    :param n_inputs: int, number of inputs to the model
    :param n_mf: int, number of membership functions per input
    :param n_rules: int, number of fuzzy rules
    :param nodes: numpy array, storage for node calculations
    :param mf_p: numpy array, parameters of the membership functions
    :param rules: numpy array, fuzzy rule base
    :param c_p: numpy array, consequent parameters for the rules
    """
    def __init__(self, n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((19,2)), mf_p=MF_parameters_it2mf2_delta, rules=Rules_it2mf2, c_p=C_parameters_it2mf2_delta):
        self.n_inputs = n_inputs
        self.n_mf = n_mf
        self.n_rules = n_rules
        self.nodes = nodes
        self.MF_parameters_it2 = mf_p
        self.Rules_it2 = rules
        self.C_parameters_it2 = c_p

    def model(self,entradas):
        """
        Perform inference based on the IT2FLS model.
        :param entradas: numpy array, input values for the model
        :return: float, output of the fuzzy logic model
        """
        self.nodes[0:self.n_inputs,0]=entradas

        for i in range(0,self.n_inputs):
            for j in range(0,self.n_mf):
                ind=self.n_inputs+i*self.n_mf+j
                x = entradas[i]
                pb = self.MF_parameters_it2[i*self.n_mf+j,2]
                pc = self.MF_parameters_it2[i*self.n_mf+j,3]
                pai = self.MF_parameters_it2[i*self.n_mf+j,0]
                tmp1i = (x - pc)/pai
                if tmp1i == 0:
                    tmp2i=0
                else:
                    tmp2i = (tmp1i*tmp1i)**pb
                self.nodes[ind,0]=self.MF_parameters_it2[i*self.n_mf+j,4]/(1+tmp2i)
                pas = self.MF_parameters_it2[i*self.n_mf+j,1]
                tmp1s = (x - pc)/pas
                if tmp1s == 0:
                    tmp2s=0
                else:
                    tmp2s = (tmp1s*tmp1s)**pb
                self.nodes[ind,1]=1/(1+tmp2s)

        st=self.n_inputs+self.n_inputs*self.n_mf
        for i in range(st,st+self.n_rules):
            self.nodes[i,0]=np.cumprod(self.nodes[self.Rules_it2[i-st,:],0])[-1]
            self.nodes[i,1]=np.cumprod(self.nodes[self.Rules_it2[i-st,:],1])[-1]

        st=self.n_inputs+self.n_inputs*self.n_mf
        wi=self.nodes[st:st+self.n_rules,0]
        ws=self.nodes[st:st+self.n_rules,1]

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

        st=self.n_inputs+self.n_inputs*self.n_mf+2*self.n_rules
        for i in range(0,self.n_rules):
           self.nodes[i+st,0] = X[i]*(np.sum(self.C_parameters_it2[i,0:-1]*entradas)+self.C_parameters_it2[i,-1])

        y = np.sum(self.nodes[-self.n_rules-1:-1,0])
        return y
    
class T1FLS:
    """
    Initialize a T1FLS model.
    :param n_inputs: int, number of inputs to the model
    :param n_mf: int, number of membership functions per input
    :param n_rules: int, number of fuzzy rules
    :param nodes: numpy array, storage for node calculations
    :param mf_p: numpy array, parameters of the membership functions
    :param rules: numpy array, fuzzy rule base
    :param c_p: numpy array, consequent parameters for the rules
    """
    def __init__(self, n_inputs=2, n_mf=2, n_rules=4, nodes=np.zeros((16,1)), mf_p=MF_parameters_t1mf2_delta, rules=Rules_t1mf2, c_p=C_parameters_t1mf2_delta):
        self.n_inputs = n_inputs
        self.n_mf = n_mf
        self.n_rules = n_rules
        self.nodes = nodes
        self.MF_parameters_t1 = mf_p
        self.Rules_t1 = rules
        self.C_parameters_t1 = c_p

    def model(self,entradas):
        """
        Perform inference based on the T1FLS model.
        :param entradas: numpy array, input values for the model
        :return: float, output of the fuzzy logic model
        """
        for i in range(0,self.n_inputs):
            for j in range(0,self.n_mf):
                ind=i*self.n_mf+j
                x = entradas[i]
                pb = self.MF_parameters_t1[i*self.n_mf+j,1]
                pc = self.MF_parameters_t1[i*self.n_mf+j,2]
                pa = self.MF_parameters_t1[i*self.n_mf+j,0]
                tmp1 = (x - pc)/pa
                if tmp1 == 0:
                    tmp2=0
                else:
                    tmp2 = (tmp1*tmp1)**pb
                self.nodes[ind,0]=1/(1+tmp2)
        
        st=self.n_inputs*self.n_mf
        for i in range(st,st+self.n_rules):
            self.nodes[i,0]=np.cumprod(self.nodes[self.Rules_t1[i-st,:],0])[-1]

        st=self.n_inputs*self.n_mf+self.n_rules
        for i in range(st,st+self.n_rules):
            self.nodes[i,0]=self.nodes[i-self.n_rules,0]/np.sum(self.nodes[st-self.n_rules:st,0])

        st=self.n_inputs*self.n_mf+2*self.n_rules
        for i in range(0,self.n_rules):
           self.nodes[i+st,0] = self.nodes[st-self.n_rules+i,0]*(np.sum(self.C_parameters_t1[i,0:-1]*entradas)+self.C_parameters_t1[i,-1])

        y = np.sum(self.nodes[-self.n_rules:,0])
        return y