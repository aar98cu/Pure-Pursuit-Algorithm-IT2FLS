import dearpygui.dearpygui as dpg
from config import *
from vehicle import *
import math
import numpy as np
import time

def fit_interface_size(sender, app_data):
    global width
    global height
    
    width = dpg.get_viewport_width()
    height = dpg.get_viewport_height()
    
    dpg.configure_item("window", width=width, height=height)
    
    dpg.configure_item("left_panel", width=int(width * 0.2), height=40)
    dpg.configure_item("right_panel", width=int(width * 0.77), height=int(height*0.9))

def update_trayectoria(sender, app_data):
    seleccion = app_data
    x_data, y_data = zip(*trayectorias[seleccion])
    
    dpg.set_value("trayectoria_plot", [x_data, y_data])
    dpg.bind_item_theme("trayectoria_plot", "path_theme")
    dpg.fit_axis_data("xaxis")
    dpg.fit_axis_data("yaxis")

def update_control(sender, app_data):
    seleccion = app_data  # El nombre de la trayectoria seleccionada
    x_data, y_data = zip(*control[seleccion])  # Extraer puntos x, y de la trayectoria seleccionada
    
def run_simulation():
    ego =  Mathematic_Model(0,0,1.5708,0)

    x_data, y_data = dpg.get_value("trayectoria_plot")

    ruta = np.stack((x_data, y_data), axis=-1)
    traj = Trajectory(x_data, y_data)
    goal = traj.getPoint(len(x_data) - 1)

    PI_acc = PI()

    traj_ego_x = []
    traj_ego_y = []

    elm1 = []
    elm2 = []

    while getDistance([ego.x, ego.y], goal) > 1:
        noisy_x = ego.x + np.random.normal(-noise_std_dev, noise_std_dev)
        noisy_y = ego.y + np.random.normal(-noise_std_dev, noise_std_dev)
        #Agregar el ruido
        target_point, current, index = traj.getTargetPoint([ego.x, ego.y])

        # use PID to control the vehicle
        #vel_err = target_vel - ego.vel
        vel_err = math.sqrt(math.pow(target_point[1] - ego.y,2)+math.pow(target_point[0] - ego.x,2))-L
        acc = PI_acc.control(vel_err)

        yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
        """
        if (target_point[0]<ego.x and target_point[1]>ego.y):
            yaw_err = (math.atan(abs(target_point[0]-ego.x)/abs(target_point[1]-ego.y))*180/math.pi+90)*math.pi/180 - ego.yaw
        elif (target_point[0]<ego.x and target_point[1]<ego.y):
            yaw_err = (math.atan(abs(target_point[1]-ego.y)/abs(target_point[0]-ego.x))*180/math.pi+180)*math.pi/180 - ego.yaw
        elif(target_point[1]<ego.y and target_point[0]>ego.x):
            yaw_err = (math.atan(abs(target_point[0]-ego.x)/abs(target_point[1]-ego.y))*180/math.pi+270)*math.pi/180 - ego.yaw
        else:
            yaw_err = (math.atan(abs(target_point[1]-ego.y)/abs(target_point[0]-ego.x))*180/math.pi)*math.pi/180 - ego.yaw
        """

        #yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

        distancias = np.sqrt((ruta[:, 0] - ego.x)**2 + (ruta[:, 1] - ego.y)**2)
        indice_punto_cercano = np.argmin(distancias)
        
        # Calcular el error lateral
        if index > 0:
            elm1.append(current*math.sin(yaw_err))
            elm2.append(distancias[indice_punto_cercano]*math.sin(math.atan2(ruta[indice_punto_cercano, 1] - ego.y, ruta[indice_punto_cercano, 0] - ego.x) - ego.yaw))
            #print(sum([abs(el1) for el1 in elm1])/len(elm1), sum([abs(el2) for el2 in elm2])/len(elm2))

        delta = math.atan2((2*1.3*math.sin(yaw_err))/current,1)

        """
        error_theta_g = yaw_err*180/math.pi
        delta = calculo_volante.modelo(np.array([error_theta_g, (error_theta_g-error_theta_a)/0.2]))*math.pi/180
        error_theta_a=error_theta_g
        """

        #delta=math.atan(yaw_err*1.3/0.6870) #Quite la división de yaw_error entre el TM

        #Volante usando el error de self.yaw como referencia  
        #delta = yaw_err
        
        #Volante usando una ecuación de la literatura
        #delta=math.atan2(1.3*(2.0/(L**2)*((target_point[1]-ego.y)*math.cos(yaw_err)-(target_point[0]-ego.x)*math.sin(yaw_err))),1)

        #Volante usando un controlador PID basado en el error lateral
        #delta = m(PI_yaw.control(yaw_err))
        
        #delta = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

        #Volante usando una ecuación de la literatura - Enhanced Pure Pursuit Algorithm & Autonomous Driving
        #delta=math.atan2((2*1.3*math.sin(yaw_err))/L,1)
        #delta=math.atan2((2*1.3*math.sin(yaw_err))/current,1)

        #Optimal Fuzzy... Alejandra Mancilla - No funciona
        #delta = math.atan((WB*(ego.vel/WB*math.tan(ego.delta)))/(ego.vel+0.000000000000000001)) #Optimal Fuzzy... Alejandra Mancilla - No funciona
        
        if delta>0.698132:
            delta=0.698132
        elif delta<-0.698132:
            delta=-0.698132

        # move the vehicle
        ego.update(acc, delta)

        # store the trajectory
        traj_ego_x.append(ego.x)
        traj_ego_y.append(ego.y)

        dpg.add_line_series(traj_ego_x, traj_ego_y,parent="yaxis")
        update_vehicle(ego.x, ego.y, ego.yaw, delta)
        time.sleep(dt)

def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

class Mathematic_Model:
    def __init__(self, x, y, yaw, vel=0, delta=0):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        self.delta = delta

    def update(self, acc, delta):
        self.xa = self.x
        self.ya = self.y
        self.delta=delta
        self.x += self.vel * math.cos(self.yaw) * dt
        self.y += self.vel * math.sin(self.yaw) * dt
        self.yaw += self.vel * math.tan(self.delta) / WB * dt
        self.vel = acc
        #self.vel += acc * dt
        #self.vel = 1

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

        while curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)
            
        self.last_idx = target_idx
        return self.getPoint(target_idx), curr_dist, self.last_idx