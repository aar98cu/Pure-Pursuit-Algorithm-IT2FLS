import dearpygui.dearpygui as dpg
from config import *
from vehicle import *
import math
import numpy as np
import time
import matplotlib.pyplot as plt

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

def geometric_lateral_control(params):
    delta = math.atan2((2*1.3*math.sin(params.get("yaw_error", 0)))/params.get("ld", 0),1)
    return delta

def based_on_error(params):
    delta = params.get("yaw_error", 0)
    return delta

def it2fls(params):
    steering = IT2FLS()
    delta = steering.model(np.array([params.get("yaw_error", 0), params.get("d_yaw_error", 0)]))
    return delta

def t1fls(params):
    return

def update_control(sender, app_data):
    global control_func
    selected_control = app_data
    
    control_func = control_functions[selected_control]

control_functions = {
    "Geometric Lateral Control": geometric_lateral_control,
    "Based on error": based_on_error,
    "IT2FLS": it2fls,
    "T1FLS": t1fls
}
    
def run_simulation():
    ego =  Mathematic_Model(0,0,1.5708,0)

    x_data, y_data = dpg.get_value("trayectoria_plot")

    ruta = np.stack((x_data, y_data), axis=-1)
    traj = Trajectory(x_data, y_data)
    goal = traj.getPoint(len(x_data) - 1)

    PI_acc = PI()

    traj_ego_x = []
    traj_ego_y = []

    elm1 = [0]
    elm2 = [0]
    hem = [0]
    index = 0

    with open("./results/simulation.txt", "w") as file:
        file.write("mse_lateral_error,mse_heading_error\n")
        
        while getDistance([ego.x, ego.y], goal) > 1 or index<len(x_data)-2:
            target_point, current, index = traj.getTargetPoint([ego.x, ego.y])

            vel_err = math.sqrt(math.pow(target_point[1] - ego.y,2)+math.pow(target_point[0] - ego.x,2))-L
            acc = np.clip(PI_acc.control(vel_err), 0, 2)

            distancias = np.sqrt((ruta[:, 0] - ego.x)**2 + (ruta[:, 1] - ego.y)**2)
            indice_punto_cercano = np.argmin(distancias)

            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw

            yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
            
            if index > 0:
                elm1.append(current*math.sin(yaw_err))
                elm2.append(distancias[indice_punto_cercano]*math.sin(math.atan2(ruta[indice_punto_cercano, 1] - ego.y, ruta[indice_punto_cercano, 0] - ego.x) - ego.yaw))
                hem.append(yaw_err)
                file.write(f"{(np.mean(np.square(elm2)))},{(np.mean(np.square(hem)))}\n")

            try:
                params = {
                    "yaw_error": yaw_err,
                    "d_yaw_error": (hem[-1]-hem[-2])/dt,
                    "ld": current,
                }
            except:
                params = {
                    "yaw_error": yaw_err,
                    "d_yaw_error": 0,
                    "ld": current,
                }

            delta = np.clip(control_func(params), -0.698132, 0.698132)

            ego.update(acc, delta)

            traj_ego_x.append(ego.x)
            traj_ego_y.append(ego.y)

            dpg.set_value("x", f"X: {ego.x:.2f}m")
            dpg.set_value("y", f"Y: {ego.y:.2f}m")
            dpg.set_value("theta", f"Theta: {(ego.yaw*180/math.pi):.2f}°")
            dpg.set_value("v", f"V: {ego.vel:.2f}m/s")
            dpg.set_value("delta", f"Delta: {(delta*180/math.pi):.2f}°")
            dpg.set_value("le1", f"Lateral error 1: {elm1[-1]:.2f}m")
            dpg.set_value("mle1", f"Mean lateral error 1: {(np.mean(np.square(elm1))):.2f}m")
            dpg.set_value("le2", f"Lateral error 2: {elm2[-1]:.2f}m")
            dpg.set_value("mle2", f"Mean lateral error 2: {(np.mean(np.square(elm2))):.2f}m")
            dpg.set_value("he", f"Heading error: {(yaw_err*180/math.pi):.2f}°")
            dpg.set_value("mhe", f"Mean heading error 1: {(np.mean(np.square(hem))):.2f}°")
            
            update_vehicle(ego.x, ego.y, ego.yaw, delta)
            dpg.set_value("reference_plot", [traj_ego_x, traj_ego_y])
            dpg.bind_item_theme("reference_plot", "reference_theme")
            dpg.fit_axis_data("xaxis")
            dpg.fit_axis_data("yaxis")
            time.sleep(dt)

        plt.figure()
        plt.plot(elm1, label='Error lateral método 1')
        plt.plot(elm2, label='Error lateral método 2')
        plt.xlabel('Iteración')
        plt.ylabel('Error lateral')
        plt.legend()
        plt.grid(True)
        plt.title('Errores laterales durante el seguimiento de la trayectoria')
        plt.show()

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
        if dpg.get_value("noise_checkbox"):
            self.x += self.vel * math.cos(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)
            self.y += self.vel * math.sin(self.yaw) * dt + np.random.uniform(-noise_std_dev, noise_std_dev)
        else:
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
    
class IT2FLS:
    def __init__(self, n_entradas=2, n_mf=2, n_reglas=4, nodos=np.zeros((19,2))):
        self.n_entradas = n_entradas
        self.n_mf = n_mf
        self.n_reglas = n_reglas
        self.nodos = nodos

    def model(self,entradas):
        self.nodos[0:self.n_entradas,0]=entradas

        for i in range(0,self.n_entradas):
            for j in range(0,self.n_mf):
                ind=self.n_entradas+i*self.n_mf+j
                x = entradas[i]
                pb = MF_parametros_it2[i*self.n_mf+j,2]
                pc = MF_parametros_it2[i*self.n_mf+j,3]
                pai = MF_parametros_it2[i*self.n_mf+j,0]
                tmp1i = (x - pc)/pai
                if tmp1i == 0:
                    tmp2i=0
                else:
                    tmp2i = (tmp1i*tmp1i)**pb
                self.nodos[ind,0]=MF_parametros_it2[i*self.n_mf+j,4]/(1+tmp2i)
                pas = MF_parametros_it2[i*self.n_mf+j,1]
                tmp1s = (x - pc)/pas
                if tmp1s == 0:
                    tmp2s=0
                else:
                    tmp2s = (tmp1s*tmp1s)**pb
                self.nodos[ind,1]=1/(1+tmp2s)

        st=self.n_entradas+self.n_entradas*self.n_mf
        for i in range(st,st+self.n_reglas):
            self.nodos[i,0]=np.cumprod(self.nodos[Reglas_it2[i-st,:],0])[-1]
            self.nodos[i,1]=np.cumprod(self.nodos[Reglas_it2[i-st,:],1])[-1]

        st=self.n_entradas+self.n_entradas*self.n_mf
        wi=self.nodos[st:st+self.n_reglas,0]
        ws=self.nodos[st:st+self.n_reglas,1]

        wi_orig = wi.copy()
        wi = np.sort(wi)
        ws = np.sort(ws)
        order = wi_orig.argsort()
        wn = (wi+ws)/2
        yi = np.sum(wi * wn)/np.sum(wn)
        ys = np.sum(ws * wn)/np.sum(wn)

        l=0
        r=0
        Xl=np.zeros(self.n_reglas)
        Xr=np.zeros(self.n_reglas)

        for i in range(0,self.n_reglas-1):
            if wi[i]<=yi and yi<=wi[i+1]:
                l=i
            if ws[i]<=ys and ys<=ws[i+1]:
                r=i

        Xl = np.divide(np.concatenate((ws[0:l+1], wi[l+1:]), axis=0),np.sum(np.concatenate((ws[0:l+1], wi[l+1:]), axis=0)))
        Xr = np.divide(np.concatenate((wi[0:r+1], ws[r+1:]), axis=0),np.sum(np.concatenate((wi[0:r+1], ws[r+1:]), axis=0)))
        X = ((Xl+Xr)/2)[order]

        st=self.n_entradas+self.n_entradas*self.n_mf+2*self.n_reglas
        for i in range(0,self.n_reglas):
           self.nodos[i+st,0] = X[i]*(np.sum(C_parametros_it2[i,0:-1]*entradas)+C_parametros_it2[i,-1])

        y = np.sum(self.nodos[-self.n_reglas-1:-1,0])
        return y