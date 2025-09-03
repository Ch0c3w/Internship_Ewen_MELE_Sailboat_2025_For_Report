import os
import sys
import numpy as np
import matplotlib.pyplot as plt
%matplotlib qt
import simplekml
import time

def sawtooth(x : float) -> float:
  return 2*np.arctan(np.tan(x/2));

def read_log(log_number : int) -> np.ndarray:
    logfilename = 'Log_files/NAVLOG' + str(log_number) + '.TXT'
    logfile = open(logfilename, 'r')
    logs = []
    line = logfile.readline()
    while line:
        values = line.split()
        logs.append([float(val) for val in values])
        line = logfile.readline()
    logs = np.array(logs).T
    # print(logs)
    return logs
    
def get_latest_log_number() -> int:
    file_list = os.listdir('Log_files')
    max_number = 0
    for filename in file_list:
        if filename[0:6] == 'NAVLOG' and int(filename[6:-4]) > max_number and filename[-4:] == '.TXT':
            max_number = int(filename[6:-4])
    return max_number

def draw_trajectory(logs : np.ndarray) -> None:
    lat = logs[1] * np.pi / 180
    long = logs[2] * np.pi / 180
    R = 6371e3
    x = R * (long - long[0]) * np.cos(lat[0])
    y = R * (lat - lat[0])
    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.set_title("Boat's trajectory")
    return

def toKML(log_number : int):
    # https://developers.google.com/kml/documentation/kml_tut
    # https://simplekml.readthedocs.io/en/latest/index.html
    logfilename = 'Log_files/NAVLOG' + str(log_number) + '.TXT'
    kmlfilename = 'KML_files/KML_' + str(log_number) + '.kml'
    kml = simplekml.Kml()
    ls = kml.newlinestring(name='GPS data from NAVLOG' + str(log_number) + '.TXT')
    ls.style.linestyle.width = 5
    ls.style.linestyle.color = simplekml.Color.blue
    logfile = open(logfilename, 'r')
    gps_data = []
    line = logfile.readline()
    while line:
        values = line.split()
        gps_data.append((values[2], values[1]))
        line = logfile.readline()
    ls.coords = gps_data
    logfile.close()
    kml.save(kmlfilename)

def map2(x : float, in_min : float, in_max : float, out_min : float, out_max : float) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def conv_rud_com_to_pos(com: np.ndarray) -> np.ndarray:
    return np.clip(map2(com, 200, 410, 50, -50), -50, 50)

def conv_sail_com_to_pos(com: np.ndarray) -> np.ndarray:
    return np.clip(map2(com, 225, 350, 0, 90), 0, 90)
    

def GPS2cart(gpscoord : np.ndarray, refcoord : np.ndarray) -> np.ndarray:
    refcoord_rad = refcoord * np.pi / 180
    gpscoord_rad = gpscoord * np.pi / 180
    # print(f'GPS coordinates: {gpscoord_rad}')
    R = 6371e3
    x = R * (gpscoord_rad[1] - refcoord_rad[1]) * np.cos(refcoord_rad[0])
    y = R * (gpscoord_rad[0] - refcoord_rad[0])
    return np.array([[x], [y]])
    
def generate_animation(logs : np.ndarray, ref_point : np.ndarray) -> None:
    ref_point *= np.pi/180
    lat = logs[1] * np.pi / 180
    long = logs[2] * np.pi / 180
    head = logs[3] * np.pi / 180
    SOG = logs[4]
    rud_angles = conv_rud_com_to_pos(logs[7]) * np.pi / 180
    sail_angles = conv_sail_com_to_pos(logs[8]) * np.pi / 180
    R = 6371e3
    x = R * (long - ref_point[1]) * np.cos(ref_point[0])
    y = R * (lat - ref_point[0])
    print(f'x = {x}')
    print(f'y = {y}')
    time = logs[0]
    fig, ax = plt.subplots()
    ax.xmin=-100
    ax.xmax=100
    ax.ymin=-100
    ax.ymax=100
    size_factor = 9
    dir_wind = sawtooth((logs[5] + logs[3]) * np.pi / 180)
    boat_shape = size_factor * np.array([[-0.25, 0, 0.25, 0.25, -0.25, -0.25],
                           [0., 1, 0., -2., -2., 0.]])
    sail_shape = size_factor * np.array([[0., 0.],
                           [0., -0.9]])
    rud_shape = size_factor * np.array([[0., 0.],
                          [-0., -0.3]])
    arrow_pos = (0.85, 0.85)
    for i in range(len(logs[0])):
        ax.clear()
        ax.set_xlim(ax.xmin,ax.xmax)
        ax.set_ylim(ax.ymin,ax.ymax)
        # print(f'i = {i}')
        R_boat = np.array([[np.cos(head[i]), -np.sin(head[i])],
                           [np.sin(head[i]), np.cos(head[i])]]) 
        R_rud_re = np.array([[np.cos(rud_angles[i]), -np.sin(rud_angles[i])],
                             [np.sin(rud_angles[i]), np.cos(rud_angles[i])]])
        R_sail_re = np.array([[np.cos(sail_angles[i]), -np.sin(sail_angles[i])],
                           [np.sin(sail_angles[i]), np.cos(sail_angles[i])]])
        true_wind = np.array([[SOG[i] * np.sin(head[i]) - logs[6][i] * np.sin(dir_wind[i])], 
                              [SOG[i] * np.cos(head[i]) - logs[6][i] * np.cos(dir_wind[i])]])
        true_wind_angle = sawtooth(np.arctan2(true_wind[0][0], true_wind[1][0]) - np.pi)
        print(f'true wind : {true_wind}')
        print(f'true wind angle : {true_wind_angle * 180 / np.pi}')
        true_wind = np.array([[0, -1], [1, 0]]) @ true_wind
        if np.linalg.norm(true_wind) != 0:
            arrow_tip_pos = (arrow_pos[0] + 0.15 * true_wind[0, 0] / np.linalg.norm(true_wind), arrow_pos[1] + 0.15 * true_wind[1, 0] / np.linalg.norm(true_wind))
            ax.annotate(
                '', 
                xy=arrow_tip_pos, xycoords='axes fraction',
                xytext=arrow_pos, textcoords='axes fraction',
                arrowprops=dict(facecolor='red', shrink=0.05)
                )
            ax.text(0.7, 0.8, f'True wind speed : {np.linalg.norm(true_wind)}',
                    transform=ax.transAxes,
                    fontsize=8)
        # print((R_boat @ boat_shape)[0] + x[i])
        ax.plot((R_boat @ boat_shape)[0] + x[i], (R_boat @ boat_shape)[1] + y[i], color = ('black' if logs[-1][i] == 0 else 'blue'))
        ax.plot((R_sail_re @ R_boat @ sail_shape)[0] + x[i], (R_sail_re @ R_boat @ sail_shape)[1] + y[i], color = 'green')
        ax.plot((R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[0] + x[i], (R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[1] + y[i], color = 'green')
        ax.plot(x[:i+1], y[:i+1], 'b')
        try:
            plt.pause((time[i+1] - time[i])/1000)
        except IndexError:
            pass
    return

def simulate_computing(logs : np.ndarray, ref_point : np.ndarray, listpoints : list) -> None:
    ref_point *= np.pi/180
    r = 10
    q = 1
    gamma = np.pi/4
    phi = np.pi/3
    angle_ruddermax = 50
    lat = logs[1] * np.pi / 180
    long = logs[2] * np.pi / 180
    head = logs[3] * np.pi / 180
    SOG = logs[4]
    rud_angles = conv_rud_com_to_pos(logs[4]) * np.pi / 180
    sail_angles = conv_sail_com_to_pos(logs[5]) * np.pi / 180
    R = 6371e3
    j = 0
    fig, ax = plt.subplots()
    ax.xmin=-100
    ax.xmax=100
    ax.ymin=-100
    ax.ymax=100
    size_factor = 6
    boat_shape = size_factor * np.array([[-0.25, 0, 0.25, 0.25, -0.25, -0.25],
                           [0., 1, 0., -2., -2., 0.]])
    sail_shape = size_factor * np.array([[0., 0.],
                           [0., -0.9]])
    rud_shape = size_factor * np.array([[0., 0.],
                          [-0., -0.3]])
    arrow_shape = size_factor / 2 * np.array([[0, 0, -0.25, 0, 0.25], 
                           [0, 2, 1.75, 2, 1.75]])
    rud_angles = conv_rud_com_to_pos(logs[7]) * np.pi / 180
    sail_angles = conv_sail_com_to_pos(logs[8]) * np.pi / 180
    arrow_pos = (0.85, 0.85)
    x = R * (long - ref_point[1]) * np.cos(ref_point[0])
    y = R * (lat - ref_point[0])
    dir_wind = (logs[5] + logs[3]) * np.pi / 180
    for i in range(len(logs[0])):
        ax.clear()
        ax.set_xlim(ax.xmin,ax.xmax)
        ax.set_ylim(ax.ymin,ax.ymax)
        start_point = listpoints[j] * np.pi/180
        end_point = listpoints[j+1] * np.pi/180
        xa = R * (start_point[1] - ref_point[1]) * np.cos(ref_point[0])
        ya = R * (start_point[0] - ref_point[0])
        A = np.array([[xa], 
                      [ya]])
        print(f'A = {A}')
        xb = R * (end_point[1] - ref_point[1]) * np.cos(ref_point[0])
        yb = R * (end_point[0] - ref_point[0])
        B = np.array([[xb], 
                      [yb]])
        print(f'B = {B}')
        AB = B - A
        C = AB / np.linalg.norm(AB)
        R_boat = np.array([[np.cos(head[i]), -np.sin(head[i])],
                           [np.sin(head[i]), np.cos(head[i])]]) 
        R_rud_re = np.array([[np.cos(rud_angles[i]), -np.sin(rud_angles[i])],
                             [np.sin(rud_angles[i]), np.cos(rud_angles[i])]])
        R_sail_re = np.array([[np.cos(sail_angles[i]), -np.sin(sail_angles[i])],
                           [np.sin(sail_angles[i]), np.cos(sail_angles[i])]])
        M = np.array([[x[i]], 
                      [y[i]]])
        print(f'Test time : {logs[0][i]}')
        print(f'M = {M}')
        print(f'Heading = {head[i] * 180 / np.pi}')
        D = M - A
        if (A - B)[0] * (M - B)[0] + (A - B)[1] * (M - B)[1] < 0 and j < len(listpoints) - 2:
            j += 1
        if j == len(listpoints) - 2:
            print('Fin de parcours.')
        e = C[0][0] * D[1][0] - D[0][0] * C[1][0]
        print(f'e = {e}')
        print(f'SOG = {SOG[i]}')
        true_wind = np.array([[SOG[i] * np.sin(head[i]) - logs[6][i] * np.sin(dir_wind[i])], 
                              [SOG[i] * np.cos(head[i]) - logs[6][i] * np.cos(dir_wind[i])]])
        true_wind_angle = sawtooth(np.arctan2(true_wind[0][0], true_wind[1][0]) - (np.pi))
        print(f'true wind : {true_wind}')
        print(f'true wind angle : {true_wind_angle * 180 / np.pi}')
        if abs(e) > r/2:
            q = np.sign(e)
        print(f'q = {q}')
        angle_line = sawtooth(np.arctan2(AB[1][0], AB[0][0]) - np.pi/2)
        print(f'Angle line = {angle_line * 180 / np.pi}')
        
        R_line  = np.array([[np.cos(angle_line), -np.sin(angle_line)],
                            [np.sin(angle_line), np.cos(angle_line)]])
        angle_nom = sawtooth(angle_line - 2 * gamma * np.arctan(e/r) / np.pi)
        R_nom = np.array([[np.cos(angle_nom), -np.sin(angle_nom)],
                          [np.sin(angle_nom), np.cos(angle_nom)]])
        print(f'Angle nominal = {angle_nom * 180 / np.pi}')
        if (np.cos(true_wind_angle - angle_nom) + np.cos(phi) < 0) or (abs(e) < r and np.cos(true_wind_angle - angle_line) + np.cos(phi) < 0):
            aimed_angle = sawtooth(np.pi + true_wind_angle - q * phi)
            print("TACKING ACTIVE")
        else:
            aimed_angle = angle_nom
        print(f'Aimed angle : {aimed_angle * 180 / np.pi}')
        R_aimed = np.array([[np.cos(aimed_angle), -np.sin(aimed_angle)],
                            [np.sin(aimed_angle), np.cos(aimed_angle)]])
        angle_rudder = angle_ruddermax*np.sin(head[i]-aimed_angle)
        if np.cos(head[i] - aimed_angle)<0:
            angle_rudder = angle_ruddermax * np.sign(np.sin(head[i] - aimed_angle))
        print(f'Angle rudder simulated: {angle_rudder}')
        print(f'Angle rudder real: {rud_angles[i] * 180 / np.pi}')
        
        R_rud_th = np.array([[np.cos(angle_rudder * np.pi / 180), -np.sin(angle_rudder * np.pi / 180)],
                             [np.sin(angle_rudder * np.pi / 180), np.cos(angle_rudder * np.pi / 180)]])
        
        angle_sail_th = np.pi / 2 * (np.cos(true_wind_angle - aimed_angle) + 1) / 2
        R_sail_th = np.array([[np.cos(angle_sail_th), -np.sin(angle_sail_th)],
                             [np.sin(angle_sail_th), np.cos(angle_sail_th)]])
        print(f'Angle sail simulated : {angle_sail_th * 180 / np.pi}')
        print(f'Angle sail real : {sail_angles[i] * 180  / np.pi}')
        arrow_tip_pos = (arrow_pos[0] + true_wind[0, 0], arrow_pos[1] + true_wind[1, 0])
        ax.plot((R_boat @ boat_shape)[0] + x[i], (R_boat @ boat_shape)[1] + y[i], color = ('black' if logs[-1][i] == 0 else 'blue'))
        ax.plot((R_sail_re @ R_boat @ sail_shape)[0] + x[i], (R_sail_re @ R_boat @ sail_shape)[1] + y[i], color = 'green')
        ax.plot((R_sail_th @ R_boat @ sail_shape)[0] + x[i], (R_sail_th @ R_boat @ sail_shape)[1] + y[i], color = 'red')
        ax.plot((R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[0] + x[i], (R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[1] + y[i], color = 'green')
        ax.plot((R_boat @ (R_rud_th @ rud_shape - size_factor * np.array([[0], [2]])))[0] + x[i], (R_boat @ (R_rud_th @ rud_shape - size_factor * np.array([[0], [2]])))[1] + y[i], color = 'red')
        ax.plot([xa, xb], [ya, yb])
        ax.plot(x[:i+1], y[:i+1], 'b')
        ax.plot((R_line @ arrow_shape + A)[0], (R_line @ arrow_shape + A)[1])
        ax.plot((R_nom @ arrow_shape)[0] + x[i], (R_nom @ arrow_shape)[1] + y[i], color = 'red')
        ax.plot((R_aimed @ arrow_shape)[0] + x[i], (R_aimed @ arrow_shape)[1] + y[i], color = 'green')
        true_wind = np.array([[0, -1], [1, 0]]) @ true_wind
        if np.linalg.norm(true_wind) != 0:
            arrow_tip_pos = (arrow_pos[0] + 0.15 * true_wind[0, 0] / np.linalg.norm(true_wind), arrow_pos[1] + 0.15 * true_wind[1, 0] / np.linalg.norm(true_wind))
            ax.annotate(
                '', 
                xy=arrow_tip_pos, xycoords='axes fraction',
                xytext=arrow_pos, textcoords='axes fraction',
                arrowprops=dict(facecolor='red', shrink=0.05)
                )
            ax.text(0.7, 0.8, f'True wind speed : {np.linalg.norm(true_wind)}',
                    transform=ax.transAxes,
                    fontsize=8)
        try:
            plt.pause((logs[0][i+1] - logs[0][i])/1000)
        except IndexError:
            pass
        
def display_scenario_logs(lognumber : int, ref_point : np.ndarray, listscenario : list) -> None:
    logs = read_log(lognumber)
    ref_point_rad = ref_point * np.pi/180
    r = 10
    q = 1
    gamma = np.pi/4
    phi = np.pi/3
    angle_ruddermax = 50
    lat = logs[1] * np.pi / 180
    long = logs[2] * np.pi / 180
    head = logs[3] * np.pi / 180
    SOG = logs[4]
    rud_angles = conv_rud_com_to_pos(logs[4]) * np.pi / 180
    sail_angles = conv_sail_com_to_pos(logs[5]) * np.pi / 180
    scenario_num = logs[-1]
    prev_scen_num = -1
    R = 6371e3
    j = 0
    fig, ax = plt.subplots()
    ax.xmin=-50
    ax.xmax=50
    ax.ymin=-50
    ax.ymax=50
    size_factor = 6
    boat_shape = size_factor * np.array([[-0.25, 0, 0.25, 0.25, -0.25, -0.25],
                           [0., 1, 0., -2., -2., 0.]])
    sail_shape = size_factor * np.array([[0., 0.],
                           [0., -0.9]])
    rud_shape = size_factor * np.array([[0., 0.],
                          [-0., -0.3]])
    arrow_shape = size_factor / 2 * np.array([[0, 0, -0.25, 0, 0.25], 
                           [0, 2, 1.75, 2, 1.75]])
    rud_angles = conv_rud_com_to_pos(logs[7]) * np.pi / 180
    sail_angles = conv_sail_com_to_pos(logs[8]) * np.pi / 180
    arrow_pos = (0.85, 0.85)
    x = R * (long - ref_point_rad[1]) * np.cos(ref_point_rad[0])
    y = R * (lat - ref_point_rad[0])
    dir_wind = (logs[5] + logs[3]) * np.pi / 180
    lenlogs = len(logs[0])
    listscenario_cart = []
    listpoints_cart = []
    for scenar in listscenario:
        for coord in scenar:
            listpoints_cart.append(GPS2cart(coord, ref_point))
        listscenario_cart.append(listpoints_cart)
        listpoints_cart = []
    # print(listscenario_cart)
    i = 0
    while i < lenlogs:
        ax.clear()
        ax.set_xlim(ax.xmin,ax.xmax)
        ax.set_ylim(ax.ymin,ax.ymax)
        scen_num = int(scenario_num[i])
        if prev_scen_num != scen_num:
            prev_scen_num = scen_num
            j = 0
            print(f'Scenario number : {scen_num}')
        R_boat = np.array([[np.cos(head[i]), -np.sin(head[i])],
                           [np.sin(head[i]), np.cos(head[i])]]) 
        R_rud_re = np.array([[np.cos(rud_angles[i]), -np.sin(rud_angles[i])],
                             [np.sin(rud_angles[i]), np.cos(rud_angles[i])]])
        R_sail_re = np.array([[np.cos(sail_angles[i]), -np.sin(sail_angles[i])],
                           [np.sin(sail_angles[i]), np.cos(sail_angles[i])]])
        M = np.array([[x[i]], 
                      [y[i]]])
        if scen_num != 9:
            A = listscenario_cart[scen_num][j]
            B = listscenario_cart[scen_num][j + 1]
            D = M - A
            if (A - B)[0] * (M - B)[0] + (A - B)[1] * (M - B)[1] < 0:
                print("Switching to next line")
                if j < len(listscenario_cart[scen_num]) - 2:
                    j += 1
                else: 
                    j = 0
            for n in range(len(listscenario_cart[scen_num]) - 1):
                # print(listscenario_cart[scen_num][n])
                ax.plot([listscenario_cart[scen_num][n][0, 0], listscenario_cart[scen_num][n+1][0, 0]], [listscenario_cart[scen_num][n][1, 0], listscenario_cart[scen_num][n+1][1, 0]], color = 'blue' if n!=j else 'red')
        true_wind = np.array([[SOG[i] * np.sin(head[i]) - logs[6][i] * np.sin(dir_wind[i])], 
                              [SOG[i] * np.cos(head[i]) - logs[6][i] * np.cos(dir_wind[i])]])
        true_wind_angle = sawtooth(np.arctan2(true_wind[0][0], true_wind[1][0]) - (np.pi))
        arrow_tip_pos = (arrow_pos[0] + true_wind[0, 0], arrow_pos[1] + true_wind[1, 0])
        ax.plot((R_boat @ boat_shape)[0] + x[i], (R_boat @ boat_shape)[1] + y[i], color = ('black' if logs[-1][i] == 0 else 'blue'))
        ax.plot((R_sail_re @ R_boat @ sail_shape)[0] + x[i], (R_sail_re @ R_boat @ sail_shape)[1] + y[i], color = 'green')
        ax.plot((R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[0] + x[i], (R_boat @ (R_rud_re @ rud_shape - size_factor * np.array([[0], [2]])))[1] + y[i], color = 'green')
        ax.plot(x[(max(0, i-1000)):i+1], y[(max(0, i-1000)):i+1], 'b')
        true_wind = np.array([[0, -1], [1, 0]]) @ true_wind
        if np.linalg.norm(true_wind) != 0:
            arrow_tip_pos = (arrow_pos[0] + 0.15 * true_wind[0, 0] / np.linalg.norm(true_wind), arrow_pos[1] + 0.15 * true_wind[1, 0] / np.linalg.norm(true_wind))
            ax.annotate(
                '', 
                xy=arrow_tip_pos, xycoords='axes fraction',
                xytext=arrow_pos, textcoords='axes fraction',
                arrowprops=dict(facecolor='red', shrink=0.05)
                )
            ax.text(0.7, 0.8, f'True wind speed : {np.linalg.norm(true_wind)}',
                    transform=ax.transAxes,
                    fontsize=8)
        try:
            plt.pause((logs[0][i+1] - logs[0][i])/10000)
        except IndexError:
            pass
        i += 1
        
if __name__ == '__main__' :
    # ref_point = np.array([52.4844041, -1.8898449])
    # Point1 = np.array([52.485958, -1.889726])
    # Point2 = np.array([52.4860084, -1.8889055])
    # Point1 = np.array([52.4844663, -1.8895039])
    # Point2 = np.array([52.4843069, -1.8905943])
    # Point3 = np.array([52.4845141, -1.8905922])
    # Point4 = np.array([52.4847932, -1.8899488])
    # Point5 = np.array([52.4846881, -1.8896900])
    Point10 = np.array([52.429390, -1.946684])
    Point20 = np.array([52.429450, -1.946031])
    scenario0 = [Point10, Point20, Point10]
    Point11 = np.array([52.429246, -1.946501])
    Point21 = np.array([52.429420, -1.946751])
    Point31 = np.array([52.429440, -1.946201])
    scenario1 = [Point11, Point21, Point31, Point11]
    Point12 = np.array([52.429254, -1.946549])
    Point22 = np.array([52.429447, -1.946549])
    scenario2 = [Point12, Point22, Point12]
    Point13 = np.array([52.429250, -1.946510])
    Point23 = np.array([52.429438, -1.946770])
    Point33 = np.array([52.429502, -1.945649])
    Point43 = np.array([52.429433, -1.946102])
    scenario3 = [Point13, Point23, Point33, Point43, Point13]
    listscenario = [scenario0, scenario1, scenario2, scenario3, scenario0, scenario1, scenario3]
    water_refpoint = np.array([52.429363, -1.946551])
    toKML(51)
    display_scenario_logs(52, water_refpoint, listscenario)
    # generate_animation(read_log(51), water_refpoint)
    # simulate_computing(read_log(50), ref_point, [Point1, Point2, Point3, Point4, Point5, Point1])