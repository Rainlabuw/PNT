import serial
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from collections import deque
# Serial port configuration
serial_port = 'COM4'  # Replace with your serial port
baud_rate = 115200      # Replace with your baud rate

# Global variable to store the latest distances
latest_distances = None
buffer_size = 20  # Size of the buffer for running averages

dist_a_buffer = deque(maxlen=buffer_size)
dist_b_buffer = deque(maxlen=buffer_size)
dist_c_buffer = deque(maxlen=buffer_size)


def trilateration(pos_a, pos_b, pos_c, dist_a, dist_b, dist_c):
    pos1 = np.array(pos_a)
    pos2 = np.array(pos_b)
    pos3 = np.array(pos_c)
    A = (-2*pos1[0]+2*pos2[0])
    B = (-2*pos1[1]+2*pos2[1])
    C = dist_a**2-dist_b**2-pos1[0]**2+pos2[0]**2-pos1[1]**2+pos2[1]**2
    D = (-2*pos2[0]+2*pos3[0])
    E = (-2*pos2[1]+2*pos3[1])
    F = dist_b**2-dist_c**2-pos2[0]**2+pos3[0]**2-pos2[1]**2+pos3[1]**2
    
    x = (C*E-F*B)/(E*A-B*D)
    y = (C*D-A*F)/(B*D-A*E)
    print(x,y)
    return np.array([x,y,0])


def read_serial_data():
    global latest_distances
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    while True:
        line = ser.readline().decode('utf-8').strip()
        distances = re.findall(r"TAG ID : \d, distance : ([\d.]+)", line)
        print(distances)
        
        if len(distances) == 3:
            if(float(distances[0])<=0):
                distances[0] = 0
            if(float(distances[1])<=0):
                distances[1] =0
            if(float(distances[2])<=0):
                distances[2] =0
            dist_a_buffer.append(float(distances[0]))
            dist_b_buffer.append(float(distances[1]))
            dist_c_buffer.append(float(distances[2]))
            if len(dist_a_buffer) == buffer_size:
                avg_a = sum(dist_a_buffer) / len(dist_a_buffer)
                avg_b = sum(dist_b_buffer) / len(dist_b_buffer)
                avg_c = sum(dist_c_buffer) / len(dist_c_buffer)
                latest_distances = (avg_a, avg_b, avg_c)
    ser.close()


pos_a = (0, 0)
pos_b = (0.8128, 0)
pos_c = (0.3429, 0.4064)
# Set up the figure for plotting
fig, ax = plt.subplots()
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
sc = ax.scatter([], [], color='red')  # Initial empty scatter plot
ax.scatter(*pos_a, color='blue', label='Arduino A')
ax.scatter(*pos_b, color='green', label='Arduino B')
ax.scatter(*pos_c, color='orange', label='Arduino C')
ax.legend()
# Animation update function
def animate(i):
    global latest_distances
    if latest_distances:
        position = trilateration(pos_a, pos_b, pos_c, *latest_distances)
        sc.set_offsets([position])  # Update the position of the scatter plot
    return sc,
# Create and start the animation
ani = animation.FuncAnimation(fig, animate, interval=100)

# Start serial reading in a separate thread
thread = threading.Thread(target=read_serial_data)
thread.daemon = True
thread.start()

plt.show()
