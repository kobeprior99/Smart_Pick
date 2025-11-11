'''
Author: Kobe Prior
Date: NOV 3, 2025
Purpose: Graph raw capacitance vs time from esp32 serial output.
'''
#import required libraries
import serial, serial.tools.list_ports, time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from config import BAUD, MAX_POINTS 

def select_ser_port():
    '''
    offer the user choices for serial ports and return the one they select
    '''
    #offer the user to select seral port in terminal
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("no serial ports found")
        return None
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f'[{i}] {port.device} - {port.description}')
    while True:
        try: 
            index = int(input("Select port number: "))
            if index >= 0 and index < len(ports):
                return ports[index].device
            else:
                print('Invalid choice, try again.')
        except:
            print("Please enter a valid number.")

def main():
    '''
    Graph capacitance vs time
    '''
    #start serial
    port = select_ser_port()    
    if port is None:
        exit()

    ser = serial.Serial(port, BAUD, timeout=1)
    time.sleep(.5) #wait for esp to reset

    print(f'Connected to {port} at {BAUD} baud.')
    #plot
    #capcitance data
    times_c = deque(maxlen=MAX_POINTS)
    caps = deque(maxlen=MAX_POINTS)
    #acceleration data
    times_a =deque(maxlen=MAX_POINTS)
    accelerations = deque(maxlen=MAX_POINTS)
    # Create 2 vertically stacked subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    fig.suptitle("Live Data from ESP32")

    # --- Capacitance plot (top) ---
    line_c, = ax1.plot([], [], lw=2, label='Capacitance', color='tab:blue')
    ax1.set_ylabel('Capacitance (raw units)')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylim(10777215,13777215)  # 2^24 - 1 full-scale
    ax1.legend()
    ax1.grid(True)

    # --- Acceleration plot (bottom) ---
    line_a, = ax2.plot([], [], lw=2, label='Acceleration (x-axis)', color='tab:orange')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Acceleration')
    ax2.legend()
    ax2.grid(True)
    

    def update(frame):
        '''
        Update plot live
        '''
        line_in = ser.readline().decode('utf-8').strip()
        if not line_in:
            return line_c,line_a 
        #Expect format TIME,<t>,CAP,<c>
        #also expect TIME <t> ACCEL <dx> 
        parts = line_in.split(',')
        #debug print
        # print(parts)
        try:
            #FORMAT: TIME,<t>,CAP,<c> 
            if len(parts) == 4 and parts[0] == "TIME" and parts[2] == "CAP":
                t = float(parts[1])
                c_raw = float(parts[3])
                caps.append(c_raw)
                times_c.append(t)
                line_c.set_data(times_c,caps)
                ax1.relim()
                ax1.autoscale_view(scaley=False)
            #FORMAT: TIME,<t>,ACCEL,<c> 
            elif len(parts) ==4 and parts[0] == 'TIME' and parts[2] == 'ACCEL':
                t = float(parts[1])
                a_x = float(parts[3])
                accelerations.append(a_x)
                times_a.append(t)
                line_a.set_data(times_a,accelerations)
                ax2.relim()
                ax2.autoscale_view(scaley=True)
        except ValueError as e:
            print(f"Parse error {e}") 

        return line_c,line_a 

    ani = FuncAnimation(fig, update, interval= 10, cache_frame_data=False)
    plt.show()
    ser.close()
    print('Serial port closed')
    
if __name__ == '__main__':
    main()
