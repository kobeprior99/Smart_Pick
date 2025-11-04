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
from config import BAUD 

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
    MAX_POINTS = 300 # number of samples visible
    times = deque(maxlen=MAX_POINTS)
    caps = deque(maxlen=MAX_POINTS)
    fig, ax = plt.subplots()
    line, = ax.plot([],[], lw=2, label = 'Capacitance')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Capacitance (raw units)')
    ax.legend()
    ax.set_title("Live Capacitance vs Time")

    def update(frame):
        '''
        Update plot live
        '''
        line_in = ser.readline().decode('utf-8').strip()
        if not line_in:
            return line, 
        #Expect format TIME,<t>,CAP,<c>
        parts = line_in.split(',')
        #debug print
        # print(parts)
        try:
            if len(parts) == 4 and parts[0] == "TIME" and parts[2] == "CAP":
                t = float(parts[1])
                c_raw = float(parts[3])
                caps.append(c_raw)
                times.append(t)
                line.set_data(times,caps)
                ax.relim()
                ax.autoscale_view()
        except ValueError as e:
            print(f"Parse error {e}") 

        return line, 

    ani = FuncAnimation(fig, update, interval= 10, cache_frame_data=False)
    plt.show()
    ser.close()
    print('Serial port closed')
    
if __name__ == '__main__':
    main()
