'''
Author: Kobe Prior
Date: NOV 11, 2025
Purpose: Graph raw capacitance vs time from esp32 serial output.
'''
'''
Author: Kobe Prior
Date: NOV 11, 2025
Purpose: Graph raw capacitance vs time from ESP32 serial output.
'''

import serial, serial.tools.list_ports, time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from config import BAUD, MAX_POINTS

def select_ser_port():
    """Offer the user choices for serial ports and return the one they select."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return None

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")

    while True:
        try:
            index = int(input("Select port number: "))
            if 0 <= index < len(ports):
                return ports[index].device
            print("Invalid choice, try again.")
        except ValueError:
            print("Please enter a valid number.")

def main():
    """Graph capacitance vs time."""
    port = select_ser_port()
    if port is None:
        exit()

    # Faster connection setup and buffer read
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.3)
    print(f"Connected to {port} at {BAUD} baud.")

    # Data buffers
    times = deque(maxlen=MAX_POINTS)
    caps = deque(maxlen=MAX_POINTS)

    # Plot setup
    fig, ax = plt.subplots(figsize=(8, 5))
    line_c, = ax.plot([], [], lw=1.8, color='tab:blue')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Capacitance (raw units)')
    ax.set_ylim(10777215, 13777215)
    ax.grid(True)

    def update(_):
        """Update plot live."""
        # Drain the serial buffer quickly
        while ser.in_waiting:
            line_in = ser.readline().decode(errors='ignore').strip()

        if not line_in:
            return line_c,

        try:
            t, c = map(float, line_in.split(','))
        except ValueError:
            return line_c,

        # Update data
        times.append(t)
        caps.append(c)
        line_c.set_data(times, caps)

        # Fixed-width moving window (e.g. 5 s)
        ax.set_xlim(max(0, t - 5000), t)

        return line_c,

    ani = FuncAnimation(fig, update, interval=100, blit=True, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    ser.close()
    print("Serial port closed.")

if __name__ == "__main__":
    main()
