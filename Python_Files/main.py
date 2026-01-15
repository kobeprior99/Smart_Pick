"""
filename: main.py

Description: Graphical User interface load cell live plotter
Author: Kobe Prior
Date: Jan 13 2026
"""

from nicegui import ui
import serial
import serial.tools.list_ports
import plotly.graph_objects as go
from collections import deque
import threading
import time

class LoadCellPlotter:
    def __init__(self):
        self.serial_conn = None
        self.is_running = False
        self.thread = None
        
        # Data buffers (store last 2000 points)
        self.max_points = 2000
        self.time_data = deque(maxlen=self.max_points)
        self.cap_data = deque(maxlen=self.max_points)
        self.accel_data = deque(maxlen=self.max_points)
        
        # UI elements
        self.port_select = None
        self.start_btn = None
        self.stop_btn = None
        self.status_label = None
        self.cap_plot = None
        self.accel_plot = None
        
        # Data counter for UI
        self.data_count = 0
        
    def get_available_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def read_serial_data(self):
        """Background thread to read serial data"""
        import struct
        
        self.serial_conn.reset_input_buffer()
        
        # Binary packet: uint32 timestamp + int32 capacitance + float32 accel_z = 12 bytes
        packet_size = 12
        
        print("Serial read thread started, waiting for data...")
        
        # Try to detect what format we're receiving
        format_detected = False
        is_binary = False
        
        while self.is_running:
            try:
                waiting = self.serial_conn.in_waiting
                
                if waiting > 0 and not format_detected:
                    # Peek at first byte to detect format
                    first_byte = self.serial_conn.read(1)
                    if first_byte.isdigit():
                        print("Detected CSV format")
                        is_binary = False
                        # Put byte back by reading rest of line
                        line = first_byte + self.serial_conn.readline()
                        print(f"First line: {line}")
                    else:
                        print("Detected binary format")
                        is_binary = True
                        # Put byte back into buffer conceptually
                        self.serial_conn.reset_input_buffer()
                    format_detected = True
                
                if is_binary and waiting >= packet_size:
                    if self.data_count == 0:
                        print(f"First binary packet! {waiting} bytes available")
                    
                    data = self.serial_conn.read(packet_size)
                    
                    try:
                        # Unpack binary data
                        t_us, cap_raw, accel_z = struct.unpack('<Iif', data)
                        
                        # Sanity check
                        if t_us > 0 and t_us < 4294967295 and abs(accel_z) < 100:
                            t_seconds = t_us / 1000000.0
                            
                            self.time_data.append(t_seconds)
                            self.cap_data.append(cap_raw)
                            self.accel_data.append(accel_z)
                            self.data_count += 1
                            
                            if self.data_count % 500 == 0:
                                print(f"Received {self.data_count} samples - t={t_seconds:.2f}s, cap={cap_raw}, accel={accel_z:.3f}")
                        else:
                            print(f"Invalid data: t_us={t_us}, accel={accel_z}")
                    except struct.error as e:
                        print(f"Unpack error: {e}, data: {data.hex()}")
                        
                elif not is_binary and waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line and ',' in line:
                        if self.data_count == 0:
                            print(f"First CSV line: {line}")
                        
                        parts = line.split(',')
                        if len(parts) == 3:
                            try:
                                t_us = int(parts[0]) / 1000000.0
                                cap_raw = int(parts[1])
                                accel_z = float(parts[2])
                                
                                self.time_data.append(t_us)
                                self.cap_data.append(cap_raw)
                                self.accel_data.append(accel_z)
                                self.data_count += 1
                                
                                if self.data_count % 500 == 0:
                                    print(f"Received {self.data_count} samples")
                            except (ValueError, IndexError):
                                pass
                            
            except Exception as e:
                print(f"Serial read error: {e}")
                import traceback
                traceback.print_exc()
                break
                
            time.sleep(0.0001)
        
        print("Serial read thread stopped")
    
    def update_plots(self):
        """Update both plots with latest data"""
        if len(self.time_data) == 0:
            return
        
        # Update status with sample count
        if self.is_running:
            self.status_label.text = f'Status: Running ({self.data_count} samples)'
        

        # Create time axis relative to start (keep in seconds)
        start_time = self.time_data[0]
        time_rel = [(t - start_time) for t in self.time_data]
        
        # Update capacitance plot
        cap_fig = go.Figure()
        cap_fig.add_trace(go.Scatter(
            x=time_rel,
            y=list(self.cap_data),
            mode='lines',
            name='Capacitance',
            line=dict(color='#2196F3', width=2)
        ))
        cap_fig.update_layout(
            title='Capacitance (Raw)',
            xaxis_title='Time (s)',
            yaxis_title='Capacitance (Raw ADC)',
            template='plotly_white',
            height=300,
            margin=dict(l=60, r=20, t=40, b=40),
            showlegend=False,
            xaxis=dict(fixedrange=False),
            yaxis=dict(fixedrange=False)
        )
        self.cap_plot.figure = cap_fig
        self.cap_plot.update()
        
        # Update acceleration plot
        accel_fig = go.Figure()
        accel_fig.add_trace(go.Scatter(
            x=time_rel,
            y=list(self.accel_data),
            mode='lines',
            name='Acceleration Z',
            line=dict(color='#FF5722', width=2)
        ))
        accel_fig.update_layout(
            title='Acceleration Z-axis (m/s²)',
            xaxis_title='Time (s)',
            yaxis_title='Acceleration (m/s²)',
            template='plotly_white',
            height=300,
            margin=dict(l=60, r=20, t=40, b=40),
            showlegend=False,
            xaxis=dict(fixedrange=False),
            yaxis=dict(fixedrange=False)
        )
        self.accel_plot.figure = accel_fig
        self.accel_plot.update()
    
    def start_acquisition(self):
        """Start serial communication and data acquisition with debug info"""
        ports = self.get_available_ports()
        print("Available serial ports:", ports)

        port = self.port_select.value
        print("Selected port:", port)

        if not port:
            ui.notify('Please select a serial port', type='warning')
            return

        try:
            # Try to open the serial port
            self.serial_conn = serial.Serial(port, 115200, dsrdtr=None)
            self.serial_conn.setRTS(False)
            self.serial_conn.setDTR(False)
            time.sleep(3)  # sometimes 5s is safer for ESP32

            
            print(f"Opened serial port: {self.serial_conn.name}, is_open={self.serial_conn.is_open}")

            if not self.serial_conn.is_open:
                print("Error: Serial port did not open successfully!")
                ui.notify("Failed to open serial port (not open)", type='negative')
                return


            # Clear old data
            self.time_data.clear()
            self.cap_data.clear()
            self.accel_data.clear()
            self.data_count = 0

            self.is_running = True
            self.thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.thread.start()

            # Update UI
            self.start_btn.disable()
            self.stop_btn.enable()
            self.port_select.disable()
            self.status_label.text = f'Status: Running ({self.data_count} samples)'
            self.status_label.style('color: green')

            # Start plot update timer
            self.plot_timer.activate()

            ui.notify('Acquisition started', type='positive')

        except serial.SerialException as e:
            print("SerialException:", e)
            ui.notify(f'Failed to open port: {str(e)}', type='negative')
            self.is_running = False

        except Exception as e:
            print("Unexpected error opening serial port:", e)
            ui.notify(f'Unexpected error: {str(e)}', type='negative')
            self.is_running = False
    
    def stop_acquisition(self):
        """Stop serial communication"""
        self.is_running = False
        
        if self.thread:
            self.thread.join(timeout=2)
            
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            
        # Update UI
        self.start_btn.enable()
        self.stop_btn.disable()
        self.port_select.enable()
        self.status_label.text = 'Status: Stopped'
        self.status_label.style('color: red')
        
        # Stop plot updates
        self.plot_timer.deactivate()
        
        ui.notify('Acquisition stopped', type='info')
    
    def build_ui(self):
        """Build the NiceGUI interface"""
        ui.page_title('Load Cell Live Plotter')
        
        with ui.header().classes('items-center justify-between'):
            ui.label('Load Cell Data Acquisition').classes('text-h4')
        
        with ui.card().classes('w-full'):
            ui.label('Serial Port Configuration').classes('text-h6')
            
            with ui.row().classes('w-full items-center gap-4'):
                ports = self.get_available_ports()
                self.port_select = ui.select(
                    ports,
                    label='Serial Port',
                    value=ports[0] if ports else None
                ).classes('w-64')
                
                ui.button(
                    'Refresh Ports',
                    on_click=lambda: self.port_select.set_options(self.get_available_ports())
                ).props('flat')
                
                self.status_label = ui.label('Status: Idle').classes('text-lg')
        
        with ui.card().classes('w-full'):
            with ui.row().classes('gap-4'):
                self.start_btn = ui.button(
                    'Start Acquisition',
                    on_click=self.start_acquisition,
                    color='positive'
                ).props('size=lg')
                
                self.stop_btn = ui.button(
                    'Stop Acquisition',
                    on_click=self.stop_acquisition,
                    color='negative'
                ).props('size=lg')
                self.stop_btn.disable()
        
        # Capacitance plot
        with ui.card().classes('w-full'):
            self.cap_plot = ui.plotly({}).classes('w-full')
        
        # Acceleration plot
        with ui.card().classes('w-full'):
            self.accel_plot = ui.plotly({}).classes('w-full')
        
        # Create plot update timer (updates every 100ms)
        self.plot_timer = ui.timer(0.1, self.update_plots, active=False)

# Create and run the application
plotter = LoadCellPlotter()
plotter.build_ui()

ui.run(title='Load Cell Plotter', port=8080, reload=False)
