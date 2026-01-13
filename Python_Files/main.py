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
        
    def get_available_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def read_serial_data(self):
        """Background thread to read serial data"""
        self.serial_conn.reset_input_buffer()
        
        while self.is_running:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    # Parse CSV: time_us, capacitance, acceleration
                    parts = line.split(',')
                    if len(parts) == 3:
                        try:
                            t_us = float(parts[0]) / 1000000.0  # Convert to seconds
                            cap_raw = float(parts[1])
                            accel_z = float(parts[2])
                            
                            # Append to buffers
                            self.time_data.append(t_us)
                            self.cap_data.append(cap_raw)
                            self.accel_data.append(accel_z)
                            
                        except ValueError:
                            pass  # Skip malformed lines
                            
            except Exception as e:
                print(f"Serial read error: {e}")
                break
                
            time.sleep(0.001)  # Small delay to prevent CPU spinning
    
    def update_plots(self):
        """Update both plots with latest data"""
        if len(self.time_data) == 0:
            return
            
        # Create time axis relative to start
        time_rel = [t - self.time_data[0] for t in self.time_data]
        
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
            margin=dict(l=60, r=20, t=40, b=40)
        )
        self.cap_plot.update_figure(cap_fig)
        
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
            margin=dict(l=60, r=20, t=40, b=40)
        )
        self.accel_plot.update_figure(accel_fig)
    
    def start_acquisition(self):
        """Start serial communication and data acquisition"""
        port = self.port_select.value
        
        if not port:
            ui.notify('Please select a serial port', type='warning')
            return
            
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            
            # Clear old data
            self.time_data.clear()
            self.cap_data.clear()
            self.accel_data.clear()
            
            self.is_running = True
            self.thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.thread.start()
            
            # Update UI
            self.start_btn.disable()
            self.stop_btn.enable()
            self.port_select.disable()
            self.status_label.text = f'Status: Running on {port}'
            self.status_label.style('color: green')
            
            # Start plot update timer
            self.plot_timer.activate()
            
            ui.notify('Acquisition started', type='positive')
            
        except Exception as e:
            ui.notify(f'Failed to open port: {str(e)}', type='negative')
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
