"""
filename: main.py

Description: Graphical User interface load cell live plotter
Author: Kobe Prior
Date: Jan 13 2026
"""

from nicegui import ui, app
import serial
import serial.tools.list_ports
import asyncio
from config import BAUDRATE

ser = None
erase_dialog = None
read_dialog = None
filename_input = None

SELECTED_COM_PORT = 'SELECT SMART_PICK PORT'

async def set_com_port(port: str):
    global SELECTED_COM_PORT, ser
    SELECTED_COM_PORT = port
    try:
        ser = serial.Serial(SELECTED_COM_PORT, BAUDRATE)
        ser.dtr = True
        ser.rts = True
        await asyncio.sleep(3)
    except Exception as e:
        print(f'Failed to open serial port: {e}')


def serial_ready():
    return ser is not None and ser.is_open


def handle_stop():
    if not serial_ready():
        ui.notify('Serial not connected', type='negative')
        return
    try:
        ser.write(b'E')
        ui.notify('Stop command sent', type='negative')
    except Exception as e:
        ui.notify(f'Serial error: {e}', type='negative')


def handle_read():
    if SELECTED_COM_PORT == "SELECT SMART_PICK PORT":
        ui.notify('Must select port first', type='negative')
    else:
        read_dialog.open()


def download_csv():
    if ser is None or not ser.is_open:
        ui.notify("Serial not connected", type="negative")
        return
    filename = filename_input.value
    if filename == "":
        ui.notify("Enter a filename", type="negative")
        return
    path = f"{filename}.csv"
    ui.notify("Reading data from device...", type="info")
    ser.reset_input_buffer()
    ser.write(b'R')
    with open(path, "w") as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line == "EOF":
                break
            f.write(line + "\n")
    ui.notify(f"Saved CSV: {path}", type="positive")
    read_dialog.close()


def handle_erase():
    if SELECTED_COM_PORT == "SELECT SMART_PICK PORT":
        ui.notify('Must select port first', type='negative')
    else:
        erase_dialog.open()


async def confirm_erase():
    if not serial_ready():
        ui.notify('Serial not connected', type='negative')
        return
    try:
        ser.write(b'D')
        ui.notify('Erase started — this takes approximately 2 minutes...', type='warning')
        erase_dialog.close()
        
        # Read responses in background without blocking UI
        while True:
            await asyncio.sleep(0.1)  # yield to UI between reads
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if 'Erasing' in line:
                    ui.notify(line, type='warning')
                elif 'KB' in line:
                    ui.notify(line, type='info')  # progress updates
                elif 'Flash erased' in line:
                    ui.notify('Flash erased successfully', type='positive')
                    break
                elif 'ERROR' in line:
                    ui.notify(f'Error: {line}', type='negative')
                    break

    except Exception as e:
        ui.notify(f'Serial error: {e}', type='negative')


# ── Simulate page ─────────────────────────────────────────────────────────────
@ui.page('/simulate')
def simulate_page():
    ui.add_head_html('''
    <style>
        body, html { margin: 0; padding: 0; height: 100%; }
        .nicegui-content { height: 100vh; display: flex; flex-direction: column; padding: 0 !important; }
    </style>
    ''')

    # Back button header
    with ui.row().style(
        'width: 100%; padding: 16px 24px; box-sizing: border-box; '
        'align-items: center; border-bottom: 1px solid #e0e0e0;'
    ):
        ui.button(icon='arrow_back', on_click=lambda: ui.navigate.to('/')) \
            .props('flat round')
        ui.label('Simulate Miner Experience').style(
            'font-size: 1.3rem; font-weight: 600; margin-left: 12px;'
        )

    # Content area — placeholder for user to fill in
    with ui.column().style(
        'flex: 1; width: 100%; height: calc(100vh - 73px); '
        'display: flex; flex-direction: column; '
        'align-items: center; justify-content: center; '
        'gap: 16px; box-sizing: border-box;'
    ):
        ui.icon('construction').style('font-size: 4rem; color: #aaa;')
        ui.label('TODO: IMPLEMENTATION PENDING').style(
            'font-size: 1.4rem; color: #aaa; font-weight: 500;'
        )


# ── Main page ──────────────────────────────────────────────────────────────────
@ui.page('/')
def main_page():
    global erase_dialog, read_dialog, filename_input

    with ui.dialog() as erase_dialog, ui.card():
        ui.label('Are you sure you want to erase flash memory?')
        with ui.row():
            ui.button('Cancel', on_click=erase_dialog.close)
            ui.button('Erase', on_click=confirm_erase)

    with ui.dialog() as read_dialog, ui.card():
        ui.label("Enter filename for CSV")
        filename_input = ui.input(label="Filename")
        with ui.row():
            ui.button("Cancel", on_click=read_dialog.close)
            ui.button('Download', on_click=download_csv)

    ui.add_head_html('''
    <style>
        body, html { margin: 0; padding: 0; height: 100%; }
        .nicegui-content { height: 100vh; display: flex; flex-direction: column; padding: 0 !important; }

        .hover-lift {
            border-radius: 16px !important;
            transition: box-shadow 0.2s ease, transform 0.15s ease !important;
        }
        .hover-lift:hover {
            box-shadow: 0 12px 40px rgba(0, 0, 0, 0.35) !important;
            transform: translateY(-3px);
        }
        .hover-lift:active {
            box-shadow: 0 4px 16px rgba(0, 0, 0, 0.25) !important;
            transform: translateY(0px);
        }

        .big-btn {
            width: 100%;
            flex: 1;
            font-size: 2rem !important;
            font-weight: 700 !important;
            letter-spacing: 0.05em;
        }
        .big-btn .q-icon {
            font-size: 2.2rem !important;
            margin-right: 16px;
        }

        .sim-btn {
            width: 100%;
            height: 100%;
            display: flex !important;
            flex-direction: column !important;
            align-items: center !important;
            justify-content: flex-end !important;
            padding: 0 0 24px 0 !important;
            font-size: 1.6rem !important;
            font-weight: 700 !important;
            letter-spacing: 0.04em;
            background-color: #2c2c2c !important;
            color: #ffffff !important;
            border-radius: 16px !important;
            overflow: hidden;
            position: relative;
        }
    </style>
    ''')

    with ui.row().style(
        'width: 100%; padding: 16px 24px; box-sizing: border-box; '
        'align-items: center; border-bottom: 1px solid #e0e0e0;'
    ):
        ports = serial.tools.list_ports.comports()
        portsList = {p.device: f'{p.device} - {p.description}' for p in ports}
        ui.select(
            options=portsList,
            label=SELECTED_COM_PORT,
            on_change=lambda e: asyncio.create_task(set_com_port(e.value))
        ).style('width: 300px')
        ui.label('Smart Pick Dashboard').style(
                'font-size: 1.3rem; font-weight: 600; margin-left: 12px;'
            )
    with ui.row().style(
        'flex: 1; width: 100%; height: 80vh; '
        'display: flex; flex-direction: row; '
        'align-items: stretch; '
        'gap: 20px; padding: 24px; box-sizing: border-box;'
    ):
        # Left: three stacked control buttons
        with ui.column().style(
            'flex: 1; display: flex; flex-direction: column; '
            'align-items: stretch; gap: 20px;'
        ):
            ui.button('Stop Recording', icon='stop_circle', on_click=handle_stop, color='red') \
                .classes('big-btn hover-lift')
            ui.button('Read Data', icon='download', on_click=handle_read, color='blue') \
                .classes('big-btn hover-lift')
            ui.button('Erase Flash', icon='delete_forever', on_click=handle_erase, color='orange') \
                .classes('big-btn hover-lift')

        # Right: simulate miner experience — navigates to /simulate
        with ui.element('div').style(
            'flex: 1; display: flex; flex-direction: column; align-items: stretch;'
        ):
            ui.button(
                'Simulate Miner Experience',
                icon='precision_manufacturing',
                on_click=lambda: ui.navigate.to('/simulate')
            ).classes('big-btn hover-lift').style(
                'height: 100%; background-color: #757575 !important; '
                'color: #ffffff !important;'
            )

ui.run(title="Smart Pick Control Dashboard", reload=False)
