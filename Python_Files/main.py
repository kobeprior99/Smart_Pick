"""
filename: main.py

Description: Graphical User interface load cell live plotter
Author: Kobe Prior
Date: Jan 13 2026
"""

from nicegui import ui
import serial
import serial.tools.list_ports
import asyncio
from config import BAUDRATE

# to keep track of com port 
ser = None
#dialogue for erase
erase_dialog = None

read_dialog = None

filename_input = None
SELECTED_COM_PORT = 'SELECT SMART_PICK PORT' #global variable to store com selection
async def set_com_port(port:str):
    global SELECTED_COM_PORT,ser
    SELECTED_COM_PORT = port
    #debug
    #print(f'COM port set to {SELECTED_COM_PORT}')
    try:
        ser = serial.Serial(SELECTED_COM_PORT,BAUDRATE)
        ser.dtr = True
        ser.rts =True
        await asyncio.sleep(3)#allow arduino to reset
    except Exception as e:
        print(f'Failed to open serial port: {e}')


def handle_stop():
    if SELECTED_COM_PORT == "SELECT SMART_PICK PORT":
        ui.notify('Must select port first', type='negative')
        return

    ser.write(b'E')
    ui.notify('Stop command sent', type='negative')
        

def handle_read():
    if SELECTED_COM_PORT == "SELECT SMART_PICK PORT":
        ui.notify('Must select port first', type='negative')
    else:
        read_dialog.open()

async def download_csv():
    filename = filename_input.value

    if filename == "":
        ui.notify("Enter a file name", type="negative")
        return

    path = f"data/{filename}.csv"

    ui.notify("Reading data from device...", type="info")

    ser.write(b'R')

    await asyncio.sleep(0.5)

    with open(path, "w") as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()

            if line == "EOF":
                break

            f.write(line + "\n")

    ui.notify(f"Saved CSV: {path}", type="positive")
    read_dialog.close()

def handle_erase():
    if (SELECTED_COM_PORT == "SELECT SMART_PICK PORT"):
        ui.notify('Must select port first', type= 'negative')
    else:
        erase_dialog.open()
def confirm_erase():
    ser.write(b'D')
    ui.notify("erase command sent", type = 'warning');
    erase_dialog.close()

# main page
@ui.page('/') 
def main_page():
    '''
    Main Page: where the user starts
    User can select the SMART_PICK port at the very start
    ''' 
    #connect to arduino part one time at the main screen:
    global erase_dialog
    global read_dialog
    global filename_input
    #dialogue box
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
            ui.button("Download", on_click=lambda: asyncio.create_task(download_csv()))



    ports = serial.tools.list_ports.comports()
    portsList = {p.device:f'{p.device} - {p.description}'for p in ports} 
    # COM port
    com_input = ui.select(
        options = portsList, 
        label = SELECTED_COM_PORT,
        on_change=lambda e: asyncio.create_task(set_com_port(e.value))
    ).style('width: 300px')


    ui.separator()

    # control buttons
    with ui.row():
        ui.button('Stop Recording', on_click=handle_stop, color='red')
        ui.button('Read Data', on_click=handle_read, color='blue')
        ui.button('Erase Flash', on_click=handle_erase, color='orange')
#run app
ui.run(title="Smart Pick Control Dashboard", reload=False)
