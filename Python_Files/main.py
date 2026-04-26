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
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from config import BAUDRATE
import io
import numpy as np

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
        .upload-zone {
            border: 2px dashed #cbd5e1;
            border-radius: 12px;
            padding: 32px;
            text-align: center;
            transition: border-color 0.2s, background 0.2s;
            cursor: pointer;
            background: #f8fafc;
        }
        .upload-zone:hover {
            border-color: #3b82f6;
            background: #eff6ff;
        }
    </style>
    ''')
 
    # ── state ──────────────────────────────────────────────────────────────────
    state = {
        'df': None,           # full dataframe
        'window_start': 0,    # current window start index
        'window_size': 400,   # number of samples in view (~500ms at 800Hz)
        'playing': False,
        'play_task': None,
        'default_window_size': 400
    }
 
    # ── header ─────────────────────────────────────────────────────────────────
    with ui.row().style(
        'width: 100%; padding: 16px 24px; box-sizing: border-box; '
        'align-items: center; border-bottom: 1px solid #e0e0e0; '
        'background: white; flex-shrink: 0;'
    ):
        ui.button(icon='arrow_back', on_click=lambda: ui.navigate.to('/')) \
            .props('flat round')
        ui.label('Simulate Miner Experience').style(
            'font-size: 1.3rem; font-weight: 600; margin-left: 12px;'
        )
 
    # ── main content ───────────────────────────────────────────────────────────
    with ui.column().style(
        'flex: 1; width: 100%; padding: 24px; box-sizing: border-box; '
        'gap: 20px; overflow-y: auto;'
    ):
 
        # ── file upload section ────────────────────────────────────────────────
        upload_card = ui.card().style(
            'width: 100%; padding: 24px; box-sizing: border-box;'
        )
        with upload_card:
            ui.label('Load CSV Data').style(
                'font-size: 1.1rem; font-weight: 600; margin-bottom: 16px; display: block;'
            )
            with ui.row().style('align-items: center; gap: 16px; flex-wrap: wrap;'):
                file_label = ui.label('No file loaded').style(
                    'color: #64748b; font-size: 0.9rem; flex: 1;'
                )
                ui.upload(
                    label='Choose CSV file',
                    auto_upload=True,
                    on_upload=lambda e: handle_upload(e),
                ).props('accept=".csv"').style('flex-shrink: 0;')
 
        # ── chart area ─────────────────────────────────────────────────────────
        chart_card = ui.card().style(
            'width: 100%; padding: 16px; box-sizing: border-box;'
        )
        with chart_card:
            ma_input = ui.number(
                label='MA window', value=20, min=1, max=500, step=1
                ).style('width: 100px;')
            max_time_input = ui.number(
                label='Max time (s)', value=None, min=0, step=0.1, placeholder='all'
                ).style('width: 120px;')
            accel_plot = ui.plotly({}).style('width: 100%; height: 280px;')
            accel_plot.on('plotly_selected', lambda e: handle_selection(e))
            fft_plot = ui.plotly({}).style('width: 100%; height: 280px;')
            cap_plot   = ui.plotly({}).style('width: 100%; height: 280px;')
 
        # ── playback controls ──────────────────────────────────────────────────
        controls_card = ui.card().style(
            'width: 100%; padding: 16px 24px; box-sizing: border-box;'
        )
        with controls_card:
            ui.label('Playback').style(
                'font-size: 0.9rem; font-weight: 600; '
                'color: #64748b; margin-bottom: 12px; display: block;'
            )
            with ui.row().style('align-items: center; gap: 16px; flex-wrap: wrap;'):
                btn_zoom_out = ui.button(icon = 'zoom_out', on_click=lambda: reset_window())\
                    .props('flat round').tooltip('reset window size')
                btn_rewind = ui.button(icon='skip_previous', on_click=lambda: rewind()) \
                    .props('flat round').tooltip('Rewind to start')
 
                # step-forward button
                btn_step = ui.button(icon='skip_next', on_click=lambda: step_forward())\
                    .props('flat round').tooltip('Step forward 1 sample')
                btn_play = ui.button(icon='play_arrow', on_click=lambda: toggle_play()) \
                    .props('flat round').tooltip('Play / Pause')
 
                # window position slider
                slider = ui.slider(min=0, max=100, value=0, step=1) \
                    .style('flex: 1; min-width: 200px;') \
                    .on('change', lambda e: seek(e.args))
                # window size selector
                ui.label('Window:').style('color: #64748b; font-size: 0.85rem;')
                ui.select(
                    options={200: '200 samples', 400: '400 samples',
                             800: '800 samples', 1600: '1600 samples', 3200: '3200 samples', 6400: '6400 samples',12800: '12800 samples'},
                    value=400,
                    on_change=lambda e: set_window_size(e.value)
                ).style('min-width: 140px;')
 
                position_label = ui.label('0 / 0 samples').style(
                    'color: #64748b; font-size: 0.85rem; min-width: 130px;'
                )
 
        # ── stats row ──────────────────────────────────────────────────────────
        stats_row = ui.row().style(
            'width: 100%; gap: 12px; flex-wrap: wrap;'
        )
        with stats_row:
            def stat_card(title, value_id):
                with ui.card().style(
                    'flex: 1; min-width: 140px; padding: 16px; '
                    'box-sizing: border-box; text-align: center;'
                ):
                    ui.label(title).style(
                        'font-size: 0.75rem; font-weight: 600; '
                        'color: #94a3b8; text-transform: uppercase; '
                        'letter-spacing: 0.05em; margin-bottom: 4px; display: block;'
                    )
                    return ui.label('—').style(
                        f'font-size: 1.4rem; font-weight: 700; color: #1e293b;'
                    )
 
            lbl_total    = stat_card('Total Samples', 'total')
            lbl_duration = stat_card('Duration', 'duration')
            lbl_max_acc  = stat_card('Peak Accel (LSB)', 'peak_accel')
            lbl_cap_range = stat_card('Cap Range', 'cap_range') 
 
    # ── helpers ────────────────────────────────────────────────────────────────
    def get_df():
        df = state['df']
        if df is None:
                return None
        max_t = max_time_input.value
        if max_t is not None and max_t >0:
            df = df[df['timestamp_us'] <= max_t *1_000_000]
        return df if len(df) >0 else None
    def make_accel_figure(df_window):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=df_window['timestamp_us'] / 1_000_000,
            y=df_window['acceleration'],
            mode='lines',
            name='Acceleration',
            line=dict(color='#3b82f6', width=1.5),
        ))
        fig.update_layout(
            margin=dict(l=48, r=16, t=32, b=40),
            title=dict(text='Acceleration Magnitude (raw LSB)', font=dict(size=13)),
            xaxis=dict(title='Time (s)', showgrid=True, gridcolor='#f1f5f9'),
            yaxis=dict(title='LSB', showgrid=True, gridcolor='#f1f5f9'),
            plot_bgcolor='white',
            paper_bgcolor='white',
            showlegend=False,
            hovermode='x unified',
            dragmode = 'select',
            selectdirection='h'
        )
        return fig.to_dict()
    def make_fft_figure(df_window):
        acc = df_window['acceleration'].values
        if len(acc) < 2:
            return {}
        #estimate the sampling frequency from timestamps
        t = df_window['timestamp_us'].values / 1_000_000
        dt = np.mean(np.diff(t))
        fs = 1.0 /dt if dt > 0 else 1
        #remove DC
        acc = acc - np.mean(acc)

        #fft_plot
        N = len(acc)
        fft_vals = np.fft.rfft(acc)
        freqs = np.fft.rfftfreq(N, d=dt)
        magnitude = np.abs(fft_vals) / N
        fig = go.Figure()
        # exclude the 0 Hz bin 
        fig.add_trace(go.Scatter(
            x=freqs[1:],
            y=magnitude[1:],
        mode='lines',
        line=dict(color='#ef4444', width=1.5),
    ))

        fig.update_layout(
            margin=dict(l=48, r=16, t=32, b=40),
            title=dict(text='Acceleration FFT (Magnitude Spectrum)', font=dict(size=13)),
            xaxis=dict(title='Frequency (Hz)', showgrid=True, gridcolor='#f1f5f9'),
            yaxis=dict(title='Magnitude', showgrid=True, gridcolor='#f1f5f9'),
            plot_bgcolor='white',
            paper_bgcolor='white',
            showlegend=False,
            hovermode='x unified',
        )

        return fig.to_dict()
    def make_cap_figure(df_window, ma_window=20):
        fig = go.Figure()

        # Raw signal (faded)
        fig.add_trace(go.Scatter(
            x=df_window['timestamp_us'] / 1_000_000,
            y=df_window['capacitance'],
            mode='lines',
            name='Raw',
            line=dict(color='rgba(16,185,129,0.25)', width=1),
            fill='tozeroy',
            fillcolor='rgba(16,185,129,0.04)',
        ))

        # Moving average overlay
        ma = df_window['capacitance'].rolling(window=ma_window, center=True, min_periods=1).mean()
        fig.add_trace(go.Scatter(
            x=df_window['timestamp_us'] / 1_000_000,
            y=ma,
            mode='lines',
            name=f'MA({ma_window})',
            line=dict(color='#10b981', width=2),
        ))

        # Base y-range on the MA to avoid raw spikes dominating
        cap_min = ma.min()
        cap_max = ma.max()
        padding = (cap_max - cap_min) * 0.1 or 10

        fig.update_layout(
            margin=dict(l=48, r=16, t=32, b=40),
            title=dict(text='Capacitance (raw ADC counts)', font=dict(size=13)),
            xaxis=dict(title='Time (s)', showgrid=True, gridcolor='#f1f5f9'),
            yaxis=dict(
                title='ADC counts',
                showgrid=True,
                gridcolor='#f1f5f9',
                range=[cap_min - padding, cap_max + padding],
            ),
            legend=dict(
                x=0.01,
                y=0.99,
                xanchor='left',
                yanchor='top',
                bgcolor='rgba(255,255,255,0.7)',
            ),
            plot_bgcolor='white',
            paper_bgcolor='white',
            showlegend=True,
            hovermode='x unified',
        )
        return fig.to_dict()
 
    def update_plots():
        df = get_df() 
        if df is None:
            return
        i = state['window_start']
        j = min(i + state['window_size'], len(df))
        window = df.iloc[i:j]

        accel_plot.figure = make_accel_figure(window)
        accel_plot.update()
        
        fft_plot.figure = make_fft_figure(window)
        fft_plot.update()

        cap_plot.figure = make_cap_figure(window, ma_window = int(ma_input.value))
        cap_plot.update()
 
        # update slider and position label
        max_start = max(0, len(df) - state['window_size'])
        pct = int(i / max_start * 100) if max_start > 0 else 0
        slider.value = pct
        position_label.text = f'{i:,} / {len(df):,} samples'
 
    def update_stats(df):
        duration_s = (df['timestamp_us'].max() - df['timestamp_us'].min()) / 1_000_000
        lbl_total.text    = f'{len(df):,}'
        lbl_duration.text = f'{duration_s:.2f} s'
        lbl_max_acc.text  = f'{int(df["acceleration"].max())}'
        cap_min = 6_000_000 
        cap_max = 9_000_000 
        lbl_cap_range.text = f'{cap_max - cap_min:,}'
 
    def handle_upload(e):
        try:
            content = e.content.read().decode('utf-8', errors='ignore')
            # strip any trailing EOF line the ESP32 adds
            content = '\n'.join(
                line for line in content.splitlines()
                if line.strip().upper() != 'EOF'
            )
            df = pd.read_csv(io.StringIO(content))
            df.columns = [c.strip().lower().replace(' ', '_') for c in df.columns]
 
            # sort by timestamp and drop bad rows
            df = df.sort_values('timestamp_us').reset_index(drop=True)
            df = df[df['timestamp_us'] > 0]
            #zero base the timestamps so plot starts at t=0.
            df['timestamp_us'] = df['timestamp_us'] -df['timestamp_us'].min()

            state['df'] = df
            state['window_start'] = 0
 
            file_label.text = f'{e.name}  —  {len(df):,} samples loaded'
            file_label.style('color: #16a34a; font-size: 0.9rem;')
 
            update_stats(df)
            update_plots()
            ui.notify(f'Loaded {len(df):,} samples', type='positive')
 
        except Exception as ex:
            ui.notify(f'Failed to load file: {ex}', type='negative')
            file_label.text = f'Error: {ex}'
            file_label.style('color: #dc2626; font-size: 0.9rem;')
 
    def rewind():
        state['playing'] = False
        btn_play.props('icon=play_arrow')
        state['window_start'] = 0
        update_plots()
    def step_forward():
        if state['df'] is None:
            return
        df = get_df()
        max_start = max(0,len(df) - state['window_size'])
        state['window_start'] = min(state['window_start']+1,max_start)
        update_plots()
    def handle_selection(e):
        if state['df'] is None or not e.args:
                return
        data = e.args  
        try:
            x_min = data['range']['x'][0]   # seconds
            x_max = data['range']['x'][1]
        except (KeyError, TypeError):
            return

        df = get_df()
        # convert time range to sample indices
        times = df['timestamp_us'] / 1_000_000
        i_start = int((times - x_min).abs().argmin())
        i_end   = int((times - x_max).abs().argmin())

        state['window_start'] = i_start
        state['window_size']  = max(1, i_end - i_start)
        update_plots()
# reset restores to last dropdown value
    def reset_window():
        state['window_size'] = state['default_window_size']
        update_plots()

    def seek(pct):
        if state['df'] is None:
            return
        max_start = max(0, len(state['df']) - state['window_size'])
        state['window_start'] = int(pct / 100 * max_start)
        update_plots()
 
    def set_window_size(size):
        state['window_size'] = size
        state['default_window_size'] = size
        update_plots()
 
    def toggle_play():
        if state['df'] is None:
            ui.notify('Load a CSV file first', type='warning')
            return
        state['playing'] = not state['playing']
        if state['playing']:
            btn_play.props('icon=pause')
            asyncio.ensure_future(play_loop())
        else:
            btn_play.props('icon=play_arrow')
 
    async def play_loop():
        """Advance the window by 1/4 window size every 300ms — smooth scrub."""
        step = max(1, state['window_size'] // 4)
        while state['playing']:
            df = state['df']
            if df is None:
                break
            max_start = max(0, len(df) - state['window_size'])
            if state['window_start'] >= max_start:
                state['window_start'] = max_start
                state['playing'] = False
                btn_play.props('icon=play_arrow')
                update_plots()
                break
            state['window_start'] = min(state['window_start'] + step, max_start)
            update_plots()
            await asyncio.sleep(0.3)
 


@ui.page('/increment_capDAC')
def increment_capDAC():
    ui.add_head_html('''
    <style>
        body, html { margin: 0; padding: 0; height: 100%; }
        .nicegui-content { height: 100vh; display: flex; flex-direction: column; padding: 0 !important; }
        .upload-zone {
            border: 2px dashed #cbd5e1;
            border-radius: 12px;
            padding: 32px;
            text-align: center;
            transition: border-color 0.2s, background 0.2s;
            cursor: pointer;
            background: #f8fafc;
        }
        .upload-zone:hover {
            border-color: #3b82f6;
            background: #eff6ff;
        }
    </style>
    ''')
 
    # ── header ─────────────────────────────────────────────────────────────────
    with ui.row().style(
        'width: 100%; padding: 16px 24px; box-sizing: border-box; '
        'align-items: center; border-bottom: 1px solid #e0e0e0; '
        'background: white; flex-shrink: 0;'
    ):
        ui.button(icon='arrow_back', on_click=lambda: ui.navigate.to('/')) \
            .props('flat round')
        ui.label('Increment CAPDAC').style(
            'font-size: 1.3rem; font-weight: 600; margin-left: 12px;'
        )
        with ui.column().style('width:100%; padding: 24px; box-sizing: border-box; align-items: center;'):
                ui.label('Adjust CAPDAC').style('font-size: 1rem; font-weight: 500; margin-bottom: 8px;')
                slider = ui.slider(min=-63, max=63, value=0, step=1).style('width: 60%;')
                slider_label = ui.label('Increment: 0').style('margin-top: 4px; color: #555;')
                slider.on('update:model-value', lambda e: slider_label.set_text(f'Increment: {e.args}'))

        # ── submit ─────────────────────────────────────────────────────────────────
                async def on_submit():
                    amount = int(slider.value)
                    ser.write(f'I {amount}\n'.encode())
                    
                    while True:
                        await asyncio.sleep(0.1)
                        if ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if not line:
                                continue
                            if line.startswith('0x'):
                                parts = line.split(',')
                                if len(parts) == 2:
                                    capdac_hex = parts[0]
                                    raw = parts[1]
                                    ui.notify(f'CAPDAC: {capdac_hex}  Raw: {raw}', type='positive')
                                else:
                                    ui.notify(line, type='info')
                                break
                            elif 'ERROR' in line:
                                ui.notify(f'Error: {line}', type='negative')
                                break
             
                ui.button('Submit', on_click=on_submit).style('margin-top: 16px; background: #3b82f6; color: white;')
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
            'flex: 1; display: flex; flex-direction: column; align-items: stretch; gap: 20px;'
        ):
            ui.button(
                'Simulate Miner Experience',
                icon='precision_manufacturing',
                on_click=lambda: ui.navigate.to('/simulate')
            ).classes('big-btn hover-lift').style(
                'height: 45%; background-color: #757575 !important; '
                'color: #ffffff !important;'
            )
            ui.button(
                'Increment CAPDAC',
                icon='thumb_up',
                on_click=lambda: ui.navigate.to('/increment_capDAC')
            ).classes('big-btn hover-lift').style(
                'height: 40%; background-color: #DB324D !important; '
                'color: #ffffff !important;'
            )

ui.run(title="Smart Pick Control Dashboard", reload=False)
