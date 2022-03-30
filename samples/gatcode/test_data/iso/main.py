#!/usr/bin/python3
import os
import datetime
import time
import pandas as pd
from bokeh.layouts import row, column
from bokeh.plotting import figure, output_file, show
from bokeh.models import ColumnDataSource, Grid, LinearAxis, Plot, Step, DatetimeTickFormatter
from bokeh.models.tools import CrosshairTool, WheelZoomTool

# -------------------------------------------------------------------------------
# Definitions
# -------------------------------------------------------------------------------

LOGFILE_PATH = os.getcwd() + "/logs/"
SENDER_CSV_PATH = LOGFILE_PATH + "mote_116.csv"
RECEIVER_CSV_PATH = LOGFILE_PATH + "mote_117.csv"

# -------------------------------------------------------------------------------
# Globals
# -------------------------------------------------------------------------------

gpio_8bit = {
    'bit_0': 0,
    'bit_1': 0,
    'bit_2': 0,
    'bit_3': 0,
    'bit_4': 0,
    'bit_5': 0,
    'bit_6': 0,
    'bit_7': 0
}

# -------------------------------------------------------------------------------
# Code
# -------------------------------------------------------------------------------

def open_csv_files():
    date_cols = ['time']
    sender_df = pd.read_csv(SENDER_CSV_PATH, parse_dates=date_cols)
    receiver_df = pd.read_csv(RECEIVER_CSV_PATH, parse_dates=date_cols)
    return { 'sender' : sender_df, 'receiver' : receiver_df }

def datetime_to_timestamp_us(datetime):
    return int(datetime.timestamp() * 1000000)

def replace_datetime_with_timestamp_in_df(dataframe):
    dataframe['time_hr'] = dataframe['time']
    dataframe['time'] = dataframe['time'].apply(datetime_to_timestamp_us)

def combine_timestamps_within_tolerance(dataframe, tolerance_us):
    last_timestamp_us = dataframe.iloc[0]['time']
    for index, row in dataframe.iterrows():
        current_timestamp_us = row['time']
        if(current_timestamp_us - last_timestamp_us <= tolerance_us):
            dataframe.at[index, 'time'] = last_timestamp_us
            last_timestamp_us = last_timestamp_us
        else:
            last_timestamp_us = current_timestamp_us

def group_rows_by_timestamp(dataframe):
    return dataframe.groupby(['time']).agg(tuple).applymap(list).reset_index()

def remove_duplicate_hr_ts(dataframe):
    for index, row in dataframe.iterrows():
        dataframe.at[index, 'time_hr'] = row['time_hr'][0]

def reformat_hr_ts(dataframe):
    for index, row in dataframe.iterrows():
        dataframe.at[index, 'time_hr'] = pd.to_datetime(row['time_hr'])#.strftime('%H:%M:%S.%f')

def encode(gpio, level):
    if gpio == 25:
        gpio_8bit['bit_0'] = 1 if level == 1 else 0
    elif gpio == 24:
        gpio_8bit['bit_1'] = 1 if level == 1 else 0
    elif gpio == 23:
        gpio_8bit['bit_2'] = 1 if level == 1 else 0
    elif gpio == 22:
        gpio_8bit['bit_3'] = 1 if level == 1 else 0
    elif gpio == 27:
        gpio_8bit['bit_4'] = 1 if level == 1 else 0
    elif gpio == 18:
        gpio_8bit['bit_5'] = 1 if level == 1 else 0
    elif gpio == 4:
        gpio_8bit['bit_6'] = 1 if level == 1 else 0
    elif gpio == 17:
        gpio_8bit['bit_7'] = 1 if level == 1 else 0
    return

def encode_8bit_and_add_new_column(dataframe):
    gpio_encoded_8bit = []
    for index, row in dataframe.iterrows():
        gpio = row['evt_gpio']
        level = row['evt_lvl']
        list(map(encode, gpio, level))
        gpio_encoded_8bit.append(gpio_8bit.copy())
    dataframe.insert(3, '8bit_encoding', gpio_encoded_8bit)

def decode_gpio_8bit(dict):
    return dict['bit_0'] * 1 + dict['bit_1'] * 2 + dict['bit_2'] * 4 + \
           dict['bit_3'] * 8 + dict['bit_4'] * 16 + dict['bit_5'] * 32 + \
           dict['bit_6'] * 64 + dict['bit_7'] * 128

def reset_gpio_8bit():
    gpio_8bit['bit_0'] = 0
    gpio_8bit['bit_1'] = 0
    gpio_8bit['bit_2'] = 0
    gpio_8bit['bit_3'] = 0
    gpio_8bit['bit_4'] = 0
    gpio_8bit['bit_5'] = 0
    gpio_8bit['bit_6'] = 0
    gpio_8bit['bit_7'] = 0

def decode_8bit_encoding_and_add_new_column(dataframe):
    gpio_decoded_8bit = []
    for index, row in dataframe.iterrows():
        gpio_decoded_8bit.append(decode_gpio_8bit(row['8bit_encoding']))
    dataframe.insert(4, '8bit_value', gpio_decoded_8bit)

def prepare_dataframe(dataframe):
    reset_gpio_8bit()
    replace_datetime_with_timestamp_in_df(dataframe)
    combine_timestamps_within_tolerance(dataframe, 50)
    dataframe = group_rows_by_timestamp(dataframe)
    reformat_hr_ts(dataframe)
    remove_duplicate_hr_ts(dataframe)
    encode_8bit_and_add_new_column(dataframe)
    decode_8bit_encoding_and_add_new_column(dataframe)
    return dataframe

def draw_step_plot(sender_df, receiver_dfs, title):
    p = figure(x_axis_type='datetime', plot_height=500, plot_width=1400)
    p.xaxis.formatter=DatetimeTickFormatter(
        microseconds = ['%H:%M:%S.2N'],
        milliseconds = ['%H:%M:%S.%2N'],
        seconds = ['%H:%M:%S.%2N'],
        minsec = ['%H:%M:%S.%2N'],
        minutes = ['%H:%M:%S.%2N'],
        hourmin = ['%H:%M:%S.%2N'],
        hours = ['%H:%M:%S.%2N'],
        days = ['%H:%M:%S.%2N'],
        months = ['%H:%M:%S.%2N'],
        years = ['%H:%M:%S.%2N'],
    )

    sender = Step(x='time_hr', y='8bit_value', line_color="#f46d43", mode="after")
    p.add_glyph(ColumnDataSource(sender_df), sender)

    for recv_df in receiver_dfs:
        recv = Step(x='time_hr', y='8bit_value', line_color="#4f4f4f", mode="after")
        p.add_glyph(ColumnDataSource(recv_df), recv)

    p.title.text = title
    p.xaxis.axis_label = 'Time'
    p.yaxis.axis_label = 'Packet ID'
    p.toolbar.active_scroll = p.select_one(WheelZoomTool)
    p.toolbar_location = None
    p.add_tools(CrosshairTool())
    return p

def calculate_latency(dataframe):
    for index, row in dataframe.iterrows():
        dataframe.at[index, 'latency_us'] = row['time_receiver'] - row['time_sender']
        dataframe.at[index, 'latency_ms'] = int(row['time_receiver'] - row['time_sender']) / 1000
    dataframe['latency_us'] = dataframe['latency_us'].astype(int)

def main():
    csv_files = open_csv_files()
    sender_df = prepare_dataframe(csv_files['sender'])
    receiver_df = prepare_dataframe(csv_files['receiver'])
    
    step_plot = draw_step_plot(sender_df, [receiver_df], 'Packet IDs over time (50us tolerance)')
    show(column(row(step_plot)))

    merged = pd.merge(sender_df[['time', '8bit_value']], receiver_df[['time', '8bit_value']], on = '8bit_value', suffixes = ('_sender', '_receiver'))
    calculate_latency(merged)

    print(merged)

if __name__=="__main__":
    main()