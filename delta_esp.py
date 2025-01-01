#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import struct
import crc16
import machine
import os
import gc
from machine import UART, Pin, reset
from umqttsimple import MQTTClient

DEBUG=True
READ_BYTES = 1024
STX=0x02
ETX=0x03
ENQ=0x05
ACK=0x06
NAK=0x15

os.dupterm(None, 1)
conn = UART(0, baudrate=19200, bits=8, parity=None, stop=1,tx=Pin(1), rx=Pin(3), rxbuf=200, timeout=3000)
rts = Pin(4, Pin.OUT)

mqtt_server = 'IP Address' # Change to MQTT Broker IP address
client_id = 'esp_reader' # MQTT client id, needs to be unique
topic_pub_delta = 'Solar' # MQTT topic to publish to

attempt_limit = 10  # Number of attempts before sleeping
sleep_duration = 3600  # Sleep duration in seconds (1 hour)
attempts = 0
last_message = 0
message_interval = 60

def ma_mi(data):
    ma, mi = struct.unpack('>BB', data)
    return '{:02d}.{:02d}'.format(ma, mi)

# Variables in the data-block of a Delta Solivia 5.0 AP G3 inverter,
# If you have a different inverter you may have to adjust data structure,
# Refer to Public RS485 Protocol 1V2.pdf for your specific model
# The fields for each variable are as follows: 
# name, struct, size in bytes, decoder, multiplier-exponent (10^x), unit
DELTA_DATA = (
    ("SAP part number", "11s", str),
    ("SAP serial number", "18s", str),
    ("SAP date code", "4s", str),
    ("SAP revision", "2s", str),
    ("Software revision AC control", "2s", ma_mi, 0),
    ("Software revision DC control", "2s", ma_mi, 0),
    ("Software revision display", "2s", ma_mi, 0),
    ("Software revision ens control", "2s", ma_mi, 0),
    ("Solar Current at Input 1", "H", float, -1, "A"),
    ("Solar Voltage at Input 1", "H", float, 0, "V"),
    ("Solar Isolation Resistance at Input 1", "H", int, 0, "kOhm"),
    ("Solar Current at Input 2", "H", float, -1, "A"),
    ("Solar Voltage at Input 2", "H", float, 0, "V"),
    ("Solar Isolation Resistance at Input 2", "H", int, 0, "kOhm"),
    ("AC Current", "H", float, -1, "A"),    
    ("AC Voltage", "H", float, 0, "V"),
    ("AC Power", "H", int, 0, "W"),
    ("AC Frequency", "H", float, -2, "Hz"),
    ("Supplied AC energy", "H", int, 0, "Wh"),
    ("Inverter runtime", "H", int, 0, "Minutes"),
    ("Calculated temperature at ntc (DC side)", "h", int, 0, "C"),
    ("Solar input 1 MOV resistance", "H", int, 0, "kOhm"),
    ("Solar input 2 MOV resistance", "H", int, 0, "kOhm"),
    ("Calculated temperature at ntc (AC side)", "h", int, 0, "C"),
    ("AC voltage (AC control)", "H", float, -1, "V"), 
    ("AC frequency (AC control)", "H", float, -2, "Hz"),
    ("DC injection current (AC control)", "H", float, -3, "A"), 
    ("AC voltage (ENS control)", "H", float, -1, "V"),
    ("AC frequency (ENS control)", "H", float, -2, "Hz"),
    ("DC injection current (ENS control)", "H", float, -3, "A"),
    ("Maximum solar 1 input current", "H", float, -1, "A"),
    ("Maximum solar 1 input voltage", "H", float, 0, "V"),
    ("Maximum solar 1 input power", "H", int, 0, "W"),
    ("Minimum isolation resistance solar 1", "H", int, 0, "kOhm"),
    ("Maximum isolation resistance solar 1", "H", int, 0, "kOhm"),
    ("Maximum solar 2 input current", "H", float, -1, "A"),
    ("Maximum Solar 2 input voltage", "H", float, 0, "V"),
    ("Maximum solar 2 input power", "H", int, 0, "W"),
    ("Minimum isolation resistance solar 2", "H", int, 0, "kOhm"),
    ("Maximum isolation resistance solar 2", "H", int, 0, "kOhm"),
    ("Maximum AC Current of today", "H", float, -1, "A"),
    ("Minimum AC Voltage of today", "H", float, 0, "V"),    
    ("Maximum AC Voltage of today", "H", float, 0, "V"),
    ("Maximum AC Power of today", "H", int, 0, "W"),
    ("Minimum AC Frequency of today", "H", float, -2, "Hz"),
    ("Maximum AC Frequency of today", "H", float, -2, "Hz"),
    ("Supplied AC energy", "I", int, -1, "KWh"),
    ("Inverter Runtime", "I", int, 0, "Hours"),
    ("Global Alarm Status", "B", int),
    ("Status DC Input", "B", int),
    ("Limits DC Input", "B", int),
    ("Status AC Input", "B", int),
    ("Limits AC Input", "B", int),
    ("Isolation Warning Status", "B", int),
    ("DC Hardware Failure", "B", int),
    ("AC Hardware Failure", "B", int),
    ("ENS Hardware Failure", "B", int),
    ("Internal Bulk Failure", "B", int),
    ("Internal Communications Failure", "B", int),
    ("AC Hardware Disturbance", "B", int),
    ("History status messages", "20s", str),
)

DELTA_STRUCT = '>' + ''.join([item[1] for item in DELTA_DATA])

def connect_mqtt():
  global client_id, mqtt_server
  client = MQTTClient(client_id, mqtt_server, user="USER", password="PASS") # Change USER and PASS to MQTT Broker details
  client.connect()
  print('Connected to %s MQTT broker' % (mqtt_server))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()
  
##Send cmd/subcmd (e.g. 0x60/0x01) and optional data to the RS485 bus
def send(conn, req, cmd, subcmd, data=b'', addr=1):
    # Raise RTS to signal the start of transmission
    rts.on()
    time.sleep_ms(2)  # Allow signal to stabilize

    assert req in (ENQ, ACK)  # req should be one of ENQ, ACK
    msg = struct.pack('BBBBB', req, addr, 2 + len(data), cmd, subcmd)
    if len(data) > 0:
        msg = struct.pack('5s%ds' % len(data), msg, data)
    crcval = crc16.calcData(msg)
    lsb = crcval & (0xff)
    msb = (crcval >> 8) & 0xff
    data = struct.pack('B%dsBBB' % len(msg), STX, msg, lsb, msb, ETX)
    if DEBUG: print(">>> SEND:", (msg), "=>", (data))
    
    conn.write(data)
    conn.flush()
    
    # Lower RTS to signal end of transmission (ready to receive)
    time.sleep_ms(5)  # Short delay to ensure transmission completion
    rts.off()

## Attempt to read messages from a serial connection
def receive(conn):
    data = bytearray()
    while True:
        buf = conn.read(READ_BYTES)
        if buf:
            if DEBUG: print(">>> RAW RECEIVE:", buf)
            data.extend(buf)
        if (not buf) or len(buf) < READ_BYTES:
            break

    idx = 0
    while idx + 9 <= len(data):
        if data[idx] != STX:
            idx += 1
            continue
        stx, req, addr, size = struct.unpack('>BBBB', data[idx:idx+4])
        if req not in (ACK, NAK):
            print("Bad req value: {:02x} (should be one of ACK/NAK)".format(req))
            idx += 1
            continue
        if idx + 4 + size + 3 > len(data):
            print("Can't read %d bytes from buffer" % size)
            idx += 1
            continue
        msg, lsb, msb, etx = struct.unpack('>%dsBBB' % size, data[idx+4:idx+7+size])
        if etx != ETX:
            print("Bad ETX value: {:02x}".format(etx))
            idx += 1
            continue
        crc_calc = crc16.calcData(data[idx+1:idx+4+size])
        crc_msg = msb << 8 | lsb
        if crc_calc != crc_msg:
            print("Bad CRC check")
            idx += 1
            continue
        yield {
             "data": data,
             "stx": stx,
             "req": req,
             "addr": addr,
             "size": size,
             "msg": msg,
             "lsb": lsb,
             "msb": msb,
             "etx": etx,
        }
        idx += 4 + size

def decode_msg(data):
    req = data['req']
    data['raw'] = data['msg'][2:]
    if req == ACK:
        data['values'] = struct.unpack(DELTA_STRUCT, data['raw'])
    return data

def send_data_chunk(client, data_points_chunk):
    # Function to process each chunk of data and publish it via MQTT
    line_data = "\n".join(data_points_chunk)
    try:
        client.publish(topic_pub_delta, line_data)  # Publish the influx line data
        print(f"Message successfully published to {topic_pub_delta}: {line_data}")
    except OSError as e:
        print(f"Error publishing data: {e}")
        restart_and_reconnect()

def process_data_in_chunks(client, decoded, measurement="Solar_Power", chunk_size=10):
    data_points = []  # Initialize the list for data points
    for i, item in enumerate(DELTA_DATA):
        label = item[0]
        decoder = item[2]
        scale = item[3] if len(item) > 3 else 0
        units = item[4] if len(item) > 4 else ''
        value = decoder(decoded['values'][i])
        
        if not units:
            print(f"Skipping entry without units: {label}")
            continue
        
        if scale != 0:
            value *= pow(10, scale)
        
        # If the decoder is a float, format the value with one decimal place
            if decoder == float:
                value = round(value, 1)
            if decoder == str:
                if isinstance(value, bytes):
                    value = value.decode("utf-8")  # Decode bytes to string
                elif not isinstance(value, str):
                    raise TypeError(f"Expected value to be bytes or str, but got {type(value)}")
        
        # Escape spaces in label
        escaped_label = label.replace(" ", r"\ ")
        
        # Add to the data_points list
        fields = f'units="{units}",value={value}'
        tags = f'label={escaped_label}'
        line = f"{measurement},{tags} {fields}"
        data_points.append(line)
        
        # Process the chunk if we've reached the chunk size
        if len(data_points) >= chunk_size:
            send_data_chunk(client, data_points)  
            data_points = [] 
    
    # Process any remaining data if less than chunk_size
    if data_points:
        send_data_chunk(client, data_points)
    
    gc.collect()
    return True

def main():
    global last_message,attempts
    
    try:
        client = connect_mqtt()  
    except OSError as e:
        print(f"Failed to connect to MQTT: {e}")
        restart_and_reconnect()
        
    while True:
        if last_message is None or (time.time() - last_message) > message_interval:
            send(conn, ENQ, 0x60, 0x01)
            time.sleep_ms(100)
            response_received = False  # Assume no response
            
            for data in receive(conn):
                if data['req'] == (ACK):
                    decoded = decode_msg(data)
                    process_data_in_chunks(client, decoded, measurement="Solar_Power", chunk_size=10)
                    last_message = time.time()
                    gc.collect()
                    print("Completed processing and submitting data")
                    response_received = True

            if not response_received:
                attempts += 1
                if attempts >= attempt_limit:
                    print("No response after 10 attempts. Sleeping for 1 hour.")
                    time.sleep(sleep_duration)  # Sleep for 1 hour
                    attempts = 0  # Reset attempts after sleep
                        
if __name__ == "__main__":
    main()
