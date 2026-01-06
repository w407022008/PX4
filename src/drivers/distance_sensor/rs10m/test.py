#!/usr/bin/env python3
"""
RS10M Distance Sensor Test Script
This script reads data from a distance sensor connected to /dev/ttyUSB0
and parses distance values in the format "[num] mm\r\n".
"""
import serial
import time
import re
import sys
from enum import Enum


class ParseState(Enum):
    UNSYNCED = 0      # Not synchronized with frame
    IN_NUMBER = 1     # Reading digits of the distance value
    GOT_SPACE = 2     # Got ' ' (space) - first char of end sequence
    GOT_FIRST_M = 3   # Got first 'm'
    GOT_SECOND_M = 4  # Got second 'm'
    GOT_CR = 5        # Got '\r' (carriage return)
    GOT_LF = 6        # Got '\n' (line feed)


def parse_distance_data(data_buffer):
    """
    Parse distance data from the sensor buffer to extract the last complete frame
    """
    state = ParseState.UNSYNCED
    num_start = -1  # Start index of the distance number
    num_end = -1    # End index of the distance number
    found_frame = False
    distance_m = -1.0

    for i, c in enumerate(data_buffer):
        char = chr(c) if isinstance(c, int) else c

        if state == ParseState.UNSYNCED:
            if char.isdigit():
                # Start of a potential number
                num_start = i
                state = ParseState.IN_NUMBER
        elif state == ParseState.IN_NUMBER:
            if char.isdigit():
                # Continue reading the number
                pass
            elif char == ' ':
                # Number ended, start of frame end sequence
                num_end = i - 1  # Last digit of the number
                # Extract and validate the distance value
                if num_start >= 0 and num_end >= num_start:
                    # Create a temporary string for the number
                    num_str = data_buffer[num_start:num_end + 1].decode('ascii', errors='ignore')

                    try:
                        dist_value = float(num_str)
                        # Validate conversion and range (0-10m = 0-10000mm)
                        if dist_value >= 0.0 and dist_value <= 10000.0:
                            distance_m = dist_value / 1000.0  # Convert mm to m
                            found_frame = True
                    except ValueError:
                        pass  # Invalid number, continue parsing
                state = ParseState.GOT_SPACE
            else:
                # If we get anything other than a space after the number, reset
                num_start = -1
                num_end = -1
                found_frame = False
                state = ParseState.UNSYNCED
        elif state == ParseState.GOT_SPACE:
            if char == 'm':
                state = ParseState.GOT_FIRST_M
            else:
                # Unexpected, reset to look for number
                num_start = -1
                num_end = -1
                found_frame = False
                if char.isdigit():
                    num_start = i
                    state = ParseState.IN_NUMBER
                else:
                    state = ParseState.UNSYNCED
        elif state == ParseState.GOT_FIRST_M:
            if char == 'm':
                state = ParseState.GOT_SECOND_M
            else:
                # Unexpected, reset
                num_start = -1
                num_end = -1
                found_frame = False
                if char.isdigit():
                    num_start = i
                    state = ParseState.IN_NUMBER
                else:
                    state = ParseState.UNSYNCED
        elif state == ParseState.GOT_SECOND_M:
            if char == '\r':
                state = ParseState.GOT_CR
            elif char == 'm':
                # If we get another 'm', stay in this state
                state = ParseState.GOT_SECOND_M
            else:
                # Unexpected, reset
                num_start = -1
                num_end = -1
                found_frame = False
                if char.isdigit():
                    num_start = i
                    state = ParseState.IN_NUMBER
                else:
                    state = ParseState.UNSYNCED
        elif state == ParseState.GOT_CR:
            if char == '\n':
                state = ParseState.GOT_LF
            else:
                # Unexpected, reset
                num_start = -1
                num_end = -1
                found_frame = False
                if char.isdigit():
                    num_start = i
                    state = ParseState.IN_NUMBER
                else:
                    state = ParseState.UNSYNCED
        elif state == ParseState.GOT_LF:
            # Frame complete, look for next number
            if char.isdigit():
                # Start of a potential number
                num_start = i
                num_end = -1
                state = ParseState.IN_NUMBER
            else:
                # Stay unsynchronized
                state = ParseState.UNSYNCED

    # If no valid frame was found or the distance is invalid, return None
    if not found_frame or distance_m < 0.0:
        return None

    return distance_m


def main():
    # Serial port configuration
    port = '/dev/ttyUSB0'
    baudrate = 115200

    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud")

        # Clear any initial buffer
        ser.flushInput()

        print("Reading distance data... Press Ctrl+C to exit")

        while True:
            # Read available data
            if ser.in_waiting > 0:
                # Read all available bytes
                data = ser.read(ser.in_waiting)

                # Decode to string for display purposes
                try:
                    raw_str = data.decode('ascii', errors='ignore')
                    print(f"Raw data: {repr(raw_str)}")  # Print raw data for debugging
                except:
                    print(f"Raw bytes: {data}")

                # Add the new data to our buffer
                if not hasattr(main, 'buffer'):
                    main.buffer = bytearray()

                main.buffer.extend(data)

                # Limit buffer size to prevent overflow
                if len(main.buffer) > 1024:
                    main.buffer = main.buffer[-512:]  # Keep last 512 bytes

                # Try to parse distance from the buffer
                distance = parse_distance_data(main.buffer)

                if distance is not None:
                    print(f"Distance: {distance:.3f} m ({distance*1000:.0f} mm)")
                    # Reset buffer after successful parsing
                    main.buffer = bytearray()

            time.sleep(0.01)  # Small delay to prevent excessive CPU usage

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()
        sys.exit(0)


if __name__ == "__main__":
    main()
