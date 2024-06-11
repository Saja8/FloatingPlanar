import socket
import RPi.GPIO as GPIO
import time
import math
import threading
import sys
import numpy as np

# Import IMU library
sys.path.append('/home/saja48/BerryIMU/python-BerryIMU-gyro-accel-compass-filters/')
import IMU

# Constants for motor and encoder
HOST_MOTOR = '0.0.0.0'
PORT_MOTOR = 65433
PWM_PIN = 12
DIR_PIN = 16
ENCODER_A_PIN = 17
ENCODER_B_PIN = 27
ENCODER_Z_PIN = 4
ppr = 640
encoderPos = 0

# Constants for IMU
HOST_IMU = '0.0.0.0'
PORT_IMU = 65434
ACC_LPF_FACTOR = 0.4
ACC_MEDIANTABLESIZE = 5
WINDOW_SIZE = 100

# Initialize IMU
IMU.detectIMU()
if IMU.BerryIMUversion == 99:
    print("No BerryIMU found... exiting")
    sys.exit()
IMU.initIMU()

# GPIO setup for motor and encoder
GPIO.setmode(GPIO.BCM)
GPIO.setup([PWM_PIN, DIR_PIN], GPIO.OUT)
GPIO.setup([ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)
pwm = GPIO.PWM(PWM_PIN, 1000)
pwm.start(0)

# Encoder callbacks
def doEncoderA(channel):
    global encoderPos
    if GPIO.input(ENCODER_A_PIN) == GPIO.input(ENCODER_B_PIN):
        encoderPos += 1
    else:
        encoderPos -= 1

def doEncoderB(channel):
    global encoderPos
    if GPIO.input(ENCODER_A_PIN) != GPIO.input(ENCODER_B_PIN):
        encoderPos += 1
    else:
        encoderPos -= 1

def doEncoderZ(channel):
    global encoderPos
    encoderPos = 0

GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=doEncoderA)
GPIO.add_event_detect(ENCODER_B_PIN, GPIO.BOTH, callback=doEncoderB)
GPIO.add_event_detect(ENCODER_Z_PIN, GPIO.RISING, callback=doEncoderZ)

# Function to run motor
def run_motor(direction, duration, speed):
    GPIO.output(DIR_PIN, direction)
    pwm.start(speed)
    time.sleep(duration)
    pwm.stop()

# Median filter function for IMU data
def median_filter(input_value, median_table):
    median_table.append(input_value)
    if len(median_table) > ACC_MEDIANTABLESIZE:
        median_table.pop(0)
    return sorted(median_table)[len(median_table) // 2]

# Function to send combined encoder and IMU data
def send_combined_data(conn):
    global encoderPos
    last_encoderPos = encoderPos
    last_time = time.time()
    previous_ACCz = 0
    acc_z_buffer = np.zeros(WINDOW_SIZE)
    buffer_index = 0

    while True:
        # Encoder data
        current_time = time.time()
        encoder_delta = encoderPos - last_encoderPos
        angle = (encoderPos / float(ppr * 4)) * 2 * math.pi
        angle_delta = (encoder_delta / float(ppr * 4)) * 2 * math.pi
        average_angular_velocity = angle_delta / (current_time - last_time) if current_time != last_time else 0

        # Accelerometer data
        ACCz = IMU.readACCz()
        ACCz = ACCz * ACC_LPF_FACTOR + previous_ACCz * (1 - ACC_LPF_FACTOR)
        acc_z_buffer[buffer_index % WINDOW_SIZE] = ACCz
        buffer_index += 1

        # Vibration amplitude and dominant frequency
        vibration_amplitude = abs(ACCz - previous_ACCz)
        dominant_frequency = 0
        if buffer_index >= WINDOW_SIZE:
            window_data = acc_z_buffer - np.mean(acc_z_buffer)
            fft_result = np.fft.fft(window_data)
            frequencies = np.fft.fftfreq(WINDOW_SIZE, d=0.01)
            dominant_frequency_index = np.argmax(np.abs(fft_result))
            dominant_frequency = frequencies[dominant_frequency_index]

        # Combined data packet
        data_to_send = f"{angle:.2f},{average_angular_velocity:.2f},{vibration_amplitude:.2f},{dominant_frequency:.2f}\n"

        try:
            conn.sendall(data_to_send.encode())
        except socket.error:
            break

        last_time = current_time
        last_encoderPos = encoderPos
        previous_ACCz = ACCz
        time.sleep(0.01)  # Sleep for consistent interval

# Function to handle client connections
def handle_client(conn):
    send_thread = threading.Thread(target=send_combined_data, args=(conn,))
    send_thread.start()

    try:
        while True:
            data = conn.recv(1024).decode()
            if not data:
                break
            direction, duration, speed = map(int, data.split(','))
            run_motor(direction, duration, speed)
    except socket.error as e:
        print(f'Socket error: {e}')
    finally:
        conn.close()
        send_thread.join()

# Main server function
def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST_MOTOR, PORT_MOTOR))
        s.listen()
        print('Server is ready for connections')

        while True:
            try:
                conn, addr = s.accept()
                print(f'Connected by {addr}')
                client_thread = threading.Thread(target=handle_client, args=(conn,))
                client_thread.start()
            except KeyboardInterrupt:
                print('Server shutdown')
                break

    pwm.stop()
    GPIO.cleanup()
    print('Server shutdown')

# Start the server
start_server()

