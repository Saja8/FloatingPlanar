import socket
import RPi.GPIO as GPIO
import time
import math
import threading

# TCP Server setup
HOST = '0.0.0.0'
PORT = 65433

# Motor and Encoder setup
PWM_PIN = 12
DIR_PIN = 16
ENCODER_A_PIN = 17
ENCODER_B_PIN = 27
ENCODER_Z_PIN = 4
ppr = 640
encoderPos = 0

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([PWM_PIN, DIR_PIN], GPIO.OUT)
GPIO.setup([ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_Z_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setup PWM
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

try:
    GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=doEncoderA)
    GPIO.add_event_detect(ENCODER_B_PIN, GPIO.BOTH, callback=doEncoderB)
    GPIO.add_event_detect(ENCODER_Z_PIN, GPIO.RISING, callback=doEncoderZ)
except RuntimeError as e:
    print(f"RuntimeError while adding event detection: {e}")
    GPIO.cleanup()
    exit(1)

# Function to run motor
def run_motor(direction, duration, speed):
    GPIO.output(DIR_PIN, direction)
    pwm.start(speed)
    time.sleep(duration)
    pwm.stop()

# Function to send encoder data
def send_encoder_data(conn):
    global encoderPos
    last_time = time.time()
    last_encoderPos = encoderPos

    while True:
        current_time = time.time()
        time_interval = current_time - last_time
        encoder_delta = encoderPos - last_encoderPos
        angle = (encoderPos / float(ppr * 4)) * 2 * math.pi
        angle_delta = (encoder_delta / float(ppr * 4)) * 2 * math.pi
        average_angular_velocity = angle_delta / time_interval if time_interval > 0 else 0

        # Send data to client
        data_to_send = "{:.2f},{:.2f}\n".format(angle, average_angular_velocity)
        try:
            conn.sendall(data_to_send.encode())
        except socket.error:
            break

        # Update for next iteration
        last_time = current_time
        last_encoderPos = encoderPos
        time.sleep(0.01)

# Function to handle client connections
def handle_client(conn):
    # Start a new thread for sending encoder data
    encoder_thread = threading.Thread(target=send_encoder_data, args=(conn,))
    encoder_thread.start()

    try:
        while True:
            # Receive motor control command
            data = conn.recv(1024).decode()
            if not data:
                break
            direction, duration, speed = map(int, data.split(','))
            run_motor(direction, duration, speed)
    except socket.error as e:
        print(f'Socket error: {e}')
    finally:
        conn.close()
        encoder_thread.join()

# TCP Server loop
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    print('Motor Server is ready for connections')

    while True:
        try:
            conn, addr = s.accept()
            print(f'Connected by {addr}')
            client_thread = threading.Thread(target=handle_client, args=(conn,))
            client_thread.start()
        except KeyboardInterrupt:
            print('Motor Server shutdown')
            break

# Ensure GPIO cleanup
pwm.stop()
GPIO.cleanup()
print('Motor and Encoder server shutdown')
