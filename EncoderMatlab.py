import socket
import RPi.GPIO as GPIO
import time
import math
import threading

# TCP Server setup
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 65432      # Port to listen on

# Encoder setup
ENCODER_A_PIN = 17
ENCODER_B_PIN = 27
ENCODER_Z_PIN = 22
ppr = 1280  # Pulses per revolution
encoderPos = 0  # Current encoder position
last_time = time.time()  # Last time a reading was taken
last_encoderPos = 0  # Encoder position at last reading

# Callback for A pin
def doEncoderA(channel):
    global encoderPos
    if GPIO.input(ENCODER_A_PIN) == GPIO.input(ENCODER_B_PIN):
        encoderPos += 1
    else:
        encoderPos -= 1

# Callback for B pin
def doEncoderB(channel):
    global encoderPos
    if GPIO.input(ENCODER_A_PIN) != GPIO.input(ENCODER_B_PIN):
        encoderPos += 1
    else:
        encoderPos -= 1

# Callback for Z pin
def doEncoderZ(channel):
    global encoderPos
    encoderPos = 0  # Reset encoder position on Z pulse

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_Z_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=doEncoderA)
GPIO.add_event_detect(ENCODER_B_PIN, GPIO.BOTH, callback=doEncoderB)
GPIO.add_event_detect(ENCODER_Z_PIN, GPIO.RISING, callback=doEncoderZ)

# Function to continuously read encoder and calculate angle and velocity
def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print('Waiting for connection')
        global encoderPos, last_time, last_encoderPos

        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    print('Connected by', addr)
                    while True:
                        current_time = time.time()
                        encoder_delta = encoderPos - last_encoderPos
                        angle = (encoderPos / float(ppr * 4)) * 2 * math.pi
                        angle_delta = (encoder_delta / float(ppr * 4)) * 2 * math.pi
                        time_interval = current_time - last_time
                        average_angular_velocity = angle_delta / time_interval if time_interval > 0 else 0

                        # form data to string, separate by coma
                        data_to_send = "{:.2f},{:.2f}\n".format(angle, average_angular_velocity)
                        conn.sendall(data_to_send.encode())

                        last_time = current_time
                        last_encoderPos = encoderPos
                        time.sleep(0.01)
            except socket.error as e:
                print(f"Socket error: {e}")
            except KeyboardInterrupt:
                print('Server shutdown')
                break

# Function to print the angle and angular velocity continuously
def print_values():
    global encoderPos, last_time, last_encoderPos
    while True:
        current_time = time.time()
        time_interval = current_time - last_time
        encoder_delta = encoderPos - last_encoderPos
        angle = (encoderPos / float(ppr * 4)) * 2 * math.pi
        angle_delta = (encoder_delta / float(ppr * 4)) * 2 * math.pi
        average_angular_velocity = angle_delta / time_interval if time_interval > 0 else 0

        print("\rAngle: {:.2f} rad, Angular Velocity: {:.2f} rad/s".format(angle, average_angular_velocity), end="")
        time.sleep(0.1)  # Adjust the sleep time as needed

# Start the server in a separate thread
encoder_thread = threading.Thread(target=start_server)
encoder_thread.start()

# Start the print_values function in a separate thread
print_thread = threading.Thread(target=print_values)
print_thread.start()

# Main program loop
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    GPIO.cleanup()
    encoder_thread.join()
    print_thread.join()

print("Test complete")
