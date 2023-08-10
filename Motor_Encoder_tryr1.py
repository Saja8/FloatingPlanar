import RPi.GPIO as GPIO
import time

# Pin definitions
ENCODER_A_PIN = 17  # GPIO17
ENCODER_B_PIN = 27  # GPIO27
ENCODER_Z_PIN = 22  # GPIO27

# Other definitions
ppr = 600  # pulses per revolution
encoderPos = 0  # Variable to hold the encoder position

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

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach interrupts
GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=doEncoderA)
GPIO.add_event_detect(ENCODER_B_PIN, GPIO.BOTH, callback=doEncoderB)

# Main loop
try:
    while True:
        angle = (encoderPos / (float)(ppr*4)) * 360.0  # convert to degrees
        print("Angle: ", angle)
        time.sleep(0.1)  # Adjust the sleep time to your preference for updating the angle printout

except KeyboardInterrupt:
    GPIO.cleanup()
