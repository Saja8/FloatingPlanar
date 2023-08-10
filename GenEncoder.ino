// Pin definitions
#define ENCODER_A_PIN 2 // insert your A pin number here
#define ENCODER_B_PIN 3 // insert your B pin number here
#define ENCODER_Z_PIN 4 // insert your Z pin number here

const int ppr = 1250; // pulses per revolution
volatile long encoderPos = 0; // Variable to hold the encoder position
volatile bool A_set = false;
volatile bool B_set = false;
int lastZState;
bool useRadians = true; // change this to false if you want to use degrees

// The ISR for the A pin
void doEncoderA() {
  if ( digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN) ) {
    encoderPos++;
  } else {
    encoderPos--;
  }
  A_set = true;
  B_set = false;
}

// The ISR for the B pin
void doEncoderB() {
  if ( digitalRead(ENCODER_A_PIN) != digitalRead(ENCODER_B_PIN) ) {
    encoderPos++;
  } else {
    encoderPos--;
  }
  B_set = true;
  A_set = false;
}

void setup() {
  Serial.begin(9600);
  
  pinMode(ENCODER_A_PIN, INPUT_PULLUP); // set pins as inputs:
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_Z_PIN, INPUT_PULLUP); // Z pin

  // Attach the interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), doEncoderB, CHANGE);

  // Initialize Z pin state
  lastZState = digitalRead(ENCODER_Z_PIN);
}

void loop() {
  if (A_set && B_set) { // both have changed
    float angle;
    if(useRadians) {
      angle = (encoderPos / (float)(ppr*4)) * 2.0 * PI; // convert to radians
    } else {
      angle = (encoderPos / (float)(ppr*4)) * 360.0; // convert to degrees
    }
    Serial.println(angle);
    A_set = B_set = false; // reset the flags
  }

  // Check for Z pin state change
  int currentZState = digitalRead(ENCODER_Z_PIN);
  if (currentZState != lastZState) {
    // State has changed, meaning a full revolution has been completed
    lastZState = currentZState;
    if (currentZState == HIGH) { // adjust this based on your specific encoder Z-channel behavior
      encoderPos = 0;
    }
  }
}
