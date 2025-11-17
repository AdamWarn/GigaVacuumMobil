/*
 * GigaVacuumMobil - Arduino Encoder & Motor Driver
 * 
 * This firmware runs on the Arduino connected to the Raspberry Pi.
 * It handles:
 * - Reading encoder ticks from two DC motors
 * - Sending PWM signals to motor driver board
 * - Serial communication with ROS 2 hardware interface
 * 
 * Author: AdamWarn
 * Date: 2025-11-17
 */

// ==================== PIN DEFINITIONS ====================
// IMPORTANT: Update these to match your actual wiring!

// Left Motor Encoder Pins
#define LEFT_ENCODER_A 2   // Use interrupt-capable pins (2 or 3 on most Arduinos)
#define LEFT_ENCODER_B 4

// Right Motor Encoder Pins
#define RIGHT_ENCODER_A 3  // Use interrupt-capable pins
#define RIGHT_ENCODER_B 5

// Motor Driver PWM Pins
#define LEFT_MOTOR_PWM 6
#define LEFT_MOTOR_DIR 7

#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIR 8

// ==================== GLOBAL VARIABLES ====================
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// Buffer for serial communication
char serial_buffer[64];
int buffer_index = 0;

// ==================== SETUP ====================
void setup() {
  // Initialize serial communication
  Serial.begin(57600);
  
  // Configure encoder pins as inputs
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // Configure motor driver pins as outputs
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  
  // Attach interrupts for encoders
  // Note: On most Arduino boards, only pins 2 and 3 support interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
  
  // Ensure motors start stopped
  stopMotors();
  
  // Signal that Arduino is ready
  Serial.println("Arduino ready");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check for incoming serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      // End of command - process it
      serial_buffer[buffer_index] = '\0';
      processCommand(serial_buffer);
      buffer_index = 0;
    } else {
      // Add character to buffer
      if (buffer_index < sizeof(serial_buffer) - 1) {
        serial_buffer[buffer_index++] = c;
      }
    }
  }
}

// ==================== COMMAND PROCESSING ====================
void processCommand(char* cmd) {
  if (cmd[0] == 'e') {
    // Encoder request: 'e'
    // Response format: 'e{left_count},{right_count}\n'
    sendEncoderData();
  } 
  else if (cmd[0] == 'p') {
    // PWM command: 'p{left_pwm},{right_pwm}'
    // Example: 'p100,-150' means left motor forward at 100, right motor backward at 150
    int left_pwm, right_pwm;
    if (sscanf(cmd, "p%d,%d", &left_pwm, &right_pwm) == 2) {
      setMotorPWM(left_pwm, right_pwm);
    }
  }
  else if (cmd[0] == 'r') {
    // Reset encoders: 'r'
    resetEncoders();
    Serial.println("Encoders reset");
  }
}

// ==================== ENCODER FUNCTIONS ====================
void sendEncoderData() {
  // Temporarily disable interrupts to safely read encoder counts
  noInterrupts();
  long left_count = left_encoder_count;
  long right_count = right_encoder_count;
  interrupts();
  
  // Send data in format: e{left},{right}\n
  Serial.print("e");
  Serial.print(left_count);
  Serial.print(",");
  Serial.println(right_count);
}

void resetEncoders() {
  noInterrupts();
  left_encoder_count = 0;
  right_encoder_count = 0;
  interrupts();
}

// Left encoder interrupt service routine
void leftEncoderISR() {
  // Read both encoder channels
  bool A = digitalRead(LEFT_ENCODER_A);
  bool B = digitalRead(LEFT_ENCODER_B);
  
  // Determine direction based on A and B signals
  // This is a simple quadrature decoder
  if (A == B) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

// Right encoder interrupt service routine
void rightEncoderISR() {
  bool A = digitalRead(RIGHT_ENCODER_A);
  bool B = digitalRead(RIGHT_ENCODER_B);
  
  if (A == B) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
void setMotorPWM(int left_pwm, int right_pwm) {
  // Clamp PWM values to valid range
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);
  
  // Set left motor
  if (left_pwm >= 0) {
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
    analogWrite(LEFT_MOTOR_PWM, left_pwm);
  } else {
    digitalWrite(LEFT_MOTOR_DIR, LOW);
    analogWrite(LEFT_MOTOR_PWM, -left_pwm);
  }
  
  // Set right motor
  if (right_pwm >= 0) {
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, right_pwm);
  } else {
    digitalWrite(RIGHT_MOTOR_DIR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, -right_pwm);
  }
}

void stopMotors() {
  setMotorPWM(0, 0);
}

// ==================== DEBUGGING (Optional) ====================
// Uncomment this function and call it in loop() to debug encoder readings
/*
void debugEncoders() {
  static unsigned long last_print = 0;
  unsigned long now = millis();
  
  if (now - last_print > 500) {  // Print every 500ms
    noInterrupts();
    long left = left_encoder_count;
    long right = right_encoder_count;
    interrupts();
    
    Serial.print("Left: ");
    Serial.print(left);
    Serial.print(" | Right: ");
    Serial.println(right);
    
    last_print = now;
  }
}
*/
