/*
 * motor_test_esp32.ino
 *
 * This program controls a motor using an ESP32 microcontroller. 
 * It includes functionality for reading encoder feedback, 
 * controlling motor speed and direction with PWM signals, 
 * and implementing motor standby. 
 * The code is set up for high-speed communication and precise motor control.
 *
 * Components:
 * - Encoder feedback (pins ENCA, ENCB)
 * - Motor speed control (PWM pin)
 * - Motor direction control (IN1, IN2)
 * - Standby mode (STBY pin)
 *
 * Author: Arvindh Ramesh
 *
 */


#define ENCA 18  // Encoder A pin
#define ENCB 19  // Encoder B pin
#define PWM 14   // PWM pin for motor speed control
#define IN1 13   // Motor direction pin 1
#define IN2 12   // Motor direction pin 2
#define STBY 27

volatile int posi = 0;  // Encoder position variable
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(115200);  // Increased baud rate for faster communication

  // Encoder setup
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Motor control pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Setup PWM for ESP32 (using the updated ledc API)
  ledcAttach(PWM, 1000, 8);  // Attach PWM pin with frequency 1 kHz and 8-bit resolution

  // Initialize PWM signal (write 0 as initial value)
  ledcWrite(PWM, 0);

  Serial.println("Motor Test");
}

void loop() {
  // Example of setting target position as a sine wave (for testing)
  int target = 250 * sin(prevT / 1e6);

  // PID constants
  float kp = 1.0;   // Proportional constant
  float kd = 0.01;  // Derivative constant
  float ki = 0.0;   // Integral constant (start small)


  // Time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int pos = 0;
  noInterrupts();
  pos = posi;
  interrupts();

  // Error calculation
  int e = pos - target;

  // Derivative calculation
  float dedt = (e - eprev) / (deltaT);

  // Integral calculation
  eintegral = eintegral + e * deltaT;

  // Control signal (PID)
  float u = kp * e + kd * dedt + ki * eintegral;

  // Motor power (limited to 255)
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // Motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // Signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // Store previous error
  eprev = e;

  // Plot position vs target on the Serial Plotter
  Serial.print(pos);  // Position (X-axis)
  Serial.print(" ");
  Serial.println(target);  // Target (Y-axis)

  delay(100);  // Update rate
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  digitalWrite(STBY, HIGH);
  ledcWrite(pwm, pwmVal);  // Set the PWM duty cycle (0-255)
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}