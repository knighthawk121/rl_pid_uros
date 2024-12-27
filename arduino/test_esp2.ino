#include <WiFi.h>
#include <driver/ledc.h>  // Include the LEDC driver header

#define ENCA 18  // Encoder A pin
#define ENCB 19  // Encoder B pin
#define PWM 14   // PWM pin for motor speed control
#define IN1 13   // Motor direction pin 1
#define IN2 12   // Motor direction pin 2

const char* ssid = "your_ssid";
const char* password = "password";

volatile int posi = 0;  // Encoder position variable

void IRAM_ATTR readEncoder();

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Encoder setup
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Motor control pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Setup PWM for ESP32 using the new LEDC API
  ledcAttach(PWM, 1000, 8);  // Attach PWM pin with frequency 1 kHz and 8-bit resolution

  // Initialize PWM signal (write 0 as initial value)
  ledcWrite(PWM, 0);

  Serial.println("Setup complete");
}

void loop() {
  // Placeholder for main loop logic
  delay(1000);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
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

void IRAM_ATTR readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
