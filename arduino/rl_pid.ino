#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rl_pid_uros/srv/tminusp.h>

// Motor and Encoder Pins
#define ENCA 18
#define ENCB 19
#define PWM 14
#define IN1 13
#define IN2 12
#define STBY 27

// WiFi Credentials
const char *ssid = "your_ssid";
const char *password = "password";

// micro-ROS objects
rcl_node_t node;
rcl_service_t service;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

// PID and Motor Control Variables
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float kp = 1.0;
float kd = 0.01;
float ki = 0.0;
int target;
//bool useSineWaveTarget = true;

void initializeMicroRos();
void spinMicroRos();

// Service callback function
void handle_service(const void *req, void *res) {
  auto *request = (rl_pid_uros__srv__Tminusp_Request *)req;
  auto *response = (rl_pid_uros__srv__Tminusp_Response *)res;

  // Assign received PID values
  kp = request->kp;
  ki = request->ki;
  kd = request->kd;

  int pos = 0;
  noInterrupts();
  pos = posi;
  interrupts();

  // Send feedback
  response->tvp = pos;

  Serial.print("Received PID: kp=");
  Serial.print(kp);
  Serial.print(", ki=");
  Serial.print(ki);
  Serial.print(", kd=");
  Serial.println(kd);
  Serial.print("Sending TVP: ");
  Serial.println(response->tvp);
}

void setup() {
  Serial.begin(115200);

  // Pin Setup
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWM, 1000, 8);  // Attach PWM pin with frequency 1 kHz and 8-bit resolution

  // Initialize PWM signal (write 0 as initial value)
  ledcWrite(PWM, 0);


  // WiFi Setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());

  // Initialize micro-ROS
  initializeMicroRos();
}

void loop() {
  spinMicroRos();
  target = 250 * sin(prevT / 1e6);
  motorControl();
}

void motorControl() {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  int pos = 0;
  noInterrupts();
  pos = posi;
  interrupts();

  int e = pos - target;
  float dedt = (e - eprev) / (deltaT);
  eintegral += e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;
  float pwr = fabs(u);
  pwr = constrain(pwr, 0, 255);

  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;


  Serial.print(pos);
  Serial.print(" ");
  Serial.println(target);
  delay(100);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2 ) {
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

void initializeMicroRos() {
  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)"your_pc_ip", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_controller", "", &support);

  // Service Initialization
  const rosidl_service_type_support_t *service_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(rl_pid_uros, srv, Tminusp);
  rclc_service_init_default(&service, &node, service_type_support, "/tune_pid");
  rl_pid_uros__srv__Tminusp_Request req;
  rl_pid_uros__srv__Tminusp_Response res;
  // Executor Setup
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_service(&executor, &service, &req, &res, handle_service);

  Serial.println("micro-ROS Ready");
}

void spinMicroRos() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}