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
const char *ssid = "Home";
const char *password = "OpenDoor_55";

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
int target = 0;
volatile bool pidUpdateInProgress = false;

void initializeMicroRos();
void spinMicroRos();
void motorControlTask(void *parameter);
void handle_service(const void *req, void *res);
void readEncoder();
float calculate_tvp(float kp, float ki, float kd);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void handle_service(const void *req, void *res) {
    if (pidUpdateInProgress) return;

    auto *request = (rl_pid_uros__srv__Tminusp_Request *)req;
    auto *response = (rl_pid_uros__srv__Tminusp_Response *)res;

    kp = request->kp;
    ki = request->ki;
    kd = request->kd;

    noInterrupts();
    int pos = posi;
    interrupts();

    response->tvp = target - pos;  // Error

    Serial.print("Received PID: kp=");
    Serial.print(kp);
    Serial.print(", ki=");
    Serial.print(ki);
    Serial.print(", kd=");
    Serial.println(kd);
    Serial.print("Sending TVP (error): ");
    Serial.println(response->tvp);

    pidUpdateInProgress = true;
    delay(500);  // Simulate processing time
    pidUpdateInProgress = false;
}

void setup() {
    Serial.begin(115200);

    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    ledcAttach(PWM, 1000, 8);
    ledcWrite(PWM, 0);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());

    initializeMicroRos();
    xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 2048, NULL, 1, NULL, 0);
}

void loop() {
    spinMicroRos();
}

void motorControlTask(void *parameter) {
    static int prev_posi = 0;
    while (true) {
        target = 250 * sin(millis() / 1000.0);

        if (!pidUpdateInProgress) {
            long currT = micros();
            float deltaT = ((float)(currT - prevT)) / 1.0e6;
            prevT = currT;

            noInterrupts();
            int pos = posi;
            interrupts();

            int e = target - pos;
            float dedt = (e - eprev) / deltaT;
            eintegral = constrain(eintegral + e * deltaT, -1000, 1000);

            float u = kp * e + kd * dedt + ki * eintegral;
            float pwr = constrain(fabs(u), 0, 255);

            int dir = (u < 0) ? -1 : 1;
            setMotor(dir, pwr, PWM, IN1, IN2);

            eprev = e;

            Serial.print("Pos: ");
            Serial.print(pos);
            Serial.print(" Target: ");
            Serial.println(target);
        }
        delay(100);
    }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    digitalWrite(STBY, HIGH);
    ledcWrite(pwm, pwmVal);
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
}

void readEncoder() {
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 5) {
        int b = digitalRead(ENCB);
        posi += (b > 0) ? 1 : -1;
        last_interrupt_time = interrupt_time;
    }
}

void initializeMicroRos() {
    set_microros_wifi_transports((char *)ssid, (char *)password, (char *)"192.168.31.127", 8888);
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_motor_controller", "", &support);

    const rosidl_service_type_support_t *service_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(rl_pid_uros, srv, Tminusp);
    rclc_service_init_default(&service, &node, service_type_support, "/tune_pid");

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_service(&executor, &service, NULL, NULL, handle_service);

    Serial.println("micro-ROS Ready");
}

void spinMicroRos() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
