/*
 * motor_controller_ros_wifi.ino
 *
 * This program implements a motor control system on an ESP32
 * using PID control integrated with ROS 2 over WiFi.
 * The system provides real-time motor position control with feedback
 * from an encoder and
 * supports dynamic PID parameter updates through
 * a micro-ROS service interface.
 *
 * Features:
 * - WiFi-based communication
 * - micro-ROS integration with custom service (`/tune_pid`) for
 *   PID parameter tuning
 * - Motor position control using encoder feedback
 * - Multi-tasking with FreeRTOS
 * - Safety mechanisms, including anti-windup and deadband handling
 *
 * Author: Arvindh Ramesh
 *
 */

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rl_pid_uros/srv/tminusp.h>

// Constants and Pin Definitions
#define ENCA 18
#define ENCB 19
#define PWM 14
#define IN1 13
#define IN2 12
#define STBY 27

#define WIFI_TIMEOUT_MS 10000     // 10 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 5000 // Wait 5 seconds before retrying WiFi
#define STACK_SIZE 4096           // Increased stack size
#define MAX_INTEGRAL 1000.0       // Anti-windup limit
#define DEADBAND 10
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

// WiFi Credentials
const char *ssid = "Home";
const char *password = "OpenDoor_55";
const char *agent_ip = "192.168.31.127";
const int agent_port = 8888;

// Synchronization primitives
static portMUX_TYPE positionMux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t pidMutex = NULL;

// Motor Control Structure
struct MotorControl
{
  volatile int position;
  int target;
  float kp;
  float ki;
  float kd;
  float eprev;
  float eintegral;
  long prevT;
  bool updateInProgress;
};

// Global variables
MotorControl motor = {
    .position = 0,
    .target = 0,
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.01,
    .eprev = 0,
    .eintegral = 0,
    .prevT = 0,
    .updateInProgress = false};

// Target positions array
const int targetArray[] = {100, 200, 300, 400, 500};
const int targetArraySize = sizeof(targetArray) / sizeof(targetArray[0]);
volatile int targetIndex = 0;

// micro-ROS objects
rcl_node_t node;
rcl_service_t service;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Function declarations
void initializePins();
bool initializeWiFi();
bool initializeMicroRos();
void spinMicroRos();
void motorControlTask(void *parameter);
void handle_service(const void *req, void *res);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void updatePID();
void IRAM_ATTR readEncoder();



/*================================setup, loop and motor control implementation==================================*/


void setup()
{
  Serial.begin(115200);

  // Initializing mutex
  pidMutex = xSemaphoreCreateMutex();
  if (pidMutex == NULL)
  {
    Serial.println("Failed to create mutex");
    return;
  }

  initializePins();

  if (!initializeWiFi())
  {
    return;
  }

  if (!initializeMicroRos())
  {
    return;
  }

  // Initializing first target
  motor.target = targetArray[targetIndex];

  // Creating motor control task
  BaseType_t xReturned = xTaskCreatePinnedToCore(
      motorControlTask,
      "MotorControl",
      STACK_SIZE,
      NULL,
      2,
      NULL,
      0);

  if (xReturned != pdPASS)
  {
    Serial.println("Failed to create motor control task");
    return;
  }
}

void motorControlTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  int stuckCounter = 0;
  int lastPosition = 0;

  while (true)
  {
    int currentPosition = getPosition();
    lastPosition = currentPosition;

    if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
    {
      long currT = micros();
      float deltaT = ((float)(currT - motor.prevT)) / 1.0e6;
      motor.prevT = currT;

      int pos = getPosition();
      int e = pos - motor.target;

      // PID calculations
      float dedt = (e - motor.eprev) / deltaT;
      motor.eintegral = motor.eintegral + e * deltaT;
      motor.eintegral = constrain(motor.eintegral, -MAX_INTEGRAL, MAX_INTEGRAL);

      float u = motor.kp * e + motor.kd * dedt + motor.ki * motor.eintegral;
      float pwr = fabs(u);
      pwr = constrain(pwr, 0, 255);

      int dir = (u < 0) ? -1 : 1;
      setMotor(dir, pwr, PWM, IN1, IN2);

      motor.eprev = e;

      Serial.printf("Position: %d, Target: %d, Error: %d\n",
                    pos, motor.target, e);

      if (abs(e) < DEADBAND)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        targetIndex = (targetIndex + 1) % targetArraySize;
        motor.target = targetArray[targetIndex];
        motor.eintegral = 0;
      }

      xSemaphoreGive(pidMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loop()
{
  spinMicroRos();

  // Monitoring WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi connection lost!");
    vTaskDelay(pdMS_TO_TICKS(WIFI_RECOVER_TIME_MS));
    ESP.restart(); // Reset the ESP32 to recover WiFi
  }
}


/*===============================================micro-ros intialization and definition=========================================*/

bool initializeMicroRos()
{
  // setting up the WiFi transport
  set_microros_wifi_transports((char *)ssid, (char *)password,
                               (char *)agent_ip, agent_port);
  delay(2000);

  // the default allocator
  rcl_allocator_t local_allocator = rcl_get_default_allocator();
  allocator = local_allocator;

  // Initialization of the support structure with proper error checking
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to initialize support");
    return false;
  }

  // Initialization of the node
  ret = rclc_node_init_default(&node, "esp32_motor_controller", "", &support);
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to initialize node");
    return false;
  }

  // service type support
  const rosidl_service_type_support_t *service_type_support =
      ROSIDL_GET_SRV_TYPE_SUPPORT(rl_pid_uros, srv, Tminusp);

  // Initialization of the service
  ret = rclc_service_init_default(&service, &node, service_type_support, "/tune_pid");
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to initialize service");
    return false;
  }

  // Initialization of the executor
  executor = rclc_executor_get_zero_initialized_executor();
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to initialize executor");
    return false;
  }

  // Creating temporary request and response objects for the service
  rl_pid_uros__srv__Tminusp_Request req;
  rl_pid_uros__srv__Tminusp_Response res;

  // Add the service to the executor
  ret = rclc_executor_add_service(&executor, &service, &req, &res, handle_service);
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to add service to executor");
    return false;
  }

  Serial.println("micro-ROS initialized successfully");
  return true;
}

// Service callback
void handle_service(const void *req, void *res)
{
  Serial.println("Service request received at time: " + String(millis()));
  Serial.println("Received service request");
  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
  {
    auto *request = (rl_pid_uros__srv__Tminusp_Request *)req;
    auto *response = (rl_pid_uros__srv__Tminusp_Response *)res;

    if (request->kp >= 0 && request->ki >= 0 && request->kd >= 0)
    {
      motor.kp = request->kp;
      motor.ki = request->ki;
      motor.kd = request->kd;
    }
    response->tvp = getPosition();

    Serial.printf("PID Update - kp: %.2f, ki: %.2f, kd: %.2f\n",
                  motor.kp, motor.ki, motor.kd);
    Serial.printf("Current Position: %d\n", response->tvp);

    xSemaphoreGive(pidMutex);
  }
  Serial.println("Service request completed at time: " + String(millis()));
}

void spinMicroRos()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}




/*======================================motor control, wifi and pin definitions============================================*/


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  digitalWrite(STBY, HIGH);
  ledcWrite(pwm, pwmVal);
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == 1 ? LOW : HIGH);
}

void initializePins()
{
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttach(PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PWM, 0);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

bool initializeWiFi()
{
  unsigned long startAttemptTime = millis();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
  {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi connection FAILED");
    return false;
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}


// Safe position reading
int getPosition()
{
  int pos;
  portENTER_CRITICAL(&positionMux);
  pos = motor.position;
  portEXIT_CRITICAL(&positionMux);
  return pos;
}

// Interrupt handler for encoder
void IRAM_ATTR readEncoder()
{
  int b = digitalRead(ENCB);
  portENTER_CRITICAL_ISR(&positionMux);
  if (b > 0)
  {
    motor.position++;
  }
  else
  {
    motor.position--;
  }
  portEXIT_CRITICAL_ISR(&positionMux);
}