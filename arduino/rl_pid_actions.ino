#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

// ROS 2 includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_action/rcl_action.h>
#include <rcl_action/types.h>
#include <rmw_microros/rmw_microros.h>
#include <action_msgs/msg/goal_status.h>
#include <rcl_action/goal_handle.h>

// Custom action interface includes
#include <rl_pid_uros/action/tune_pid.h>

// Constants and Pin Definitions
#define ENCA 18
#define ENCB 19
#define PWM 14
#define IN1 13
#define IN2 12
#define STBY 27

#define WIFI_TIMEOUT_MS 10000
#define WIFI_RECOVER_TIME_MS 5000
#define STACK_SIZE 4096
#define MAX_INTEGRAL 1000.0
#define DEADBAND 10
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define FEEDBACK_FREQUENCY_MS 100
#define MIN_DELTA_T 1e-6
#define MAX_RECONNECT_ATTEMPTS 5
#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 255

// WiFi Credentials
const char *ssid = "Home";
const char *password = "OpenDoor_55";
const char *agent_ip = "192.168.31.127";
const int agent_port = 8888;

// Synchronization primitives
static portMUX_TYPE positionMux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t pidMutex = NULL;

// Motor Control Structure
struct MotorControl {
  volatile int position;
  int target;
  float kp;
  float ki;
  float kd;
  float eprev;
  float eintegral;
  long prevT;
  bool updateInProgress;
  bool goalActive;
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
  .updateInProgress = false,
  .goalActive = false
};

// Target positions array
const int targetArray[] = { 100, 200, 300, 400, 500 };
const int targetArraySize = sizeof(targetArray) / sizeof(targetArray[0]);
volatile int targetIndex = 0;

// Global declarations
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_action_server_t action_server;
rclc_action_goal_handle_t goal_handle;

// Function declarations (including new action-related functions)
void initializePins();
bool initializeWiFi();
bool initializeMicroRos();
void spinMicroRos();
void motorControlTask(void *arg);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void IRAM_ATTR readEncoder();

// Action messages
static rl_pid_uros__action__TunePID_Goal goal_msg;
static rl_pid_uros__action__TunePID_Feedback feedback_msg;
static rl_pid_uros__action__TunePID_Result result_msg;



void setup() {
  Serial.begin(115200);

  // Initialize mutex
  pidMutex = xSemaphoreCreateMutex();
  if (pidMutex == NULL) {
    printf("Error creating mutex\n");
    return;
  }

  initializePins();

  if (!initializeWiFi()) {
    return;
  }

  if (!initializeMicroRos()) {
    return;
  }

  // Start motor control task
  BaseType_t xReturned = xTaskCreatePinnedToCore(
    motorControlTask,
    "Motor Control",
    STACK_SIZE,
    NULL,
    1,
    NULL,
    1);

  if (xReturned != pdPASS) {
    Serial.println("Failed to create motor control task");
    return;
  }
}

// running motor control task independent of loop function.

void motorControlTask(void *arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Run at 100Hz
  //static TickType_t lastFeedbackTime = 0;

  for (;;) {
    if (motor.goalActive) {
      // Compute feedback

      xSemaphoreTake(pidMutex, portMAX_DELAY);

      long currT = micros();
      float deltaT = ((float)(currT - motor.prevT)) / 1.0e6;

      // Protect against too small deltaT
      if (deltaT < MIN_DELTA_T) {
        deltaT = MIN_DELTA_T;
      }

      motor.prevT = currT;

      // Get current position and calculate error
      int currentPos = getPosition();
      feedback_msg.current_position = currentPos;
      feedback_msg.target_position = motor.target;
      //error calculation
      int e = motor.target - currentPos;
      feedback_msg.current_error = e;

      // PID calculations
      float dedt = (e - motor.eprev) / deltaT;
      motor.eintegral = motor.eintegral + e * deltaT;
      motor.eintegral = constrain(motor.eintegral, -MAX_INTEGRAL, MAX_INTEGRAL);

      float u = motor.kp * e + motor.kd * dedt + motor.ki * motor.eintegral;

      // Convert control signal to PWM
      float pwr = fabs(u);
      pwr = constrain(pwr, MIN_PWM_VALUE, MAX_PWM_VALUE);

      // Determine direction and apply deadband compensation
      int dir = 1;
      if (u < 0) {
        dir = -1;
      }

      if (pwr < DEADBAND) {
        pwr = 0;
      }

      setMotor(dir, pwr, PWM, IN1, IN2);
      motor.eprev = e;

      xSemaphoreGive(pidMutex);

      //TickType_t currentTime = xTaskGetTickCount();
      // Publish feedback
      rcl_ret_t ret = rclc_action_publish_feedback(&goal_handle, &feedback_msg);

      if (ret != RCL_RET_OK) {
        printf("Error publishing feedback: %d\n", ret);
      }

      // Check if goal is achieved
      if (abs(e) < DEADBAND) {
        // Wait for stability
        vTaskDelay(pdMS_TO_TICKS(100));

        // Recheck position to confirm stability
        currentPos = getPosition();
        if (abs(motor.target - currentPos) < DEADBAND) {
          motor.goalActive = false;
          result_msg.final_error = motor.target - currentPos;

          // Send result
          rcl_ret_t result_ret = rclc_action_send_result(
            &goal_handle,
            GOAL_STATE_SUCCEEDED,
            &result_msg);

          if (result_ret != RCL_RET_OK) {
            Serial.printf("Error sending result: %d\n", result_ret);
          }
        }
      }
    }
    // Ensure consistent timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loop() {
  static unsigned long lastWiFiCheck = 0;
  const unsigned long WIFI_CHECK_INTERVAL = 5000;  // Check every 5 seconds

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Periodic WiFi check
  unsigned long currentMillis = millis();
  if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = currentMillis;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost!");

      // Try to reconnect
      if (!initializeWiFi()) {
        // If reconnection fails after all attempts, restart
        Serial.println("Failed to reconnect, restarting...");
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECOVER_TIME_MS));
        ESP.restart();
      }
    }
  }

  delay(10);
}





// micro-ros action server intialization and definition
// Callback function prototypes with correct signatures

bool initializeMicroRos() {
  allocator = rcl_get_default_allocator();

  // Set up transport
  Serial.println("Setting up micro-ROS transport");
  set_microros_wifi_transports((char *)ssid, (char *)password,
                               (char *)agent_ip, agent_port);
  delay(2000);

  // Wait for agent connection
  bool connected = false;
  int attempts = 0;
  const int max_attempts = 5;

  while (!connected && attempts < max_attempts) {
    connected = rmw_uros_ping_agent(WIFI_TIMEOUT_MS, 1) == RMW_RET_OK;
    attempts++;
    if (!connected) {
      Serial.println("Could not connect to micro-ROS agent, retrying...");
      delay(1000);
    }
  }

  if (!connected) {
    Serial.println("Failed to connect to micro-ROS agent");
    return false;
  }

  // Initialize ROS 2 node
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }

  ret = rclc_node_init_default(&node, "tune_pid_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize node");
    return false;
  }


  // Initialize action server
  ret = rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(rl_pid_uros, TunePID),
    "tune_pid_action");

  if (ret != RCL_RET_OK) {
    printf("Error initializing action server: %d\n", ret);
    return false;
  }

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize executor");
    return false;
  }

  // Add action server to executor with correct parameters
  ret = rclc_executor_add_action_server(
    &executor,
    &action_server,
    1,  // handles number
    &goal_msg,
    sizeof(rl_pid_uros__action__TunePID_Goal),
    handle_goal,
    handle_cancel,
    NULL  // goal request size pointer (optional)
  );

  if (ret != RCL_RET_OK) {
    printf("Error adding action server to executor: %d\n", ret);
    return false;
  }
  Serial.println("micro-ROS initialized successfully");
  return true;
}

static rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, void *context) {
  const rl_pid_uros__action__TunePID_Goal *goal =
    (const rl_pid_uros__action__TunePID_Goal *)context;

  if (goal == NULL) {
    return RCL_RET_ERROR;
  }

  xSemaphoreTake(pidMutex, portMAX_DELAY);
  motor.kp = goal->kp;
  motor.ki = goal->ki;
  motor.kd = goal->kd;
  motor.target = goal->target_position;
  motor.goalActive = true;
  xSemaphoreGive(pidMutex);

  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool handle_cancel(rclc_action_goal_handle_t *goal_handle, void *context) {
  (void)goal_handle;
  (void)context;

  xSemaphoreTake(pidMutex, portMAX_DELAY);
  motor.goalActive = false;
  xSemaphoreGive(pidMutex);

  return true;
}

//motor control, wifi and pin definitions

void initializePins() {
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttach(PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PWM, 0);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

bool initializeWiFi() {
  unsigned long startAttemptTime = millis();
  int attempts = 0;

  while (attempts < MAX_RECONNECT_ATTEMPTS) {
    WiFi.begin(ssid, password);

    unsigned long attemptStartTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - attemptStartTime < WIFI_TIMEOUT_MS) {
      delay(100);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      return true;
    }

    attempts++;
    Serial.println("\nWiFi connection attempt failed");
    WiFi.disconnect();
    delay(1000);
  }

  Serial.println("WiFi connection FAILED after all attempts");
  return false;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  digitalWrite(STBY, HIGH);
  ledcWrite(pwm, pwmVal);
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == 1 ? LOW : HIGH);
}

int getPosition() {
  int pos;
  portENTER_CRITICAL(&positionMux);
  pos = motor.position;
  portEXIT_CRITICAL(&positionMux);
  return pos;
}

// Interrupt handler for encoder
void IRAM_ATTR readEncoder() {
  int b = digitalRead(ENCB);
  portENTER_CRITICAL_ISR(&positionMux);
  if (b > 0) {
    motor.position++;
  } else {
    motor.position--;
  }
  portEXIT_CRITICAL_ISR(&positionMux);
}
