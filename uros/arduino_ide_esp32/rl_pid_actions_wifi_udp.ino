/*
 * rl_pid_actions_wifi_udp.ino
 *
 * This program integrates PID control with Reinforcement Learning (RL)
 * using a micro-ROS framework over WiFi (UDP protocol).
 * It enables real-time tuning of PID parameters through
 * custom ROS 2 action interfaces,
 * supporting wireless communication with an ESP32 microcontroller.
 *
 * Features:
 * - WiFi-based UDP communication
 * - ROS 2 custom action integration
 * - PID tuning with RL support
 *
 * Author: Arvindh Ramesh
 *
 */
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <esp_task_wdt.h>
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
#include <rl_pid_uros/action/tune_pi_dmin.h>

// Constants and Pin Definitions
#define ENCA 18
#define ENCB 19
#define PWM 14
#define IN1 13
#define IN2 12
#define STBY 27

#define WIFI_TIMEOUT_MS 10000
#define WIFI_RECOVER_TIME_MS 5000
#define MOTOR_TASK_PRIORITY 2  // Increased priority
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define FEEDBACK_FREQUENCY_MS 100
#define MIN_DELTA_T 1e-6
#define MAX_RECONNECT_ATTEMPTS 5
#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 255
#define STACK_SIZE (8192 * 2)  // Increased stack size
#define MIN_DELTA_T 0.001      // 1ms minimum time delta
#define EPSILON 1.0e-6
#define DEADBAND 5          // Reduced deadband
#define MAX_INTEGRAL 100.0  // Red
#define VELOCITY_THRESHOLD 1.0f
#define MAX_STALL_COUNT 5
#define ERROR_STABILITY_THRESHOLD 2.0f
#define STABLE_TIME_THRESHOLD 500  // 500ms
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
// WiFi Credentials
const char *ssid = "Your_ssid";
const char *password = "your_password";
const char *agent_ip = "your_local_pc_ip";
const int agent_port = 8888;

// Synchronization primitives
static portMUX_TYPE positionMux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t pidMutex = NULL;

// Motor Control Structure
struct MotorControl {
  volatile int position;
  long long target;
  double kp;
  double ki;
  double kd;
  float eprev;
  float eintegral;
  long prevT;
  bool updateInProgress;
  bool goalActive;
  float velocity;
  int lastPosition;
  unsigned long lastVelocityTime;
  int stallCount;
  unsigned long stableStartTime;
  bool isStable;
  bool isStopped;
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
void motorControlTask(void *arg);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void IRAM_ATTR readEncoder();

// Action messages
static bool goal_msg_initialized = false;
// First, update global declarations
static rl_pid_uros__action__TunePIDmin_Goal goal_msg = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .target_position = 0
};

static rl_pid_uros__action__TunePIDmin_Feedback feedback_msg = {
  .current_error = 0,
  .current_position = 0,
  .target_position = 0,
  .motor_stopped = false,
  .error_stable = false,
  .velocity = 0.0f

};

static rl_pid_uros__action__TunePIDmin_Result result_msg = {
  .final_error = 0
};

/*================================setup, loop and motor control implementation==================================*/
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

  // In setup(), update task creation:
  BaseType_t xReturned = xTaskCreatePinnedToCore(
    motorControlTask,
    "Motor Control",
    STACK_SIZE,
    NULL,
    MOTOR_TASK_PRIORITY,
    NULL,
    1);  // Run on core 1

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 10000,                                           // 10 seconds
    .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,  // Bitmask of all cores
    .trigger_panic = true
  };
}

void updateMotorStatus(MotorControl *motor) {
  unsigned long currentTime = millis();
  float deltaT = (currentTime - motor->lastVelocityTime) / 1000.0f;  // Convert to seconds

  if (deltaT > 0) {
    int currentPos = getPosition();
    motor->velocity = (currentPos - motor->lastPosition) / deltaT;
    motor->lastPosition = currentPos;
    motor->lastVelocityTime = currentTime;

    // Update stall detection
    if (abs(motor->velocity) < VELOCITY_THRESHOLD) {
      motor->stallCount++;
      if (motor->stallCount >= MAX_STALL_COUNT) {
        motor->isStopped = true;
      }
    } else {
      motor->stallCount = 0;
      motor->isStopped = false;
    }

    // Update error stability
    float currentError = motor->target - currentPos;
    if (abs(currentError) < ERROR_STABILITY_THRESHOLD) {
      if (!motor->isStable) {
        motor->stableStartTime = currentTime;
      }
      if (currentTime - motor->stableStartTime >= STABLE_TIME_THRESHOLD) {
        motor->isStable = true;
      }
    } else {
      motor->isStable = false;
    }
  }
}

// running motor control task independent of loop function.
void motorControlTask(void *arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // Reduced to 50Hz to lower system load

  while (1) {
    if (motor.goalActive) {
      // Add delay at the start to prevent watchdog triggers
      vTaskDelay(pdMS_TO_TICKS(1));

      if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Get current time and position
        long currT = micros();
        double deltaT = ((float)(currT - motor.prevT)) / 1.0e6;
        if (deltaT <= 0) {
          Serial.println("Error: Invalid time delta");
          return;
        }  // Ensure minimum deltaT
        deltaT = max(deltaT, MIN_DELTA_T);
        motor.prevT = currT;

        int currentPos = getPosition();
        int e = motor.target - currentPos;

        // PID calculations with safety limits
        float dedt = (e - motor.eprev) / (deltaT + EPSILON);
        motor.eintegral += e * deltaT;
        motor.eintegral = constrain(motor.eintegral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Calculate control signal
        float pidOutput = (motor.kp * e) + (motor.ki * motor.eintegral) + (motor.kd * dedt);

        // Convert to motor commands
        float pwr = constrain(fabs(pidOutput), 0, MAX_PWM_VALUE);
        int dir = (pidOutput >= 0) ? 1 : -1;

        // Apply deadband
        if (pwr < DEADBAND) {
          pwr = 0;
          dir = 0;
        }

        // Update motor
        setMotor(dir, pwr, PWM, IN1, IN2);
        motor.eprev = e;

        // Only send feedback occasionally to reduce system load
        static uint32_t lastFeedbackTime = 0;
        uint32_t currentTime = millis();

        if (currentTime - lastFeedbackTime >= 100) {  // 10Hz feedback rate
          lastFeedbackTime = currentTime;

          // Update feedback message
          feedback_msg.current_position = currentPos;
          feedback_msg.target_position = motor.target;
          feedback_msg.current_error = e;
          feedback_msg.motor_stopped = motor.isStopped;
          feedback_msg.error_stable = motor.isStable;
          feedback_msg.velocity = motor.velocity;

          // Release mutex before communication
          xSemaphoreGive(pidMutex);

          // Send feedback
          rcl_ret_t ret = rclc_action_publish_feedback(&goal_handle, &feedback_msg);
          if (ret != RCL_RET_OK) {
            Serial.printf("Feedback error: %d\n", ret);
          }

          // Check for goal completion
          if (motor.isStable || motor.isStopped) {
            motor.goalActive = false;
            result_msg.final_error = e;

            ret = rclc_action_send_result(&goal_handle,
                                          motor.isStable ? GOAL_STATE_SUCCEEDED : GOAL_STATE_CANCELED,
                                          &result_msg);
            if (ret != RCL_RET_OK) {
              Serial.printf("Result error: %d\n", ret);
            }
          }
        } else {
          xSemaphoreGive(pidMutex);
        }
      }
    } else {
      // If no active goal, just delay
      vTaskDelay(pdMS_TO_TICKS(10));
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
  // In loop()
  if (uxTaskGetStackHighWaterMark(NULL) < 500) {
    Serial.println("Warning: Low stack space");
  }
  delay(10);
}

/*===============================================micro-ros intialization and definition=========================================*/


bool initializeMicroRos() {
  Serial.println("\n=== Starting MicroROS Init ===");
  allocator = rcl_get_default_allocator();

  Serial.println("Setting up transport...");
  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)agent_ip, agent_port);
  delay(2000);

  Serial.println("Pinging agent...");
  if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    Serial.println("Agent ping failed");
    return false;
  }

  Serial.println("Initializing options...");
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  
  Serial.println("Initializing support...");
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  Serial.println("Initializing node...");
  RCCHECK(rclc_node_init_default(&node, "tune_pid_node", "", &support));
  
  Serial.println("Initializing action server...");
  Serial.printf("Node ptr: %p, Support ptr: %p\n", &node, &support);
  RCCHECK(rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(rl_pid_uros, TunePIDmin),
    "tune_pid_action"));

  Serial.println("Initializing executor...");
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  Serial.println("Adding action server to executor...");
  RCCHECK(rclc_executor_add_action_server(
    &executor,
    &action_server,
    1,
    &goal_msg,
    sizeof(rl_pid_uros__action__TunePIDmin_Goal),
    handle_goal,
    handle_cancel,
    &goal_msg));

  Serial.println("=== MicroROS Init Complete ===");
  return true;
}

// Goal handle callback
static rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, void *context) {
  Serial.println("=== Goal Handler Entry ===");

  if (goal_handle == NULL || context == NULL) {
    Serial.println("Error: Goal handle is NULL");
    return RCL_RET_ERROR;
  }

  rl_pid_uros__action__TunePIDmin_Goal *goal =
    (rl_pid_uros__action__TunePIDmin_Goal *)context;

  if (goal == NULL) {
    Serial.println("Error: Goal context is NULL");
    return RCL_RET_ERROR;
  }

  Serial.println("=== Raw Goal Values ===");
  Serial.print("KP raw: ");
  Serial.println(goal->kp, 6);
  Serial.print("KI raw: ");
  Serial.println(goal->ki, 6);
  Serial.print("KD raw: ");
  Serial.println(goal->kd, 6);
  Serial.print("Target raw: ");
  Serial.println(goal->target_position);

  // Add range checking
  const double MAX_PID_VALUE = 10.0;
  const long long MAX_TARGET_VALUE = 360;
  if ((fabs(goal->kp) > MAX_PID_VALUE) || (fabs(goal->ki) > MAX_PID_VALUE) || (fabs(goal->kd) > MAX_PID_VALUE) || (abs(goal->target_position) > MAX_TARGET_VALUE)) {
    Serial.println("Error: Parameters out of valid range");
    return RCL_RET_ERROR;
  }

  if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Initialize PID parameters
    motor.kp = goal->kp;
    motor.ki = goal->ki;
    motor.kd = goal->kd;
    motor.target = goal->target_position;
    motor.eprev = 0;
    motor.eintegral = 0;

    // Initialize status monitoring
    motor.velocity = 0;
    motor.lastPosition = getPosition();
    motor.lastVelocityTime = millis();
    motor.stallCount = 0;
    motor.stableStartTime = 0;
    motor.isStable = false;
    motor.isStopped = false;
    motor.goalActive = true;

    xSemaphoreGive(pidMutex);
    return RCL_RET_ACTION_GOAL_ACCEPTED;
  }

  return RCL_RET_ERROR;
}

static bool handle_cancel(rclc_action_goal_handle_t *goal_handle, void *context) {
  (void)goal_handle;
  (void)context;

  xSemaphoreTake(pidMutex, portMAX_DELAY);
  motor.goalActive = false;
  xSemaphoreGive(pidMutex);

  return true;
}

/*======================================motor control, wifi and pin definitions============================================*/

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
