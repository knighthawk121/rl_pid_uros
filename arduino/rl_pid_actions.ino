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

// WiFi Credentials
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASS";
const char *agent_ip = "192.xxx.xx.xxx";
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
void spinMicroRos();
void motorControlTask(void *arg);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void IRAM_ATTR readEncoder();

// Callback function prototypes with correct signatures
//rcl_ret_t accept_goal(const rcl_action_goal_handle_t *goal_handle);
//rcl_ret_t handle_goal(const rcl_action_goal_handle_t *goal_handle, const void *goal);
//bool handle_cancel(const rcl_action_goal_handle_t *goal_handle, void *context);

// Action messages
static bool goal_msg_initialized = false;
// First, update global declarations
static rl_pid_uros__action__TunePID_Goal goal_msg = {
  .kp = 0.0d,
  .ki = 0.0d,
  .kd = 0.0d,
  .target_position = 0
};

static rl_pid_uros__action__TunePID_Feedback feedback_msg = {
  .current_error = 0,
  .current_position = 0,
  .target_position = 0

};

static rl_pid_uros__action__TunePID_Result result_msg = {
  .final_error = 0
};


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

  // In your setup() function:
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 10000,                                           // 10 seconds
    .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,  // Bitmask of all cores
    .trigger_panic = true
  };

  /* esp_task_wdt_init(&twdt_config);
  if (xReturned != pdPASS) {
    Serial.println("Failed to create motor control task");
    return;
  }*/
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

          // Release mutex before communication
          xSemaphoreGive(pidMutex);

          // Send feedback
          rcl_ret_t ret = rclc_action_publish_feedback(&goal_handle, &feedback_msg);
          if (ret != RCL_RET_OK) {
            Serial.printf("Feedback error: %d\n", ret);
          }

          // Check for goal completion
          if (abs(e) < DEADBAND) {
            static int stableCount = 0;
            stableCount++;

            if (stableCount >= 5) {  // Reduced stability count
              motor.goalActive = false;
              result_msg.final_error = e;

              ret = rclc_action_send_result(&goal_handle, GOAL_STATE_SUCCEEDED, &result_msg);
              if (ret != RCL_RET_OK) {
                Serial.printf("Result error: %d\n", ret);
              }

              stableCount = 0;
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





// micro-ros action server intialization and definition
// Callback function prototypes with correct signatures

bool initializeMicroRos() {
  allocator = rcl_get_default_allocator();

  Serial.println("Setting up micro-ROS transport");
  set_microros_wifi_transports((char *)ssid, (char *)password,
                               (char *)agent_ip, agent_port);
  delay(2000);

  // Wait for agent connection with improved error handling
  bool connected = false;
  int attempts = 0;
  const int max_attempts = 5;

  while (!connected && attempts < max_attempts) {
    Serial.print("Attempting to connect to agent (attempt ");
    Serial.print(attempts + 1);
    Serial.println("/5)");

    connected = rmw_uros_ping_agent(WIFI_TIMEOUT_MS, 1) == RMW_RET_OK;
    attempts++;

    if (!connected && attempts < max_attempts) {
      Serial.println("Could not connect to micro-ROS agent, retrying...");
      delay(1000);
    }
  }

  if (!connected) {
    Serial.println("Failed to connect to micro-ROS agent");
    return false;
  }

  Serial.println("Connected to micro-ROS agent");

  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to initialize support: %d\n", ret);
    return false;
  }

  ret = rclc_node_init_default(&node, "tune_pid_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to initialize node: %d\n", ret);
    return false;
  }
  Serial.println("Initializing action server...");

  // Initialize the action server
  ret = rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(rl_pid_uros, TunePID),
    "tune_pid_action");

  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to initialize action server: %d\n", ret);
    return false;
  }
  // Initialize executor with sufficient handles
  executor = rclc_executor_get_zero_initialized_executor();
  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to initialize executor: %d\n", ret);
    return false;
  }
  // Add action server to executor with proper context
  ret = rclc_executor_add_action_server(
    &executor,
    &action_server,
    1,
    &goal_msg,
    sizeof(&goal_msg),
    handle_goal,
    handle_cancel,
    &goal_msg);

  if (ret != RCL_RET_OK) {
    Serial.printf("Error adding action server to executor: %d\n", ret);
    return false;
  }
  Serial.println("Action server initialized");
  Serial.println("micro-ROS initialized successfully");
  return true;
}

/*static rcl_ret_t accept_goal(const rcl_action_goal_handle_t *goal_handle){
  if (!rcl_action_goal_handle_is_active(goal_handle))
  {
    return RCL_RET_ACTION_GOAL_ACCEPTED;
  }
}*/

// Keep the better implementation of handle_goal and remove the duplicate
static rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, auto context) {
  Serial.println("=== Goal Handler Entry ===");

  if (goal_handle == NULL) {
    Serial.println("Error: Goal handle is NULL");
    return RCL_RET_ERROR;
  }
  auto goal = (rl_pid_uros__action__TunePID_Goal *)context;

  Serial.println("=== Raw Goal Values ===");
  Serial.print("KP raw: ");
  Serial.println(goal->kp, 2);
  Serial.print("KI raw: ");
  Serial.println(goal->ki, 2);
  Serial.print("KD raw: ");
  Serial.println(goal->kd, 2);
  Serial.print("Target raw: ");
  Serial.println(goal->target_position);

  // Validate goal parameters
  if (isnan(goal->kp) || isnan(goal->ki) || isnan(goal->kd)) {
    Serial.println("Error: Invalid PID parameters (NaN detected)");
    return RCL_RET_ERROR;
  }

  // Add range checking
  const float MAX_PID_VALUE = 10.0;
  const int MAX_TARGET_VALUE = 360;
  if ((goal->kp != 0.0d && fabs(goal->kp) > MAX_PID_VALUE) || (goal->ki != 0.0d && fabs(goal->ki) > MAX_PID_VALUE) || (goal->kd != 0.0d && fabs(goal->kd) > MAX_PID_VALUE) || (goal->target_position >=0 && fabs(goal->target_position) > MAX_TARGET_VALUE)) {
    Serial.println("Error: Non-zero PID parameters out of valid range");
    return RCL_RET_ERROR;
  }

  // Debug prints with safe formatting
  Serial.println("=== Goal Content ===");
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "KP: %.6f", goal->kp);
  Serial.println(buffer);
  snprintf(buffer, sizeof(buffer), "KI: %.6f", goal->ki);
  Serial.println(buffer);
  snprintf(buffer, sizeof(buffer), "KD: %.6f", goal->kd);
  Serial.println(buffer);
  snprintf(buffer, sizeof(buffer), "Target Position: %d", goal->target_position);
  Serial.println(buffer);

  // Take mutex for motor updates
  if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("Error: Could not obtain mutex lock");
    return RCL_RET_ERROR;
  }

  // Update motor parameters with validated values
  motor.kp = goal->kp;
  motor.ki = goal->ki;
  motor.kd = goal->kd;
  motor.target = goal->target_position;
  motor.eprev = 0;
  motor.eintegral = 0;
  motor.goalActive = true;

  xSemaphoreGive(pidMutex);
  Serial.println("=== Goal Accepted ===");

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
