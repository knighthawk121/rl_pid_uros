#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

// ROS 2 includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Custom action interface includes
#include <rl_pid_uros/action/tune_pid.h>  // You'll need to create this custom action interface
#include <rmw_microros/rmw_microros.h>

// Constants and Pin Definitions (unchanged)
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
#define FEEDBACK_FREQUENCY_MS 100  // Send feedback every 100ms

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
  bool goalActive;  // New field to track if a tuning goal is active
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

// micro-ROS objects
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Action server objects
rcl_action_server_t action_server;
rcl_action_goal_handle_t goal_handle;
bool goal_handle_valid = false;

// Function declarations (including new action-related functions)
void initializePins();
bool initializeWiFi();
bool initializeMicroRos();
void spinMicroRos();
void motorControlTask(void *parameter);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void IRAM_ATTR readEncoder();

// Action server callback functions
void handle_goal(rcl_action_goal_handle_t *goal_handle, void *context);
void handle_cancel(rcl_action_goal_handle_t *goal_handle, void *context);
void handle_accepted(rcl_action_goal_handle_t *goal_handle, void *context);

// Helper function for sending feedback
void send_feedback(rcl_action_goal_handle_t *goal_handle) {
  if (!goal_handle_valid) return;
  
  static rl_pid_uros__action__TunePid_Feedback feedback;
  
  // Update feedback message with current error
  int current_position = getPosition();
  feedback.current_error = motor.target - current_position;
  feedback.current_position = current_position;
  feedback.target_position = motor.target;
  
  // Send feedback
  rcl_ret_t ret = rcl_action_publish_feedback(goal_handle, &feedback);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to publish feedback");
  }
}

// Goal handling callback
void handle_goal(rcl_action_goal_handle_t *goal_handle, void *context) {
  (void)context;
  
  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
    const rl_pid_uros__action__TunePid_Goal *goal =
      rcl_action_get_goal_from_handle(goal_handle);
      
    if (goal->kp >= 0 && goal->ki >= 0 && goal->kd >= 0) {
      // Accept the goal and update PID parameters
      motor.kp = goal->kp;
      motor.ki = goal->ki;
      motor.kd = goal->kd;
      motor.goalActive = true;
      goal_handle_valid = true;
      
      // Reset integral term when new parameters are set
      motor.eintegral = 0;
      
      Serial.printf("New PID parameters accepted - kp: %.2f, ki: %.2f, kd: %.2f\n",
                    motor.kp, motor.ki, motor.kd);
                    
      // Accept the goal
      rcl_action_update_goal_state(goal_handle, RCL_ACTION_GOAL_STATE_ACCEPTED);
    } else {
      // Reject invalid parameters
      rcl_action_update_goal_state(goal_handle, RCL_ACTION_GOAL_STATE_REJECTED);
    }
    
    xSemaphoreGive(pidMutex);
  }
}

// Cancel callback
void handle_cancel(rcl_action_goal_handle_t *goal_handle, void *context) {
  (void)context;
  
  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
    motor.goalActive = false;
    goal_handle_valid = false;
    rcl_action_update_goal_state(goal_handle, RCL_ACTION_GOAL_STATE_CANCELED);
    xSemaphoreGive(pidMutex);
  }
}

// Modified motor control task
void motorControlTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Run at 100Hz
    static TickType_t lastFeedbackTime = 0;
    
    while (true) {
      if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
        long currT = micros();
        float deltaT = ((float)(currT - motor.prevT)) / 1.0e6;
        motor.prevT = currT;

        int pos = getPosition();
        int e = motor.target - pos;  // Error is now target - position

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

        // Send feedback if enough time has passed and we have an active goal
        TickType_t currentTime = xTaskGetTickCount();
        if (motor.goalActive && 
            (currentTime - lastFeedbackTime) >= pdMS_TO_TICKS(FEEDBACK_FREQUENCY_MS)) {
          send_feedback(&goal_handle);
          lastFeedbackTime = currentTime;
        }

        // Check if we've reached the target
        if (abs(e) < DEADBAND) {
          if (motor.goalActive) {
            // Send result and mark goal as succeeded
            static rl_pid_uros__action__TunePid_Result result;
            result.final_error = e;
            rcl_action_send_result(&action_server, &goal_handle, &result);
            rcl_action_update_goal_state(&goal_handle, RCL_ACTION_GOAL_STATE_SUCCEEDED);
            motor.goalActive = false;
          }
          
          // Move to next target position
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

bool initializeMicroRos() {
    // Set up the WiFi transport
    set_microros_wifi_transports((char *)ssid, (char *)password,
                                 (char *)agent_ip, agent_port);
    delay(2000);

    // Initialize allocator
    allocator = rcl_get_default_allocator();

    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) return false;

    // Initialize node
    ret = rclc_node_init_default(&node, "esp32_pid_controller", "", &support);
    if (ret != RCL_RET_OK) return false;

    // Initialize action server
    ret = rclc_action_server_init_default(
      &action_server,
      &node,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(rl_pid_uros, action, TunePid),
      "tune_pid",
      &goal_handle,
      handle_goal,
      handle_cancel,
      handle_accepted);
    
    if (ret != RCL_RET_OK) return false;

    // Initialize executor
    ret = rclc_executor_init(&executor, &support.context, 
                            RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);
    if (ret != RCL_RET_OK) return false;

    // Add action server to executor
    ret = rclc_executor_add_action_server(&executor, &action_server, 
                                         &goal_handle, sizeof(goal_handle));
    if (ret != RCL_RET_OK) return false;

    Serial.println("micro-ROS initialized successfully");
    return true;
}

// Rest of the code (setup, loop, etc.) remains largely the same
// Just replace the service spinning with action server spinning in the loop

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost!");
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECOVER_TIME_MS));
        ESP.restart();
    }
}