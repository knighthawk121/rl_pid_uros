/*
 * rl_pid_actions_serial.ino
 *
 * This program integrates PID control with Reinforcement Learning (RL)
 * using a micro-ROS framework over a serial connection.
 * It implements custom action interfaces for tuning PID parameters in real-time
 * and facilitates communication between an ESP32 microcontroller
 * and a ROS 2 environment.
 *
 * Features:
 * - Serial communication
 * - ROS 2 custom action integration
 * - PID tuning with RL support
 *
 * Author: Arvindh Ramesh
 *
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
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
#include <rmw_microxrcedds_c/config.h>

// Custom action interface includes
#include <rl_pid_uros/action/tune_pid.h>

// Constants and Pin Definitions
#define RXD2 16
#define TXD2 17
#define ENCA 18
#define ENCB 19
#define PWM 14
#define IN1 13
#define IN2 12
#define STBY 27

#define MOTOR_TASK_PRIORITY 2 // Increased priority
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define FEEDBACK_FREQUENCY_MS 100
#define MIN_DELTA_T 1e-6
#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 255
#define STACK_SIZE 8192   // Increased stack size
#define MIN_DELTA_T 0.001 // 1ms minimum time delta
#define EPSILON 1.0e-6
#define DEADBAND 5         // Reduced deadband
#define MAX_INTEGRAL 100.0 // Maximum integral value

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
    .goalActive = false};

// Global declarations
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_action_server_t action_server;
rclc_action_goal_handle_t goal_handle;

// Function declarations
void initializePins();
bool initializeMicroRos();
void motorControlTask(void *arg);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
int getPosition();
void IRAM_ATTR readEncoder();
void sendFeedback(int currentPos, int error);

// Action messages
static bool goal_msg_initialized = false;
// First, update global declarations
static rl_pid_uros__action__TunePID_Goal goal_msg = {
    .kp = 0.0,
    .ki = 0.0,
    .kd = 0.0,
    .target_position = 0};

static rl_pid_uros__action__TunePID_Feedback feedback_msg = {
    .current_error = 0,
    .current_position = 0,
    .target_position = 0

};

static rl_pid_uros__action__TunePID_Result result_msg = {
    .final_error = 0};

/*================================setup, loop and motor control implementation==================================*/

void setup()
{
  Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Initialize mutex
  pidMutex = xSemaphoreCreateMutex();
  if (pidMutex == NULL)
  {
    Serial.println("Error creating mutex");
    return;
  }

  initializePins();
  if (!initializeMicroRos())
  {
    return;
  }

  // Create motor control task
  BaseType_t xReturned = xTaskCreatePinnedToCore(
      motorControlTask,
      "Motor Control",
      STACK_SIZE,
      NULL,
      MOTOR_TASK_PRIORITY,
      NULL,
      1); // Run on core 1

  if (xReturned != pdPASS)
  {
    Serial.println("Failed to create motor control task");
    return;
  }

  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 10000,                                          // 10 seconds
      .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Bitmask of all cores
      .trigger_panic = true};
}

// Motor control task
void motorControlTask(void *arg)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  while (1)
  {
    if (motor.goalActive)
    {
      // Short delay to prevent watchdog triggers
      vTaskDelay(pdMS_TO_TICKS(1));

      if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(50)) == pdTRUE)
      {
        // Get current time and position
        long currT = micros();
        double deltaT = ((float)(currT - motor.prevT)) / 1.0e6;
        deltaT = max(deltaT, MIN_DELTA_T);
        motor.prevT = currT;

        int currentPos = getPosition();
        int e = motor.target - currentPos;

        // PID calculations
        float dedt = (e - motor.eprev) / (deltaT + EPSILON);
        motor.eintegral += e * deltaT;
        motor.eintegral = constrain(motor.eintegral, -MAX_INTEGRAL, MAX_INTEGRAL);

        float pidOutput = (motor.kp * e) + (motor.ki * motor.eintegral) + (motor.kd * dedt);

        // Convert to motor commands
        float pwr = constrain(fabs(pidOutput), 0, MAX_PWM_VALUE);
        int dir = (pidOutput >= 0) ? 1 : -1;

        // Apply deadband
        if (pwr < DEADBAND)
        {
          pwr = 0;
          dir = 0;
        }

        setMotor(dir, pwr, PWM, IN1, IN2);
        motor.eprev = e;

        // Only send feedback occasionally to reduce system load
        static uint32_t lastFeedbackTime = 0;
        uint32_t currentTime = millis();

        if (currentTime - lastFeedbackTime >= 100)
        { // 10Hz feedback rate
          lastFeedbackTime = currentTime;

          // Update feedback message
          feedback_msg.current_position = currentPos;
          feedback_msg.target_position = motor.target;
          feedback_msg.current_error = e;

          // Release mutex before communication
          xSemaphoreGive(pidMutex);

          // Send feedback
          rcl_ret_t ret = rclc_action_publish_feedback(&goal_handle, &feedback_msg);
          if (ret != RCL_RET_OK)
          {
            Serial.printf("Feedback error: %d\n", ret);
          }

          // Check for goal completion
          if (abs(e) < DEADBAND)
          {
            static int stableCount = 0;
            stableCount++;

            if (stableCount >= 5)
            { // Reduced stability count
              motor.goalActive = false;
              result_msg.final_error = e;

              ret = rclc_action_send_result(&goal_handle, GOAL_STATE_SUCCEEDED, &result_msg);
              if (ret != RCL_RET_OK)
              {
                Serial.printf("Result error: %d\n", ret);
              }

              stableCount = 0;
            }
          }
        }
        else
        {
          xSemaphoreGive(pidMutex);
        }
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(10)); // Delay if no goal is active
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Maintain consistent timing
  }
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  if (uxTaskGetStackHighWaterMark(NULL) < 500)
  {
    Serial.println("Warning: Low stack space");
  }

  delay(10);
}

/*===============================================micro-ros intialization and definition=========================================*/

bool initializeMicroRos()
{
  allocator = rcl_get_default_allocator();

  Serial.println("Setting up micro-ROS transport");
  set_microros_transports();

  delay(2000);

  // Trying to connect to agent
  Serial.println("Attempting to connect to micro-ROS agent...");

  bool connected = false;
  for (int i = 0; i < 3; i++)
  {
    connected = rmw_uros_ping_agent(1000, 3) == RMW_RET_OK;
    if (connected)
    {
      Serial.println("Successfully connected to micro-ROS agent!");
      break;
    }
    Serial.println("Could not connect to agent, retrying...");
    delay(1000);
  }

  // Initializing ROS 2 node
  // Initialize support with init options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.printf("Init options failed: %d\n", ret);
    return false;
  }

  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret == RCL_RET_INVALID_ARGUMENT)
  {
    Serial.println("  Invalid argument provided to rclc_support_init_with_options");
  }
  else if (ret == RCL_RET_BAD_ALLOC)
  {
    Serial.println("  Memory allocation failed");
  }
  else
  {
    Serial.printf("  Unknown error code: %d\n", ret);
  }

  ret = rclc_node_init_default(&node, "tune_pid_node", "", &support);
  if (ret != RCL_RET_OK)
  {
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

  if (ret != RCL_RET_OK)
  {
    Serial.printf("Error initializing action server: %d\n", ret);
    return false;
  }

  // Initialize executor
  executor = rclc_executor_get_zero_initialized_executor();
  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.println("Failed to initialize executor");
    return false;
  }

  // Add action server to executor
  ret = rclc_executor_add_action_server(
      &executor,
      &action_server,
      1,
      &goal_msg,
      sizeof(rl_pid_uros__action__TunePID_Goal),
      handle_goal,
      handle_cancel,
      &goal_msg);

  if (ret != RCL_RET_OK)
  {
    Serial.printf("Error adding action server to executor: %d\n", ret);
    return false;
  }

  Serial.println("micro-ROS initialized successfully");
  return true;
}

// Goal handle callback
static rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, void *context)
{
  Serial.println("=== Goal Handler Entry ===");

  if (goal_handle == NULL)
  {
    Serial.println("Error: Goal handle is NULL");
    return RCL_RET_ERROR;
  }

  const rl_pid_uros__action__TunePID_Goal *incoming_goal =
      (const rl_pid_uros__action__TunePID_Goal *)context;

  Serial.println("=== Raw Goal Values ===");
  Serial.print("KP raw: ");
  Serial.println(incoming_goal->kp, 6);
  Serial.print("KI raw: ");
  Serial.println(incoming_goal->ki, 6);
  Serial.print("KD raw: ");
  Serial.println(incoming_goal->kd, 6);
  Serial.print("Target raw: ");
  Serial.println(incoming_goal->target_position);

  // Add range checking
  const double MAX_PID_VALUE = 10.0;
  const long long MAX_TARGET_VALUE = 360;
  if ((fabs(incoming_goal->kp) > MAX_PID_VALUE) || (fabs(incoming_goal->ki) > MAX_PID_VALUE) || (fabs(incoming_goal->kd) > MAX_PID_VALUE) || (abs(incoming_goal->target_position) > MAX_TARGET_VALUE))
  {
    Serial.println("Error: Parameters out of valid range");
    return RCL_RET_ERROR;
  }

  if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    Serial.println("Error: Could not obtain mutex lock");
    return RCL_RET_ERROR;
  }

  motor.kp = constrain(incoming_goal->kp, 0.0, 10.0);
  motor.ki = constrain(incoming_goal->ki, 0.0, 10.0);
  motor.kd = constrain(incoming_goal->kd, 0.0, 10.0);
  motor.target = constrain(incoming_goal->target_position, 0, 360);

  motor.eprev = 0;
  motor.eintegral = 0;
  motor.prevT = micros();
  motor.goalActive = true;

  // Debug print the actual values being set
  Serial.println("=== Setting Motor Parameters ===");
  Serial.printf("KP: %.6f\n", motor.kp);
  Serial.printf("KI: %.6f\n", motor.ki);
  Serial.printf("KD: %.6f\n", motor.kd);
  Serial.printf("Target: %lld\n", motor.target);

  xSemaphoreGive(pidMutex);
  Serial.println("=== Goal Accepted ===");
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

// Cancel handle callback
static bool handle_cancel(rclc_action_goal_handle_t *goal_handle, void *context)
{
  (void)context;
  if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
  {
    motor.goalActive = false;
    xSemaphoreGive(pidMutex);
    return true;
  }
  return false;
}

/*======================================motor control, wifi and pin definitions============================================*/

// Initialize pins
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

// Set motor speed and direction
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  digitalWrite(STBY, HIGH);
  ledcWrite(pwm, pwmVal);
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == 1 ? LOW : HIGH);
}

// Get current motor position
int getPosition()
{
  int pos;
  portENTER_CRITICAL(&positionMux);
  pos = motor.position;
  portEXIT_CRITICAL(&positionMux);
  return pos;
}

// Encoder interrupt handler
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