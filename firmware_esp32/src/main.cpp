#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>

extern "C" {
  #include <rcl/rcl.h>
  #include <rcl/publisher.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <std_msgs/msg/float64.h>
  #include <sensor_msgs/msg/laser_scan.h>
  #include <rmw_microros/rmw_microros.h>
}

// Hardware Pin Definitions
#define TRIG_PIN 5
#define ECHO_PIN 19
#define SERVO_PIN 18

// Number of points per scan message (matching Gazebo structure)
#define NUM_RANGES 5

Servo mg90s_servo;

rcl_subscription_t joint_sub;
rcl_publisher_t scan_pub;

std_msgs__msg__Float64 joint_msg;
sensor_msgs__msg__LaserScan scan_msg;

float scan_range_data[NUM_RANGES];
float scan_intensity_data[NUM_RANGES];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float read_ultrasonic_meters() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) {
    return INFINITY; // Match Gazebo out-of-range convention (.inf)
  }

  float distance_m = (duration * 0.000343) / 2.0;
  if (distance_m < 0.02 || distance_m > 4.0) {
    return INFINITY;
  }
  return distance_m;
}

void joint_callback(const void *msgin) {
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
  
  float target_rad = msg->data;
  int target_deg = (int)(target_rad * 180.0 / M_PI);
  target_deg = constrain(target_deg, 0, 180);

  // Move Servo
  mg90s_servo.write(target_deg);
  delay(15);

  // Take ultrasonic sample
  float dist = read_ultrasonic_meters();

  // Fill array matching Gazebo multi-range format
  for (int i = 0; i < NUM_RANGES; i++) {
    scan_range_data[i] = dist;
    scan_intensity_data[i] = 0.0;
  }

  // Populate LaserScan Fields matching Gazebo
  scan_msg.angle_min = target_rad - 0.05;
  scan_msg.angle_max = target_rad + 0.05;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (NUM_RANGES - 1);
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.05;
  scan_msg.range_min = 0.02;
  scan_msg.range_max = 4.0;

  // Header Timestamp
  int64_t epoch_time = rmw_uros_epoch_millis();
  scan_msg.header.stamp.sec = (int32_t)(epoch_time / 1000);
  scan_msg.header.stamp.nanosec = (uint32_t)((epoch_time % 1000) * 1000000);

  // Publish
  rcl_publish(&scan_pub, &scan_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  mg90s_servo.setPeriodHertz(50);
  mg90s_servo.attach(SERVO_PIN, 544, 2550);
  mg90s_servo.write(0);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_sonar_node", "", &support);

  rmw_uros_sync_session(1000);

  rclc_subscription_init_default(
    &joint_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/joint_cmd_pos"
  );

  rclc_publisher_init_default(
    &scan_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/sonar/distance"
  );

  // Setup static array sizes matching NUM_RANGES
  scan_msg.header.frame_id.data = (char*)"ultrasonic";
  scan_msg.header.frame_id.size = strlen("ultrasonic");
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

  scan_msg.ranges.data = scan_range_data;
  scan_msg.ranges.size = NUM_RANGES;
  scan_msg.ranges.capacity = NUM_RANGES;

  scan_msg.intensities.data = scan_intensity_data;
  scan_msg.intensities.size = NUM_RANGES;
  scan_msg.intensities.capacity = NUM_RANGES;

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &joint_sub, &joint_msg, &joint_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}