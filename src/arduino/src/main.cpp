#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <main.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/string.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t publisher_status;

// ROS2 functions
// -----------------------------------------------------------------------------
void publish_status(String msg) {
    RCSOFTCHECK(rcl_publish(&publisher_status, &msg, nullptr));
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "arduino_diffdrive", "raros_nav", &support));

    // create publishers
    RCCHECK(rclc_publisher_init_best_effort(
            &publisher_status,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "arduino_diffdrive/status"));

    // create executors
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    RCSOFTCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

    RCSOFTCHECK(rcl_publisher_fini(&publisher_status, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}


// Arduino functions
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(LED_BUILTIN, OUTPUT);

    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                               state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                            ? AGENT_CONNECTED
                                            : AGENT_DISCONNECTED);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                publish_status("some status");
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

    digitalWrite(LED_BUILTIN, state == AGENT_CONNECTED);
}