#include <micro_ros_arduino.h>
#include<ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>



rcl_subscription_t subscriber;
rcl_subscription_t subscriber2;
std_msgs__msg__Int32 msg2;

geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


Servo servo_fw; 
Servo servo_rw; 
Servo servo_lw; 
Servo servo_pen_mode;


#define LED_PIN 2
#define servo_fw_pin  27
#define servo_lw_pin  25
#define servo_rw_pin  26
// #define servo_pen_pin 28

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


void servo_init(){
  servo_fw.attach(servo_fw_pin);
  servo_rw.attach(servo_rw_pin);
  servo_lw.attach(servo_lw_pin);
  // servo_pen_mode.attach(servo_pen_pin);
  
 
}
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }


}

int fw_sync(int vel){
  float fw_sync=0;
  if(vel>0){
    fw_sync=map(vel,0,120,99,167.69);

  }else if(vel<0){
    fw_sync=map(vel,-120,0,5.84,87);
   
  }else{
    fw_sync=90;

  }

  return int(fw_sync);

}

int lw_sync(int vel){
  float lw_sync=0;
  if(vel>0){
    lw_sync=map(vel,0,120,99,163.44);

  }else if(vel<0){
    lw_sync=map(vel,-120,0,8.86,87);
   
  }else{
    lw_sync=90;

  }

  return int(lw_sync);

}

int rw_sync(int vel){
  float rw_sync=0;
  if(vel>0){
    rw_sync=map(vel,0,120,99,180);

  }else if(vel<0){
    rw_sync=map(vel,-120,0,0,87);
   
  }else{
    rw_sync=90;

  }

  return int(rw_sync);

}
//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  // let us store linear.x=fw_vel
  // let us store linear.y=lw_vel
  // let us store linear.z=rw_vel
  int vel_fw=msg->linear.x;
  int vel_lw=msg->linear.y;
  int vel_rw=msg->linear.z;
  // int position_fw = map(vel_fw, -20, 20, 24, 165);
  // int position_lw = map(vel_lw, -20, 20, 26, 161);
  // int position_rw = map(vel_rw, -20, 20, 0, 180);
  

  






  vel_fw=fw_sync(vel_fw);
  vel_rw=rw_sync(vel_rw);
  vel_lw=lw_sync(vel_lw);

  Serial.print(vel_fw);
  Serial.print(" ");

  Serial.print(vel_lw);
  Serial.print(" ");
  Serial.print(vel_rw);


  // Serial.println(position_fw);
  servo_fw.write(vel_fw);
  servo_lw.write(vel_lw);
  servo_rw.write(vel_rw);
  Serial.println("hello");

  digitalWrite(LED_PIN, (msg->linear.x == 0) ? HIGH : LOW);

}
// void subscription_callback2(const void *msgin) {
//   // Your callback logic for the second subscriber
//    const std_msgs__msg__Int32 * msg2 = (const std_msgs__msg__Int32 *)msgin;
//    int pen_mode=msg2-> data;
//    if(pen_mode==1){
//     int pen_pwm=20;
//     servo_pen_mode.write(pen_pwm);

//    }
  
// }

  
  


void setup() {
  Serial.begin(115200);


  set_microros_wifi_transports("Pixel 4 XL","Amit2000","192.168.50.5",8888);
  servo_init();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot1_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/bot1"));

  // RCCHECK(rclc_subscription_init_default(
  // &subscriber2,
  // &node,
  // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  // "/pen_pose"));


    

  // create executor

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback2, ON_NEW_DATA));


}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(100);

  
}
