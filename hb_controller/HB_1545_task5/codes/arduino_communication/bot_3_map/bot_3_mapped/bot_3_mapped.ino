#include <micro_ros_arduino.h>
#include<ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>



rcl_subscription_t subscriber;
rcl_subscription_t subscriber2;
std_msgs__msg__Bool msg_bool;

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
#define servo_pen_pin 14

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


void servo_init(){
  servo_fw.attach(servo_fw_pin);
  servo_rw.attach(servo_rw_pin);
  servo_lw.attach(servo_lw_pin);
  servo_pen_mode.attach(servo_pen_pin);
  servo_pen_mode.write(105);
  
 
}
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// int fw_pwm(float rpm){
//   int pwm;
//   if(rpm>10){

//     //0 t0 90 clockwise
//     const float a = -0.000;
//     const float b = 0.0001;
//     const float c = -1.6928;
//     const float d = 102.1493;

//     pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }
//   else if(rpm<-10){
//     //90 to 180 anti-clockwise
//     const float a = -0.0006;
//     const float b = -0.0440;
//     const float c = -2.9334;
//     const float d = 75.5371;

//     pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }else{
//     pwm=90;
//   }
//   return pwm;
// }
// int rw_pwm(float rpm){
//   int pwm;
//   if(rpm>10){
//     const float a = -0.0001;
//     const float b = 0.0103;
//     const float c = -2.0183;
//     const float d = 104.8149;




//     pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }
//   else if(rpm<-10){
//     const float a = -0.0003;
//     const float b = -0.0184;
//     const float c = -2.1465;
//     const float d = 81.1205;

//     pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }else{
//     pwm=90;
//   }
//   return pwm;
// }
// int lw_pwm(float rpm){
//   int pwm;
//   if(rpm>10){
//     const float a = 0.0001;
//     const float b = -0.0132;
//     const float c = -1.4753;
//     const float d = 101.0865;





//     pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }
//   else if(rpm<-10){
//     const float a = 0.0003;
//     const float b = 0.0243;
//     const float c = -1.1821;
//     const float d = 87.1315;
//         pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
//   }else{
//     pwm=90;
//   }
//   return pwm;
// }
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
  // Serial.println(position_fw);
  // Serial.println(position_lw);
  // Serial.println(position_rw);


  // vel_fw=fw_pwm(vel_fw);
  // vel_rw=rw_pwm(vel_rw);
  // vel_lw=lw_pwm(vel_lw);
 
  
  // Serial.println(position_fw);
  servo_fw.write(vel_fw);
  servo_lw.write(vel_lw);
  servo_rw.write(vel_rw);
  Serial.println("hello");

  digitalWrite(LED_PIN, (msg->linear.x == 0) ? HIGH : LOW);

}
void subscription_callback2(const void *msgin) {
  // Your callback logic for the second subscriber
   const std_msgs__msg__Bool * msg_bool = (const std_msgs__msg__Bool *)msgin;
   bool pen_mode=msg_bool-> data;
   if(pen_mode==true){
    int pen_down=145;  //pen down
    servo_pen_mode.write(pen_down);

   }else if(pen_mode==false){
    int pen_up=105;
    servo_pen_mode.write(pen_up);
   }
  
}

  
  


void setup() {
  Serial.begin(9600);
  ESP32PWM:: allocateTimer(0);

  set_microros_wifi_transports("Redmi Note 10 Pro","laliga1234","192.168.64.112",7777);
  servo_init();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  // delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot3_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/bot3"));

  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pen3_down"));


    

  // create executor

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg_bool, &subscription_callback2, ON_NEW_DATA));


}

void loop() {
  // delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(100);

  
}
