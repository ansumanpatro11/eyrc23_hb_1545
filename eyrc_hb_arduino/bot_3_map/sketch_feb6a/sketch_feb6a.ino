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

int fw_pwm(float rpm){
  int pwm;
  if(rpm>10){

    //0 t0 90 clockwise
    const float a = -0.0012;
    const float b = 0.0923;
    const float c = -3.8639;
    const float d = 119.1027;

    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }
  else if(rpm<-10){
    //90 to 180 anti-clockwise
    const float a = -0.0006;
    const float b = -0.0478;
    const float c = -2.8861;
    const float d = 75.2306;

    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }else{
    pwm=90;
  }
  return pwm;
}
int rw_pwm(float rpm){
  int pwm;
  if(rpm>10){
    const float a = 0.0012;
    const float b = -0.1196;
    const float c = 1.1861;
    const float d = 85.5759;




    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }
  else if(rpm<-10){
    const float a = -0.0002;
    const float b = -0.0257;
    const float c = -3.0805;
    const float d = 69.3717;
    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }else{
    pwm=90;
  }
  return pwm;
}
int lw_pwm(float rpm){
  int pwm;
  if(rpm>10){
    const float a = -0.0010;
    const float b = 0.0837;
    const float c = -3.7968;
    const float d = 119.365;





    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }
  else if(rpm<-10){
    const float a = -0.0007;
    const float b = -0.0601;
    const float c = -3.2047;
    const float d = 71.6505;
        pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);
  }else{
    pwm=90;
  }
  return pwm;
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
  Serial.begin(9600);
  ESP32PWM:: allocateTimer(0);

  set_microros_wifi_transports("Redmi Note 10 Pro","laliga1234","192.168.64.112",8888);
  servo_init();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  // delay(2000);

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
  // delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(100);

  
}
