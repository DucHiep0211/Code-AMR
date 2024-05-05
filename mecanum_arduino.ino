#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/time.h> 

// ======= L298N 

// front_left Motor
#define front_left_EN         9       // pin 9 : PWM
#define front_left_in_01      24      // pin 24: in1
#define front_left_in_02      25      // pin 25: in2
#define front_left_A          0       // pin 2 :   Interrupt 
#define front_left_B          40      // pin 40: 

// front_right Motor
#define front_right_EN        10      // pin 9 : PWM  
#define front_right_in_01     26      // pin 26: in3 
#define front_right_in_02     27      // pin 27: in4
#define front_right_A         1       // pin 3 : Interrupt
#define front_right_B         42      // pin 42: 


// ======= L298N 
// rear_left Motor
#define rear_left_EN          11      // pin 11 : PWM  
#define rear_left_in_01       28      // 
#define rear_left_in_02       29
#define rear_left_A           2       // pin 21 : Interrupt
#define rear_left_B           44      // pin 44: 

// rear_right Motor
#define rear_right_EN         12     // pin 12 : PWM  
#define rear_right_in_01      30
#define rear_right_in_02      31
#define rear_right_A          3       // pin 21 : Interrupt
#define rear_right_B          46      // pin 46: 


// Dimention of robot

float distance_wheels = 0.245;
float diameter_wheel = 0.070;  

ros::NodeHandle  nh;

signed long front_left_encod = 0  ;
signed long front_right_encod= 0 ;


signed long rear_left_encod = 0;
signed long rear_right_encod = 0;

//signed long data_pose_encod[2] = {0,0};
signed long data_pose_encod[4] = {0,0,0,0};

int list_in_01[4] ={front_left_in_01,front_right_in_01,rear_left_in_01,rear_right_in_01};
int list_in_02[4] ={front_left_in_02,front_right_in_02,rear_left_in_02,rear_right_in_02};

int list_EN[4] = {front_left_EN,front_right_EN,rear_left_EN,rear_right_EN};

int list_direct[4] = {0,0,0,0};
int list_duty[4] = {0,0,0,0};

unsigned long quakhu;
float linear_x, angular_z;
float vel_left_wheel , vel_right_wheel;


float tsamp = 0.02;

std_msgs::Int32MultiArray array_pose_encod;     // Publisher array pose encoder
ros::Publisher pub_array_pose_encod("pose_encod",&array_pose_encod);

std_msgs::Float32MultiArray array_vels_robot;     // Publisher array pose encoder
ros::Publisher pub_array_vels_robot("vels_robot",&array_vels_robot);


std_msgs::Float32MultiArray array_vels_wheels;     // Publisher array pose encoder
ros::Publisher pub_array_vels_wheels("vels_wheels",&array_vels_wheels);

std_msgs::Float32MultiArray array_vels_enc;     // Publisher array pose encoder
ros::Publisher pub_array_vels_enc("vels_enc",&array_vels_enc);

void cmd_vel_Cb( const geometry_msgs::Twist& data_Twist){
  linear_x = data_Twist.linear.x;
  angular_z = data_Twist.angular.z;

  vel_right_wheel = ((2 * linear_x) + (angular_z * 235 / 1000)) / 2 ;
  vel_left_wheel = ((2 * linear_x) - (angular_z * 235 / 1000)) / 2 ;

}

void duty_Cb( const std_msgs::Int32MultiArray&  data){

  list_duty[0] = data.data[0];
  list_duty[1] = data.data[1];
  list_duty[2] = data.data[2];
  list_duty[3] = data.data[3];
}

void direct_Cb( const std_msgs::Int32MultiArray&  msg){

  list_direct[0] = msg.data[0];
  list_direct[1] = msg.data[1];
  list_direct[2] = msg.data[2];
  list_direct[3] = msg.data[3];
//  duty_right = data.data[0];
//  duty_left = data.data[1];

//  set_pwm_right_motor(vel_right_wheel,duty_right);
//  set_pwm_left_motor(vel_left_wheel,duty_left);
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel",cmd_vel_Cb);

ros::Subscriber<std_msgs::Int32MultiArray> sub_duty_motor("/duty_motor",duty_Cb);
ros::Subscriber<std_msgs::Int32MultiArray> sub_direct_motor("/direct_motor",direct_Cb);
void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  
  nh.advertise(pub_array_pose_encod);
  nh.advertise(pub_array_vels_robot);
  nh.advertise(pub_array_vels_wheels);
  nh.advertise(pub_array_vels_enc);

  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_duty_motor);
  nh.subscribe(sub_direct_motor);

  pinMode(front_left_A, INPUT_PULLUP);
  pinMode(front_left_B, INPUT_PULLUP);
  
  pinMode(front_right_A, INPUT_PULLUP);
  pinMode(front_right_B, INPUT_PULLUP);
  
  pinMode(rear_left_A, INPUT_PULLUP);
  pinMode(rear_left_B, INPUT_PULLUP);

  pinMode(rear_right_A, INPUT_PULLUP);
  pinMode(rear_right_B, INPUT_PULLUP);


  pinMode(front_left_in_01, OUTPUT);
  pinMode(front_left_in_02, OUTPUT);
  pinMode(front_left_EN, OUTPUT);

  pinMode(front_right_in_01, OUTPUT);
  pinMode(front_right_in_02, OUTPUT);
  pinMode(front_right_EN, OUTPUT);

  pinMode(rear_left_in_01, OUTPUT);
  pinMode(rear_left_in_02, OUTPUT);
  pinMode(rear_left_EN, OUTPUT);

  pinMode(rear_right_in_01, OUTPUT);
  pinMode(rear_right_in_02, OUTPUT);
  pinMode(rear_right_EN, OUTPUT);
  
  attachInterrupt(0, front_left_encoder, RISING);
  attachInterrupt(1, front_right_encoder, RISING);
  attachInterrupt(2, rear_left_encoder, RISING);
  attachInterrupt(3, rear_right_encoder, RISING);
}

void loop() {


  
  // right_pose_encod_old = right_pose_encod;
  // left_pose_encod_old = left_pose_encod;
  // quakhu=millis();

  pub_data_pose_encod();
  // pub_vels_robot();
  // pub_vel_wheels();

  
  nh.spinOnce();
}

void front_left_encoder(){
  if(digitalRead(front_left_B)==HIGH) front_left_encod ++;
  if(digitalRead(front_left_B)==LOW) front_left_encod --;
}


void front_right_encoder(){
  if(digitalRead(front_right_B)==HIGH) front_right_encod ++;
  if(digitalRead(front_right_B)==LOW) front_right_encod --;
}


void rear_left_encoder(){
  if(digitalRead(rear_left_B)==HIGH) rear_left_encod ++;
  if(digitalRead(rear_left_B)==LOW) rear_left_encod --;
}

void rear_right_encoder(){
  if(digitalRead(rear_right_B)==HIGH) rear_right_encod --;
  if(digitalRead(rear_right_B)==LOW) rear_right_encod ++;
}

void pub_data_pose_encod(){


    
    // data_pose_encod[0] = right_pose_encod;
    // data_pose_encod[1] = left_pose_encod;
    
    array_pose_encod.data = data_pose_encod;
    array_pose_encod.data_length = 4;
    pub_array_pose_encod.publish(&array_pose_encod);
}

void pub_vels_robot(){
    float data_array[2] = {0.0,0.0};
    // data_array[0] = linear_x;
    // data_array[1] = angular_z;

    // array_vels_robot.data = data_array;
    // array_vels_robot.data_length = 2 ;
    
    // pub_array_vels_robot.publish(&array_vels_robot);
}

void pub_vels_encs(float vel_enc_01, float vel_enc_02){
    float data_array[2] = {0.0,0.0};
//     data_array[0] = vel_enc_01;
//     data_array[1] = vel_enc_02;

//     array_vels_enc.data = data_array;
//     array_vels_enc.data_length = 2 ;
    
//     pub_array_vels_enc.publish(&array_vels_enc);
// }
}

void pub_vel_wheels(){
  
//  float v_r=((2 * linear_x) + (angular_z * distance_wheels / 1000)) / 2;
//  float v_l=((2 * linear_x) - (angular_z * distance_wheels / 1000)) / 2;

  // float data_array[2] = {0.0,0.0};
  // data_array[0] = v_r;
  // data_array[1] = v_l;

  // array_vels_wheels.data = data_array;
  // array_vels_wheels.data_length = 2;
  
  // pub_array_vels_wheels.publish(&array_vels_wheels);

}

void set_pwm_motor ( int stt, float direct , float duty){
  
  if (stt == 0){

  }
  
  if (direct > 0 ){
    digitalWrite(list_in_01[stt], LOW);
    digitalWrite(list_in_02[stt] , HIGH);

    analogWrite(list_EN[stt],duty);
  }

  if (direct < 0 ){
    digitalWrite(list_in_01[stt], HIGH);
    digitalWrite(list_in_02[stt] , LOW);

    analogWrite(list_EN[stt],duty);
  }

  if (duty == 0.0 ){
    digitalWrite(list_in_01[stt], LOW);
    digitalWrite(list_in_02[stt] , LOW);

    analogWrite(list_EN[stt],duty);
  }
  
  
    //   digitalWrite(right_R_PWM, LOW);
    // digitalWrite(right_L_PWM , HIGH);
  
  // if (cmd_right > 0 ) { 
  //   digitalWrite(right_R_PWM,HIGH);
  //   analogWrite(right_RL_EN,duty_right_);
  //   }
  // if (cmd_right < 0 ) { 
  //   digitalWrite(right_R_PWM,LOW);
  //   analogWrite(right_RL_EN,duty_right_);
  //   }
  // if (cmd_right == 0.0 ){
  //   digitalWrite(right_R_PWM,LOW);
  //   analogWrite(right_RL_EN,0);
  // }
  
}



// void set_pwm_right_motor ( float cmd_right , float duty_right_){
//   if (cmd_right > 0 ) { 
//     digitalWrite(right_R_PWM,HIGH);
//     analogWrite(right_RL_EN,duty_right_);
//     }
//   if (cmd_right < 0 ) { 
//     digitalWrite(right_R_PWM,LOW);
//     analogWrite(right_RL_EN,duty_right_);
//     }
//   if (cmd_right == 0.0 ){
//     digitalWrite(right_R_PWM,LOW);
//     analogWrite(right_RL_EN,0);
//   }
  
// }


// void set_pwm_right_motor ( float cmd_right , float duty_right_){
//   if (cmd_right > 0 ) { 
//     digitalWrite(right_R_PWM,HIGH);
//     analogWrite(right_RL_EN,duty_right_);
//     }
//   if (cmd_right < 0 ) { 
//     digitalWrite(right_R_PWM,LOW);
//     analogWrite(right_RL_EN,duty_right_);
//     }
//   if (cmd_right == 0.0 ){
//     digitalWrite(right_R_PWM,LOW);
//     analogWrite(right_RL_EN,0);
//   }
  
// }


// void set_pwm_left_motor ( float cmd_left , float duty_left_){
//    if (cmd_left > 0 ) { 
//     digitalWrite(left_R_PWM,LOW);
//     analogWrite(left_RL_EN,duty_left_);
//     }
//   if (cmd_left < 0 ) { 
//     digitalWrite(left_R_PWM,HIGH);
//     analogWrite(left_RL_EN,duty_left_);
//     }
//   if (cmd_left == 0.0 ){
//     digitalWrite(left_R_PWM,LOW);
//     analogWrite(left_RL_EN,0);
//   }

  
// }
