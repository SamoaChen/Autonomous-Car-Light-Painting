//This file takes desired count velocities as input through serial communication and utilize those values in a feedback loop to control the speed of two rear wheels

#include "PinChangeInterrupt.h"

//-----390 pulses per revolution
//-----DEFINE INTERUPT PIN NUMBER
#define rot_pin1 3
#define rot_pin2 4
#define rot_pin3 5
#define rot_pin4 6
#define pwm_pin_right 11
#define dir_pin1 12
#define dir_pin2 13
#define pwm_pin_left 10
#define dir_pin3 9
#define dir_pin4 8

//-----DEFINE ENCODER PARAMETERS
#define pi 3.141592653589
int rev_count = 2*390;

double angle_conversion = pi/rev_count;

volatile long counter_right = 0;
volatile long counter_left = 0;
volatile long old_counter_right = 0;
volatile long old_counter_left = 0;
unsigned long old_time = millis();
volatile double counter_velocity_right = 0.0;
volatile double counter_velocity_left = 0.0;

//-----DEFINE CONTROL PARAMETER
double kp = 500;
double ki = 30;
double error_left;
double error_right;
double desired_counter_v_left = 0;
double desired_counter_v_right = 0;
double integral_error_left = 0.0;
double integral_error_right = 0.0;
double fd_left = 0;
double fd_right = 0;
int pwm_output_left = 0;
int pwm_output_right = 0;
double conversion_factor = 66.9467; //convert count per milisecond to the approximate corresponding pwm value where 293 is the rpm rate corresponding to full pwm output

 
void setup() {
  pinMode(rot_pin1, INPUT_PULLUP);
  pinMode(rot_pin2, INPUT_PULLUP);
  pinMode(rot_pin3, INPUT_PULLUP);
  pinMode(rot_pin4, INPUT_PULLUP);
  pinMode(pwm_pin_left, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  pinMode(pwm_pin_right, OUTPUT);
  pinMode(dir_pin3, OUTPUT);
  pinMode(dir_pin4, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rot_pin1), call_back1, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rot_pin2), call_back2, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rot_pin3), call_back3, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rot_pin4), call_back4, RISING);

  Serial.begin(9600);
}

void loop() {

  vel_calculation();
  read_serial();
  control_function();
  actuation();
  Serial.println(counter_velocity_right);

}

void read_serial(){
  if(Serial.available()>0){
    if(Serial.peek()=='r'){
      Serial.read();
      desired_counter_v_left = Serial.parseInt()/1000.0;
      Serial.read();
      desired_counter_v_right = desired_counter_v_left;
    }
    
  }
}

void actuation(){
  //actuate the left motor
  if (pwm_output_right > 255){
    analogWrite(pwm_pin_right, 255);
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  }else if(pwm_output_right >= 0){
    analogWrite(pwm_pin_right, pwm_output_right);
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  }else if(pwm_output_right < -255){
    analogWrite(pwm_pin_right, 255);
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  }else if(pwm_output_right < 0){
    analogWrite(pwm_pin_right, abs(pwm_output_right));
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  }


  //actuate the right motor
  if (pwm_output_left > 255){
    analogWrite(pwm_pin_left, 255);
    digitalWrite(dir_pin3, LOW);
    digitalWrite(dir_pin4, HIGH);
  }else if(pwm_output_left >= 0){
    analogWrite(pwm_pin_left, pwm_output_left);
    digitalWrite(dir_pin3, LOW);
    digitalWrite(dir_pin4, HIGH);
  }else if(pwm_output_left < -255){
    analogWrite(pwm_pin_left, 255);
    digitalWrite(dir_pin3, HIGH);
    digitalWrite(dir_pin4, LOW);
  }else if(pwm_output_left < 0){
    analogWrite(pwm_pin_left, abs(pwm_output_left));
    digitalWrite(dir_pin3, HIGH);
    digitalWrite(dir_pin4, LOW);
  }
}

void control_function(){
  //calculate error
  error_left = desired_counter_v_left-counter_velocity_left;
  integral_error_left += error_left;
  pwm_output_left = int(conversion_factor*desired_counter_v_left+kp*error_left+ki*integral_error_left);

  error_right = desired_counter_v_right-counter_velocity_right;
  integral_error_right += error_right;
  pwm_output_right = int(conversion_factor*desired_counter_v_right+kp*error_right+ki*integral_error_right);
  
}

void vel_calculation(){
  long current_time = millis();
  double time_difference = current_time-old_time;
  if(time_difference>=5){
    //calculate counter differences
    delay(1); //spare some time for encoder to change
    int right_counter_difference = counter_right-old_counter_right;
    int left_counter_difference = counter_left-old_counter_left;

    //calculate counter velocities
    counter_velocity_right = right_counter_difference/time_difference;
    counter_velocity_left = left_counter_difference/time_difference;

    //update counter and time
    old_counter_right = counter_right;
    old_counter_left = counter_left;
    old_time = current_time;
  }

}

void call_back1(){
  if(digitalRead(rot_pin2) == LOW){
    counter_right++;
  }else{
    counter_right--;
  }
}

void call_back2(){
  if(digitalRead(rot_pin1) == LOW){
    counter_right--;
  }else{
    counter_right++;
  }
}

void call_back3(){
  if(digitalRead(rot_pin4) == LOW){
    counter_left++;
  }else{
    counter_left--;
  }
}

void call_back4(){
  if(digitalRead(rot_pin3) == LOW){
    counter_left--;
  }else{
    counter_left++;
  }
}
