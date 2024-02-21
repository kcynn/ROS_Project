#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>


ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

// Define pins for rotary encoder switch
const int PIN_A = 2;
const int PIN_B = 3;
const int SW_PIN = 4; // Rotary encoder switch pin
const int homePosition = 90; // Initial position
const int stepValue = 3;     // How fast the servo should rotate when turning the knob
const int servoPin = 9;      // Must be a pin labeled with ~
long oldPosition = -999;
int servoAngle = homePosition;
int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
bool switch_mode = false;

Encoder myEnc(PIN_A, PIN_B);

void servo1_cb( const std_msgs::UInt16& cmd1_msg){
  if(switch_mode){
  servo1.write(cmd1_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  }
  else{
    
  }
}

void servo2_cb( const std_msgs::UInt16& cmd2_msg){
  if(switch_mode){
    servo2.write(cmd2_msg.data); 
    digitalWrite(13, HIGH-digitalRead(13));  
  }
  else{
     
  }
}

void switch_callback(const std_msgs::Bool& msg) {
   switch_mode = msg.data;
  if (switch_mode) {
      digitalWrite(13, HIGH);
  } else {
     digitalWrite(13, LOW);
  }
}

ros::Subscriber<std_msgs::UInt16> servo1_sub("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> servo2_sub("servo2", servo2_cb);
ros::Subscriber<std_msgs::Bool> switch_sub("mode", &switch_callback);

void setup(){
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(servo1_sub);
  nh.subscribe(servo2_sub);
  nh.subscribe(switch_sub);
  servo2.attach(9);
  servo1.attach(8); 
  pinMode(SW_PIN, INPUT_PULLUP);
}


void loop(){
  nh.spinOnce();
  if(!switch_mode){
    MM_robot();
  }
  else{
    
  }
  delay(1);
}

void MM_robot(){
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (newPosition > oldPosition) {
      servoAngle += stepValue;
      if (servoAngle > 180)
        servoAngle = 180;
    } else {
      servoAngle -= stepValue;
      if (servoAngle < 0)
        servoAngle = 0;
    }
    servo1.write(servoAngle);
    oldPosition = newPosition;
  }

  // Check if the switch is pressed to reset the servo position to home
  if (digitalRead(SW_PIN) == LOW) {
    servoAngle = homePosition;
    servo1.write(servoAngle);
    delay(50); // Debouncing delay
  }
  // Control second servo with potentiometer
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  servo2.write(val);                  // sets the servo position according to the scaled value
}
