#include <AFMotor.h>
//Define the sensor pins
#define S1Trig A4
#define S2Trig A0
#define S3Trig A2
#define S1Echo A5
#define S2Echo A1
#define S3Echo A3



//Set the speed of the motors
#define Speed 65

//Create objects for the motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
//AF_DCMotor motor3(3);
//AF_DCMotor motor4(4);

char val;
int con;



void setup() {
  Serial.begin(9600);
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);
  //Set the speed of the motors
  motor1.setSpeed(Speed);
  motor2.setSpeed(Speed);
  //motor3.setSpeed(Speed);
  //motor4.setSpeed(Speed);
con=0;
}

void loop() {
  int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
// Check the distance using the IF condition
 if (Serial.available()){
  val = Serial.read();
  Serial.println(val);
 }
 if (val=='1'){
 con = 1;
//work();
 //reading(); 
}

if (val=='2'){
  //Stop();
  con = 2;
}
if (val=='3'){
  //Stop();
  con = 3;
}
if (val=='4'){
  //Stop();
  con = 4;
}
if (val=='5'){
  //Stop();
  con = 5;
}
if (val=='6'){
  //Stop();
  con = 6;
}


if (con==1){
reading(); 
}
if (con==2){
Stop(); 

}
if (con==3){
work(); 
reading(); 

}
if (con==4){
workL();
reading(); 

}
if (con==5){
workR();
reading(); 

}
if (con==6){
workF();
reading(); 

}
if (con==0){
Serial.println("ready");

}
Serial.println(con);
//reading(); 

}
//Get the sensor values
int sensorOne() {
  //pulse output
  digitalWrite(S1Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S1Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S1Trig, LOW);

  long t = pulseIn(S1Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

//Get the sensor values
int sensorTwo() {
  //pulse output
  digitalWrite(S2Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S2Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S2Trig, LOW);

  long t = pulseIn(S2Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

//Get the sensor values
int sensorThree() {
  //pulse output
  digitalWrite(S3Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S3Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S3Trig, LOW);

  long t = pulseIn(S3Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

/******Motor functions*******/
void forward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  
}
void left() {
  motor1.run(RELEASE);
  motor2.run(FORWARD);
  
}
void right() {
  motor1.run(FORWARD);
  motor2.run(RELEASE);

 
}
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  
}

void uright() {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
 
}

void work(){
   int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
  
  if ( 6 >= centerSensor && 12 < leftSensor || 12 < rightSensor ) {
    Stop();
    Serial.println("A");
    //delay(500);
    if (leftSensor > rightSensor) {
      left();
      Serial.println("B");
      //delay(500);
    } else {
      right();
      Serial.println("C");
      //delay(500);
    }
  }
  else if (6 >= centerSensor && 4 >= leftSensor && 4 >= rightSensor){
    uright();
    Serial.println("D");
      delay(600);
    
    }
    else if (6 <= centerSensor && leftSensor > rightSensor){
    left();
      Serial.println("E");
      
    }
    else if (6 <= centerSensor && leftSensor < rightSensor){
    right();
      Serial.println("F");
      
    }
    else if (6 <= centerSensor && leftSensor == rightSensor){
      Serial.println("G");
  forward();
  //delay(230);
  //Stop();
  //delay(10000);
      
    }
     /*else if (7 < centerSensor && 7 < leftSensor && 7 < rightSensor){
    left();
      Serial.println("left");
    delay(500);
    }*/

}

void reading(){
     int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
// Check the distance using the IF condition

Serial.print("   center=");
 Serial.print(centerSensor);
 Serial.print("   left=");
  Serial.print(leftSensor);
  Serial.print("  right=");
   Serial.println(rightSensor);
  
    
    }


    void workL(){
   int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
  
  if (6 >= centerSensor && (12 < leftSensor || 12 < rightSensor) ) {
    Stop();                                                                                                                                                                                                                                                                                                                                                                                                                                     
    Serial.println("A");
    delay(500);
    if (leftSensor > 6 ) {
      left();
      Serial.println("B");
      //delay(500);
    }
     else {
      right();
      Serial.println("C");
      delay(1000);
    }
  }
  else if (6 >= centerSensor && 5 >= leftSensor && 5 >= rightSensor){
    uright();
    Serial.println("D");
      delay(600);
    
    }
    else if (6 <= centerSensor && 4 >= leftSensor ){
    right();
      Serial.println("E");
      
    }
    else if (6 <= centerSensor && 7 < leftSensor ){
    left();
      Serial.println("F");
      
    }
    else if (6 <= centerSensor && 5 == leftSensor  ){
      Serial.println("G");
  forward();
  //delay(230);
  //Stop();
  //delay(10000);
      
    }

    
     /*else if (7 < centerSensor && 7 < leftSensor && 7 < rightSensor){
    left();
      Serial.println("left");
    delay(500);
    }*/

}

 void workR(){
   int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
  
  if (6 >= centerSensor && (12 < leftSensor || 12 < rightSensor) ) {
    Stop();                                                                                                                                                                                                                                                                                                                                                                                                                                     
    Serial.println("A");
    delay(500);
    if (rightSensor > 7 ) {
      left();
      Serial.println("B");
      //delay(500);
    }
     else {
      right();
      Serial.println("C");
      delay(1000);
    }
  }
  else if (6 >= centerSensor && 5 >= leftSensor && 5 >= rightSensor){
    uright();
    Serial.println("D");
      delay(600);
    
    }
    else if (6 <= centerSensor && 6 > rightSensor ){
    left();
      Serial.println("E");
      
    }
    else if (6 <= centerSensor && 6 <= rightSensor ){
    right();
      Serial.println("F");
      
    }
    else if (6 <= centerSensor && 6 == rightSensor  ){
      Serial.println("G");
  forward();
  //delay(230);
  //Stop();
  //delay(10000);
      
    }

    
     /*else if (7 < centerSensor && 7 < leftSensor && 7 < rightSensor){
    left();
      Serial.println("left");
    delay(500);
    }*/

}

void workF() {
  int centerSensor = sensorTwo();
  int leftSensor = sensorOne();
  int rightSensor = sensorThree();
  //int irsensor = irsensor();
// Check the distance using the IF condition
  if (6 >= centerSensor) {
    Stop();
    Serial.println("Stop");
    //delay(100);
    if (rightSensor > leftSensor) { 
      right();
      //digitalWrite(buzpin,HIGH);
      Serial.println("RIGHT");
      delay(50);
    } else {
      left();
      //digitalWrite(buzpin,HIGH);
      Serial.println("LEFT");
      delay(50);
    }
  }
  else if (6 <= centerSensor && leftSensor > rightSensor){
      left();
      Serial.println("E");
    }
  else if (6 <= centerSensor && leftSensor < rightSensor){
      right();
      Serial.println("F");   
    }
  else if (6 <= centerSensor && leftSensor == rightSensor){
      Serial.println("G");
      forward();
  //delay(230);
  //Stop();
  //delay(10000);
      
    }

  //digitalWrite(buzpin,LOW);
  //Serial.println(irsensor);
 
}