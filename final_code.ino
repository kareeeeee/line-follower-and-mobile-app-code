#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

#define button 35

//IR sensors
#define leftsensor 13
#define centersensor 14
#define rightsensor 27

int forward =1;
int right=1;
int left=1;

int leftsensor_read;
int centersensor_read;
int rightsensor_read;

//Enable for the 2motosr of directions move
#define EA1 22 
#define EB1 23

//Enable for the 2motosr of the arm
#define EA2 32
#define EB2 33

//pins for 74HC595
#define dataPin 21  
#define clockPin 19 
#define latchPin 18 

//stepper
#define enablepin 4
#define directionpin 26
#define steppin 25
#define stepsPerRevolution 6400

//Functions
void shiftOutData(byte data);
void move_forward();
void turn_right();
void turn_left();
void move_back();
void U_turn();
void stop_motors();
void motor1_arm_forward();
void motor1_arm_back();
void motor2_arm_forward();
void motor2_arm_back();
void rotate_stepper();
void rotate_stepper_inverse();
void stop_stepper();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  SerialBT.begin("ESP32_Bluetooth");

  //converting button
  pinMode(button,INPUT);
  
  //IR sensors
  pinMode(leftsensor, INPUT);
  pinMode(centersensor, INPUT);
  pinMode(rightsensor, INPUT);

//Enable for the 4 motors
  pinMode(EA1, OUTPUT);
  pinMode(EB1, OUTPUT);
  pinMode(EA2, OUTPUT);
  pinMode(EB2, OUTPUT);

//pins for the 74HC595
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  //stepper
  pinMode(enablepin,OUTPUT);
  pinMode(directionpin,OUTPUT);
  pinMode(steppin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int state =digitalRead(button);


  if(state ==0){
    //Line Follower
   leftsensor_read =digitalRead(leftsensor);
   centersensor_read =digitalRead(centersensor);
   rightsensor_read =digitalRead(rightsensor);
  
  //conditions
  if(leftsensor_read ==1 && centersensor_read ==0 && rightsensor_read ==1)    
  {
    move_forward();   //done
  } 

  if(leftsensor_read ==1 && centersensor_read ==1 && rightsensor_read ==0)
  {
    turn_right();     //done 
  }

  if(leftsensor_read ==0 && centersensor_read ==1 && rightsensor_read ==1)
  { 
    turn_left();      //done
  }

  if(leftsensor_read ==1 && centersensor_read ==1 && rightsensor_read ==1)
  {
    U_turn();       //done

  }

  if( (leftsensor_read ==1 && centersensor_read ==0 && rightsensor_read ==0)  || (leftsensor_read ==0 && centersensor_read ==0 && rightsensor_read ==1))
  {
               //Right or Forward(Available)                                   //Left or Forward(Available)  
                    //                                                                //
                    //                                                                //
                    //                                                                // 
                    //                                                                //
                    //////////                                                /////////                     
                                                                                                            //
    turn_right();  //can add a delay with the value of 90 deg.                                              //
  }                                                                                                         //
                                                                                                            //
  if(leftsensor_read ==0 && centersensor_read ==0 && rightsensor_read ==0)                    ///////////////////////////////
  {
    //Left or Forward or Right or END(Available)
    if(forward == 1){
      move_forward();
      delay(1000);
      leftsensor_read =digitalRead(leftsensor);
      centersensor_read =digitalRead(centersensor);
      rightsensor_read =digitalRead(rightsensor);
      if(leftsensor_read ==1 && centersensor_read ==1 && rightsensor_read ==1){
        move_back();
        delay(1000);
        forward=0;}
      else if(forward ==1 ){
        move_forward();
        right=1;
        left=1;}              
                    }


    if(forward ==0){
      turn_right();
      delay(1000);  //Test with 90 degree
      move_forward();
      delay(1000);
      leftsensor_read =digitalRead(leftsensor);
      centersensor_read =digitalRead(centersensor);
      rightsensor_read =digitalRead(rightsensor);
      if(leftsensor_read ==1 && centersensor_read ==1 && rightsensor_read ==1){
        move_back();
        delay(1000);
        turn_left();
        delay(1000);  //test with 90 deg.
        right =0;} 
      else if(right ==1){
        move_forward();
        forward=1;
        left=1;}               
                        }

    if(right==0){
      turn_left();
      delay(1000);    //test with 90 deg.
      move_forward();
      delay(1000);
      leftsensor_read =digitalRead(leftsensor);
      centersensor_read =digitalRead(centersensor);
      rightsensor_read =digitalRead(rightsensor);
      if(leftsensor_read ==1 && centersensor_read ==1 && rightsensor_read ==1){
        move_back();
        delay(1000);
        turn_right();
        delay(1000);  //test with 90
        left =0;}
      else if(left ==1 ){
        move_forward();
        forward=1;
        right=1;}
                      }
    
    if(forward ==0 && right ==0 && left ==0){
      stop_motors();  //End of line follower
      }
    }}


    else if(state ==1){
      //Bluetooth
      
  if(SerialBT.available() >=2){
    unsigned int byte_one =SerialBT.read();
    unsigned int byte_two =SerialBT.read();
    unsigned int digit_number =(byte_two * 256)+ byte_one;
    Serial.println(digit_number);

      if(digit_number == 1){
        turn_left();
      }

      if(digit_number == 2){
        move_forward();
      }

      if(digit_number == 3){
        turn_right();
      }

      if(digit_number == 4){
        move_back();
      }

      if(digit_number == 5){
        stop_motors();
      }

      if(digit_number == 6){
        motor1_arm_forward();
      }

      if(digit_number == 7){
        motor1_arm_back();
      }

      if(digit_number == 8){
        motor2_arm_forward();
      }

      if(digit_number == 9){
        motor2_arm_back();
      }

      if(digit_number == 11){
        rotate_stepper();
      }

      if(digit_number == 12){
        rotate_stepper_inverse();
      }

      if(digit_number == 10){
        stop_motors();
      }

      if(digit_number == 13){
        stop_stepper();
    } }
  }
}


void move_forward(){
//motors with 74HC595 for directions of moving
  analogWrite(EA1,255);
  analogWrite(EB1,255);
  //forawrd
  shiftOutData(0b00000101);  
}

void turn_right(){
  //motors with 74HC595
  analogWrite(EA1,255);
  analogWrite(EB1,255);
  shiftOutData(0b00000110);  
}

void turn_left(){
  //motors with 74HC595
  analogWrite(EA1,255);
  analogWrite(EB1,255);
  //left
  shiftOutData(0b00001001);  
}


void move_back(){
//motors with 74HC595
  analogWrite(EA1,255);
  analogWrite(EB1,255);
  //back
  shiftOutData(0b00001010); 
}

void U_turn(){
    move_back();
    delay(500);
    do{
      turn_right();
    }while(centersensor_read == 1);
    }

void stop_motors(){
//motors with 74HC595
  analogWrite(EA1,0);
  analogWrite(EB1,0);
  analogWrite(EA2,0);
  analogWrite(EB2,0);
  //stop
  shiftOutData(0b00000000);  
}


//Motor1 arm forward
void motor1_arm_forward(){
//motors with 74HC595
  analogWrite(EA2,255);
  //forawrd
  shiftOutData(0b00010000);
}

//Motor1 arm back
void motor1_arm_back(){
//motors with 74HC595
  analogWrite(EA2,255);
  //back
  shiftOutData(0b00100000);
}


//Motor2 arm forward
void motor2_arm_forward(){
//motors with 74HC595
  analogWrite(EB2,255);
  //forawrd
  shiftOutData(0b01000000);
}

//Motor2 arm back
void motor2_arm_back(){
//motors with 74HC595
  analogWrite(EB2,255);
  //back
  shiftOutData(0b10000000);
}


void rotate_stepper(){
  for(int i=0; i<stepsPerRevolution ;i++){
  //enable the stepper with current
  digitalWrite(enablepin,LOW);
  //the direction you need
    digitalWrite(directionpin,HIGH);  
  //making it rotate with controlling the velocity 
    digitalWrite(steppin,HIGH);
    delayMicroseconds(500);
    digitalWrite(steppin,LOW);
    delayMicroseconds(500);
}}

void rotate_stepper_inverse(){
  for(int i=0; i<stepsPerRevolution ;i++){
  //enable the stepper with current
  digitalWrite(enablepin,LOW);
  //the direction you need
    digitalWrite(directionpin,LOW);  
  //making it rotate with controlling the velocity 
    digitalWrite(steppin,HIGH);
    delayMicroseconds(500);
    digitalWrite(steppin,LOW);
    delayMicroseconds(500);
}}

void stop_stepper(){
  digitalWrite(enablepin,HIGH);
}

void shiftOutData(byte data) {
  digitalWrite(latchPin, LOW);  
  shiftOut(dataPin, clockPin, MSBFIRST, data);  
  digitalWrite(latchPin, HIGH);  
}
