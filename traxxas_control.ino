#include <Servo.h> 

Servo ESC;
Servo steering;

String val = "";
String val_serial = "";
int val_final = 0;


const int BUFFER_SIZE = 10;
char buf[BUFFER_SIZE];

void setup() {
  Serial.begin(115200);
  ESC.attach(9);
  steering.attach(5);
  
  ESC.write(1600);
  steering.write(92.5);
  
}

void loop() {
 
  if (val_serial.charAt(0) == 'e'){
    
    if(val_final >= 1300 && val_final <= 1600){
      ESC.write(val_final);
      
      Serial.print("Reverse motion: turning ESC to ");
      Serial.println(val_final);
    }
  
    else if(val_final > 1600 && val_final <= 1800){
      ESC.write(val_final);
      
      Serial.print("Forward motion: turning ESC to ");
      Serial.println(val_final);
    }
    
    else{
      Serial.println("cannot execute command, due to the following value");
      Serial.println(val_final);  
    }
 
  }
  
  else if(val_serial.charAt(0) == 's'){
    
    if (val_final >= 55 && val_final <= 130){
      steering.write(val_final); 
      
      Serial.print("turning steering to ");
      Serial.println(val_final);
    }
     
    else{ 
      Serial.println("cannot execute command, due to the following value");
      Serial.println(val_final);  
    }  
  }
}

void serialEvent() {
  
    if(Serial.available()){
      
      Serial.println("serial available oldugunda");
      
      val_serial = Serial.readStringUntil('f');
      
      Serial.setTimeout(0.01);
  
      val = val_serial.substring(1,val_serial.length());
      val_final = val.toInt();
    }
}

