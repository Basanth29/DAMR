

/*
Connections of Drive and Arduino
Serial Port 0 is not used to connect to drive because its connected to USB-Serial and used to show information on console.

For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
   Drive to Arduino UNO/Nano connections
   GND         -      GND
   RXD         -      D3
   TXD         -      D2

For arduino mega and other arduinos with multiple hardware serial port, any port other than 0 can be selected to connect the drive.

   Drive to Arduino Mega2560 connections
   GND         -      GND
   RXD         -      Tx1/Tx2/Tx3
   TXD         -      Rx1/Rx2/Rx3
   
*  This mode can be used when multiple motors are to be used to run at exactly the same RPM and same torque even though the voltage supply might be different.
* Also in this mode the direction of the motor can be controlled digitally via modbus ASCII commands to run the dc servo motor in both directions

* For more information see : https://robokits.co.in/motor-drives-drivers/encoder-dc-servo/rhino-dc-servo-driver-50w-compatible-with-modbus-uart-ascii-for-encoder-dc-servo-motor


*/

#include<RMCS2303drive.h>

RMCS2303 rmcs;                    //object for class RMCS2303

SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.

//parameter Settings "Refer datasheet for details"
byte slave_id_1=1;
byte slave_id_2=2;
byte slave_id_3=3;
byte slave_id_4=4;
int INP_CONTROL_MODE=257;           
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=334;
int acceleration=5000;
int speed=8000;


long int Current_position;
long int Current_Speed;

void setup()
{
   rmcs.Serial_selection(1);       //Serial port selection:0-Hardware serial,1-Software serial
   rmcs.Serial0(9600);             //Set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Speed control mode demo\r\n\r\n");

   //rmcs.begin(&Serial1,9600);    
   rmcs.begin(&myserial,9600);     
   rmcs.WRITE_PARAMETER(slave_id_1,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    
   rmcs.READ_PARAMETER(slave_id_1); 
   rmcs.WRITE_PARAMETER(slave_id_2,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    
   rmcs.READ_PARAMETER(slave_id_2); 
   rmcs.WRITE_PARAMETER(slave_id_3,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    
   rmcs.READ_PARAMETER(slave_id_3); 
   rmcs.WRITE_PARAMETER(slave_id_4,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    
   rmcs.READ_PARAMETER(slave_id_4); 
   
}

void straight(int f,int b){
     rmcs.Enable_Digital_Mode(slave_id_1,f);         
                     
   rmcs.Enable_Digital_Mode(slave_id_2,b);        
                     
   rmcs.Enable_Digital_Mode(slave_id_3,f);
                   
   rmcs.Enable_Digital_Mode(slave_id_4,b);
  
  

}

void give_speed(int a){
  rmcs.Speed(slave_id_1,a); 
  rmcs.Speed(slave_id_2,a); 
  rmcs.Speed(slave_id_3,a); 
  rmcs.Speed(slave_id_4,a);   
}
void feedback(void){
  
  Current_Speed=rmcs.Speed_Feedback(slave_id_1);
  Current_Speed=rmcs.Speed_Feedback(slave_id_2);
  Current_Speed=rmcs.Speed_Feedback(slave_id_3);
  Current_Speed=rmcs.Speed_Feedback(slave_id_4);   
} 

void break_motor(void){
  rmcs.Brake_Motor(slave_id_1,0);
  rmcs.Brake_Motor(slave_id_2,1);                 
  rmcs.Brake_Motor(slave_id_3,0);
  rmcs.Brake_Motor(slave_id_4,1);  
}

void disable(void){
  rmcs.Disable_Digital_Mode(slave_id_1,1);        
  rmcs.Disable_Digital_Mode(slave_id_2,1);
  rmcs.Disable_Digital_Mode(slave_id_3,0);
  rmcs.Disable_Digital_Mode(slave_id_4,0);    
}

void loop(void)
{
   Serial.println("Sending speed command - 8000 RPM");
   give_speed(speed);
   delay(1000);
   Serial.println("Starting the motors with given speed");                  
   straight(1,0);
   delay(3000);
   feedback();
   delay(5000);
   Serial.println("Break Motor");
   break_motor();
   delay(2000);
   Serial.println("Changing direction & Starting the motors");
   straight(0,1); //reverse direction
   delay(3000);
   feedback();
   delay(3000);
   Serial.println("Disable Motor");

   delay(3000);
}
