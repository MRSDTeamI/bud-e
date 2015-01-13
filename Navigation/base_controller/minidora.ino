#include <Servo.h>
  Servo posBack;
  Servo posFront;
  Servo driveR;
  Servo driveL;
  String SerialInput="";
  String Svalue;
  int value;
void setup()
{
  Serial.begin(9600);
  posBack.attach(7);// 1 dot
  posFront.attach(8);// 4 dot
  driveR.attach(9);// 2 dots
  driveL.attach(10);// 3 dots
}

/*
MiniDora EVX 2 (EVX 3014)
NOTE: Longer treads are the front treads; Shorter treads are the back treads.

driveR: Calibs for right side drive motors through the ESC.
FullSpeedForward:0-1349; x-y (x >> 10000)
Forward:1350(high speed)-1484 (lowest speed)
Braking:1485-1555
1556-1557: Not reliable
Reverse:1558(lowest speed)-1900 (highest speed)
FullSpeedReverse:1900-x

driveL: Calibs for left side drive motors through the ESC.
FullSpeedForward:1930-z
Forward:1572(slowest)-1930 (fastest)
Braking:1475-1570
1571-1572:Not reliable
Reverse:1474 (slowest)- 1320(fastest)
FullSpeedReverse:-1474
*/
void loop()
{
  
   if(Serial.available())
                              {
                               for(int i=1;i<=3;i++)
                               {
                                   SerialInput = Serial.readStringUntil(' ');
                                if(SerialInput!="")
                                {
                                  Serial.print("Processing Command:");Serial.print(SerialInput);Serial.println();
                                  char motor=SerialInput.charAt(0);
                                 // Serial.print("LED=");Serial.print(LED);Serial.println();
                                  for(int j=1;SerialInput.charAt(j)!='\0';j++)
                                   Svalue+=SerialInput.charAt(j);
                                  // Serial.print("Svalue=");Serial.print(Svalue);Serial.println();
                                   value=Svalue.toInt();//In case the value input is not in the range of 0-255, the input value is converted
                                  // into integer with truncation and written to the Pin(s).
                                  // Serial.print("value=");Serial.print(value);Serial.println();
                                   switch(motor)
                                       {
                                         case 'r': {Serial.print("Writing");Serial.print(value);Serial.print("to motor 'r' ");Serial.println();driveR.writeMicroseconds(value);break;}
                                         case 'l': {Serial.print("Writing");Serial.print(value);Serial.print("to motor 'l' ");Serial.println();driveL.writeMicroseconds(value);break;}
                                         //case 'b': {BlueValue=value;break;} 
                                        default: {Serial.print("Wrong Command. Please check your input");Serial.println();break;} 
                                       }
                                 }
                                 Svalue="";
                                }
                                
                                SerialInput="";
                              }
  
 
}  
  
