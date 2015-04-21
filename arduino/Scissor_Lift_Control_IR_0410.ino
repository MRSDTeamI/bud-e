#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

int relay1 = 7;
int relay2 = 5;
int relay3 = 4;
int relay4 = 3;

const int ir_pin= A0;  
float ir_distance ;    
float ir_volt;         

#define arr_size 11    
int measured_vals[arr_size] = {0,0,0,0,0,0,0,0,0,0,0};
int filter_result =0;

unsigned long start, finished, elapsed, exe_period;
unsigned long ini_pos, des_pos, tmp_pos;

void get_height( const std_msgs::UInt16& cmd_msg)
{
  
  des_pos = cmd_msg.data + 50;
  if(des_pos != ini_pos)
  
  {    
    if(des_pos > ini_pos)
    {      
      Serial.println("start up");
      tmp_pos = get_IRdistance_filtered();
      
      Serial.println(tmp_pos);
      while(abs(des_pos-tmp_pos)>1 && des_pos >= tmp_pos)
      
      {        
        digitalWrite(relay2, HIGH);
        
        digitalWrite(relay3, HIGH);
        digitalWrite(relay1, LOW);
        digitalWrite(relay4, LOW);       
        //delay(1000);   
        
        tmp_pos = get_IRdistance_filtered();
        Serial.println(tmp_pos);
      }
      
      Serial.println("finish up");
    }
    else 
    {        
        Serial.println("start down");
        tmp_pos = get_IRdistance_filtered();
        Serial.println(tmp_pos);
        
        while(abs(des_pos-tmp_pos)>1 && tmp_pos >= des_pos)
        
        {
          
          digitalWrite(relay1, HIGH);
          digitalWrite(relay4, HIGH);
          digitalWrite(relay2, LOW);
          digitalWrite(relay3, LOW);    
          //delay(1000);
          tmp_pos = get_IRdistance_filtered();
          Serial.println(tmp_pos);
        }      
        Serial.println("finish down");
     }
     ini_pos = des_pos;
     des_pos =0;   
     tmp_pos =0;
     digitalWrite(relay1, HIGH);
     digitalWrite(relay2, HIGH);
     digitalWrite(relay3, HIGH);
     digitalWrite(relay4, HIGH);  
  }
}

ros::Subscriber<std_msgs::UInt16> sub("target_height", &get_height);



void setup()        
{  
  nh.initNode(); 
  nh.subscribe(sub);
  //ini_pos =70 ;
  
  Serial.begin(57600);
  Serial.read(); 
  
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  // initialize output pins  
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
  initialize_pos();
}

void loop()   
{  
  nh.spinOnce();    
  delay(200);   
  
}

void initialize_pos()
{
  des_pos = 50;
  ini_pos = get_IRdistance_filtered();
  
  if(des_pos != ini_pos)
  {    
    if(des_pos > ini_pos)
    {      
      Serial.println("start up");
      tmp_pos = get_IRdistance_filtered();
      while(abs(des_pos-tmp_pos)>1 && des_pos >= tmp_pos)
      {
        
        digitalWrite(relay2, HIGH);
        digitalWrite(relay3, HIGH);
        digitalWrite(relay1, LOW);
        digitalWrite(relay4, LOW);        
        //delay(1000);
        tmp_pos = get_IRdistance_filtered();
      }
      Serial.println("finish up");
    }
    else 
    {        
        Serial.println("start down");
        tmp_pos = get_IRdistance_filtered();
        while( abs(des_pos-tmp_pos)>1 && tmp_pos >= des_pos)
        {          
          digitalWrite(relay1, HIGH);
          digitalWrite(relay4, HIGH);
          digitalWrite(relay2, LOW);
          digitalWrite(relay3, LOW);    
          //delay(1000);
          tmp_pos = get_IRdistance_filtered();          
        }      
        Serial.println("finish down");
     }
     ini_pos = des_pos;
     des_pos =0;   
     tmp_pos =0;
     digitalWrite(relay1, HIGH);
     digitalWrite(relay2, HIGH);
     digitalWrite(relay3, HIGH);
     digitalWrite(relay4, HIGH);  
  }  
}


int get_IRdistance_filtered() {
  // Clear array
  memset(measured_vals,0,sizeof(measured_vals));
  for( int i = 0; i < arr_size; i++) {
    // Pulse width representation.
    // Scale factor of 147 uS per Inch 
    ir_volt = analogRead(ir_pin)*0.0048828;  
    ir_distance = 65 * pow(ir_volt, -1.10);
    //Serial.println(ir_distance);   
    measured_vals[i] = ir_distance;
    //sum= sum + measured_vals[i];
    delay(40);
  } 
 
  sort_array(arr_size, measured_vals);
  filter_result = mode(arr_size, measured_vals); 
  filter_result = filter_result-6;
  //Serial.println(filter_result);  
  //filter_result = sum / arr_size;
  if(filter_result >150) filter_result =150;
  else if(filter_result <20) filter_result =20;
  
  Serial.println(filter_result);  
  return filter_result;
}

void sort_array(int array_size, int *val_array) {
 for (int i = 1; i < array_size; i++) {
   int j = val_array[i];
   int k;
   for (k = i - 1; (k >= 0) && (j < val_array[k]); k--) {
     val_array[k+1] = val_array[k];
   }
   val_array[k+1] = j;
 }
}

int mode(int array_size,int
*val_array){
 int i = 0;
 int count = 0;
 int maxCount = 0;
 int mode = 0;
 int bimodal;
 int prevCount = 0;
 while(i < (array_size-1)){
   prevCount = count;
   count = 0;
   while(val_array[i] == val_array[i+1]){
     count++;
     i++;
     
     
   }
   if(count > prevCount && count > maxCount){
     mode = val_array[i];
     maxCount = count;
     bimodal = 0;
   }
   if(count == 0){
     i++;
   }
   if(count == maxCount){      //If the dataset has 2 or more modes.
     bimodal = 1;
   }
   //Return the median if there is no mode.
   if(mode == 0 || bimodal == 1){
     mode = val_array[(array_size/2)];
   }
   return mode;
 }
}

