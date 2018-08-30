//Libraries

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <DHT.h>
#include <QList.h>


//Constants
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/// ROS initialize
ros::NodeHandle cb;

std_msgs::String test;
std_msgs::Float32 case_temperature;
std_msgs::Float32 case_humidity;
std_msgs::Float32 fan_state_speed;
std_msgs::String hum_warning;

//ros::publisher test_string("/tester
ros::Publisher case_temp("Box_Temperature", &case_temperature);
ros::Publisher case_hum("Box_Humidity", &case_humidity);
ros::Publisher fan_state("Fan_Status", &fan_state_speed);


//Variables
float hum;  //Stores humidity value
float temp; //Stores temperature value
float dutyCycle; //Store the fan duty cycle
float fanSpeed; //Store fan speed
  
float highTime = 0;
float lowTime = 0;
float setTempMin = 22;
float setTempMax = 25;
float pwmHigh;
float setPoint = 23; //degrees celcius
float error;
float kp = 70;
float ki = 0.5;
int count = 0;


// Sena added Integral control: 8th May 2018:
QList<float> queueList;

void setup()
{
  cb.initNode();
  cb.advertise(case_temp);
  cb.advertise(case_hum);
  cb.advertise(fan_state);
  
  dht.begin();  //initialize DHT sensor
  pinMode(3, OUTPUT); //initialize pin for fan PWM control
  pinMode(4, INPUT);  //initialize pin for reading fan tachymetry
  Serial.begin(9600);
 }

void loop()
{

  count++;
  
///  read data and store humidity and temperature into hum and temp
  temp = dht.readTemperature();
  hum = dht.readHumidity();
  
//  simple PI controller

  error = temp - setPoint;
  
  if (error >= 0)
  {
    error = error;
  }
  else
  {    
    error = 0;
  }

  queueList.push_front(error); // Push item at the front of the list

  if (count > 5){
    queueList.pop_back();
  }

  float integral = 0;

  for (int i=0;i< queueList.length();i++){

    integral += queueList[i];
    }

  pwmHigh = kp*error + ki*integral;

  if (pwmHigh > 1000)
  {
    pwmHigh = 1000;
  }

//  Serial.print("Pulse");
//  Serial.println(pwmHigh);
//  Serial.print("temp = ");
//  Serial.println(temp);
    Serial.print("humidity = ");
    Serial.println(hum);
//  Serial.print("error");
//  Serial.println(error);
//  Serial.print("Int =");
//  Serial.println(integral);
    

//  Run the fan at Duty Cycle
analogWrite(3,pwmHigh);

  case_temperature.data = temp;
  case_humidity.data = hum;
  fan_state_speed.data = (pwmHigh / 1000)*100;
  case_temp.publish( &case_temperature);
  case_hum.publish( &case_humidity);
  fan_state.publish(&fan_state_speed);
  cb.spinOnce();
  
  
delay(2000);
}
