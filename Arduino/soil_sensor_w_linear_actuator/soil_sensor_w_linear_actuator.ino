/**
 * ReadSHT1xValues
 *
 * Read temperature and humidity values from an SHT1x-series (SHT10,
 * SHT11, SHT15) sensor.
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
 * www.practicalarduino.com
 */
// ROS Setup Stuff
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  10
#define clockPin 11
SHT1x sht1x(dataPin, clockPin);

ros::NodeHandle nh;

std_msgs::Float32 temp_msg;
std_msgs::Float32 humid_msg;

ros::Publisher pub_temp("temperature", &temp_msg);
ros::Publisher pub_humid("humidity", &humid_msg);

// For linear actuator
int incomingByte = 0;   // for incoming serial data

void setup()
{
   Serial.begin(57600); // Open serial connection to report values to host
   Serial.println("Starting up");

   // Initialize ROS node
   nh.initNode();
   nh.advertise(pub_temp);
   nh.advertise(pub_humid);

   // Setup code for linear actuator
   // put your setup code here, to run once:
   #define AIN1 4
   //#define BIN1 7 
   #define AIN2 5
   //#define BIN2 8
   #define PWMA 6
   //myservo.attach(6);  // attaches the servo on pin 9 to the servo object
   //#define PWMB 9 
   #define STBY 9
   pinMode(AIN1, OUTPUT);
   pinMode(AIN2, OUTPUT);
   
}

void loop()
{
  float temp_c;
  float temp_f;
  float humidity;

  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  //temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  //Serial.print(temp_f, DEC);
  //Serial.print("F. Humidity: ");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  // ROS stuff
  temp_msg.data = temp_c;
  humid_msg.data = humidity;
  pub_temp.publish(&temp_msg);
  pub_humid.publish(&humid_msg);

  //nh.spinOnce(); Placing this line here prevents it from working

  //delay(2000);

  // send data only when you receive data:
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);

    // 1
    if (incomingByte == 49)// Move down
    {
      analogWrite(PWMA, 64);
      digitalWrite(AIN1, HIGH);       // sets the digital pin 13 on
      digitalWrite(AIN2, LOW);        // sets the digital pin 13 off

    }

    // 0
    else if (incomingByte == 48) // Stop
    {
      analogWrite(PWMA, 0);
    }

    // 2
    else if (incomingByte == 50) // Move up
    {
      analogWrite(PWMA, 64);
      digitalWrite(AIN1, LOW);       // sets the digital pin 13 on
      digitalWrite(AIN2, HIGH);  // LOW, LOW -> Freeze, HIGH, HIGH -> Freeze
    }
  }

  // Waiting for Serial Input
  Serial.println("Waiting for Serial Input");
  //analogWrite(PWMA, 128); // Use servo, analogWrite and servo both work
  //myservo.write(179);   
  // put your main code here, to run repeatedly:

  nh.spinOnce();
  
}
