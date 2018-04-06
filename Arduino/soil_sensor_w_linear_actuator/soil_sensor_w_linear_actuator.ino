/**
 * ReadSHT1xValues
 *
 * Read temperature and humidity values from an SHT1x-series (SHT10,
 * SHT11, SHT15) sensor.
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
 * www.practicalarduino.com
 */

//v3 changelog
// Added subscriber to listen to topics published by RPI (ROS) to control linear actuator

// ROS Setup Stuff
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Uint8.h>

#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  10
#define clockPin 11
SHT1x sht1x(dataPin, clockPin);

ros::NodeHandle nh;
 
std_msgs::Float32 temp_msg;  // To publish temp information
std_msgs::Float32 humid_msg; // To publish humidity information
std_msgs::Float32 linear_actuator_cmd_echo_msg; // To publish humidity information

std_msgs::Float32 actuator_msg; // To actuator motor

ros::Publisher pub_temp("temperature", &temp_msg);
ros::Publisher pub_humid("humidity", &humid_msg);
ros::Publisher pub_linear_command_echo("Linear_Actuator_Command_Echo", &linear_actuator_cmd_echo_msg);

// For linear actuator
int incomingByte = 0;   // for incoming serial data

// Create the callback function for the subscriber
void actuatorCb(const std_msgs::Float32& cmd_msg)
{
  actuator_msg.data = cmd_msg.data;
  linear_actuator_cmd_echo_msg.data = cmd_msg.data;
}

// Create the subscriber before void setup
ros::Subscriber<std_msgs::Float32> sub("linear_actuator_cmd", &actuatorCb );

void setup()
{

   // Initialize ROS node
   nh.initNode();
   nh.advertise(pub_temp);
   nh.advertise(pub_humid);
   nh.advertise(pub_linear_command_echo);
   nh.subscribe(sub); // Need to subscribe to subscription in void setup
   
   //Serial.begin(57600); // Open serial connection to report values to host
   //Serial.println("Starting up");

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

  float linear_actuator_cmd = actuator_msg.data;

  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  //temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();

// Serial Stuff commented out
/*
  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  //Serial.print(temp_f, DEC);
  //Serial.print("F. Humidity: ");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
*/
  // ROS stuff
  temp_msg.data = temp_c;
  humid_msg.data = humidity;
  pub_temp.publish(&temp_msg);
  pub_humid.publish(&humid_msg);
  pub_linear_command_echo.publish(&linear_actuator_cmd_echo_msg);

  //Serial.println(linear_actuator_cmd);
  // For ROS
  // 1
  if (linear_actuator_cmd == 11)// Move down
  {
    analogWrite(PWMA, 64);
    digitalWrite(AIN1, HIGH);       // sets the digital pin 13 on
    digitalWrite(AIN2, LOW);        // sets the digital pin 13 off

  }

  // 0
  else if (linear_actuator_cmd == 10) // Stop
  {
    analogWrite(PWMA, 0);
  }

  // 2
  else if (linear_actuator_cmd == 12) // Move up
  {
    analogWrite(PWMA, 64);
    digitalWrite(AIN1, LOW);       // sets the digital pin 13 on
    digitalWrite(AIN2, HIGH);  // LOW, LOW -> Freeze, HIGH, HIGH -> Freeze
  }

  // Test 
  //nh.loginfo("Linear Actuator Cmd: %d", linear_actuator_cmd);
  
  nh.spinOnce(); //Placing this line here prevents it from working - The Arduino Serial part...

  //delay(2000);

  // For Arduino Testing
  // send data only when you receive data:
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);

    // 1
    if (incomingByte == 49)// Move down
    {
      analogWrite(PWMA, 64);
      digitalWrite(AIN1, HIGH);       // sets the digital pin 13 on
      digitalWrite(AIN2, LOW);        // sets the digital pin 13 off

    }

    // 2: 50 - 48 = 2 (ASCII to dec conversion)
    else if (incomingByte == 50) // Move up
    {
      analogWrite(PWMA, 64);
      digitalWrite(AIN1, LOW);       // sets the digital pin 13 on
      digitalWrite(AIN2, HIGH);  // LOW, LOW -> Freeze, HIGH, HIGH -> Freeze
    }
    
    // By Default, if there is no signal. Stop
    // 0
    else // else if (incomingByte == 48) // Stop
    {
      analogWrite(PWMA, 0);
    }
    
  } // End Serial.available()

  // For Debugging
  // Waiting for Serial Input
  //Serial.println("Waiting for Serial Input");
  //analogWrite(PWMA, 128); // Use servo, analogWrite and servo both work
  //myservo.write(179);   
  // put your main code here, to run repeatedly:

  //nh.spinOnce();
  
}
