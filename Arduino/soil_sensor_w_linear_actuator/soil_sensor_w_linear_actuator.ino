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

#include <geometry_msgs/Twist.h>

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
//void actuatorCb(const std_msgs::Float32& cmd_msg)
void actuatorCb(const geometry_msgs::Twist& cmd_msg) // for Twist msg
{

  // Pre-Teleop
  //actuator_msg.data = cmd_msg.data;
  //linear_actuator_cmd_echo_msg.data = cmd_msg.data;

  // For Teleop Msg Parsing
  // Access the x variable in the data
  actuator_msg.data = cmd_msg.linear.x;
  linear_actuator_cmd_echo_msg.data = cmd_msg.linear.x;
}

// Create the subscriber before void setup
//ros::Subscriber<std_msgs::Float32> sub("linear_actuator_cmd", &actuatorCb );
ros::Subscriber<geometry_msgs::Twist> sub("linear_actuator_cmd", &actuatorCb );

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
  pub_linear_command_echo.publish(&linear_actuator_cmd_echo_msg); // Topic to ensure linear_actuator_cmd msg is received
  
  //Serial.println(linear_actuator_cmd);
  // For ROS
  // 1
  //if (linear_actuator_cmd == 11)// Move down
  
  // Swap to using teleop
  if (linear_actuator_cmd < 0)// Move down
  {
    analogWrite(PWMA, 64);
    digitalWrite(AIN1, HIGH);       // sets the digital pin 13 on
    digitalWrite(AIN2, LOW);        // sets the digital pin 13 off
    
  }// End else if for moving down
  
  // 2
  //else if (linear_actuator_cmd == 12) // Move up
  else if (linear_actuator_cmd > 0) // Move up
  {
    analogWrite(PWMA, 64);
    digitalWrite(AIN1, LOW);       // sets the digital pin 13 on
    digitalWrite(AIN2, HIGH);  // LOW, LOW -> Freeze, HIGH, HIGH -> Freeze
  } // End else if for moving up

  // 0: By Default, it should stop unless a signal is given
  else // if (linear_actuator_cmd == 10) // Stop
  {
    analogWrite(PWMA, 0);
  } // End else

  // After publishing the echo message.
  // Set it to be 0, so that a continuous stream of linear_actuator_cmd messages have to be sent
  // If it is not set to non-zero in the next time step, the else would trigger and turn off the motor
  linear_actuator_cmd_echo_msg.data = 0;
  
  //linear_actuator_cmd_echo_msg.linear.x = 0;
  
  // Reset the variable for the linear actuator cmd to 0, so there needs to be a continuous stream of data for it to actuate the motor
  // linear_actuator_cmd = 0; This does not work, as actuator_msg.data is still the old value, linear_actuator_cmd takes value from actuator_msg.data
  actuator_msg.data = 0;
  
  //actuator_msg.linear.x = 0; // for teleop
  

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

    // The code in here is to test the logic when using Serial monitor to send the inputs into the ProMini
    // Instead of using a ROS topic
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

    // If Serial Testing used...Earlier intermediate testing code
    // Reset incoming Byte to 0 at end of receiving the message
    incomingByte = 0;
  } // End Serial.available()
  
  //if (incomingByte != 48) or (incomingByte != 49)
  //{
  //  analogWrite(PWMA, 0);
  //}

  // For Debugging
  // Waiting for Serial Input
  //Serial.println("Waiting for Serial Input");
  //analogWrite(PWMA, 128); // Use servo, analogWrite and servo both work
  //myservo.write(179);   
  // put your main code here, to run repeatedly:

  //nh.spinOnce();
  
} // End of Main Loop
