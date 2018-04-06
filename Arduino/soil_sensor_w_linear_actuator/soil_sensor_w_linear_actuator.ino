// ROS Setup Stuff
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <SHT1x.h>
#define dataPin 10
#define clockPin 11

// Serial Communication Variables
bool msgReceived = false;   // Flag to indicate receipt of message
String inString;            // Store user input as string
char inChar;

// Sensor variables
float tempC = 0;    // Temperature in Celsius (-40 - 123.8, -40 on error)
float tempF = 0;    // Temperature in Fahrenheit (-40 - 254.9, -40 on error)
float humidity = 0; // Humidity (0-100%, negative on error)

// Initialize sensor object
SHT1x mySensor(dataPin, clockPin);


ros::NodeHandle nh;

std_msgs::Float32 temp_msg;
std_msgs::Float32 humid_msg;

ros::Publisher pub_temp("temperature", &temp_msg);
ros::Publisher pub_humid("humidity", &humid_msg);

// For linear actuator
int incomingByte = 0;   // for incoming serial data

void setup() 
{

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
  
  // Initialize Serial communications
  Serial.begin(57600);
  
  // Prompt user with instructions
  Serial.println("Initializing SHT10 Temp/Humidity Sensor...");
  Serial.println("Type any of the following commands, followed by the ENTER key:");
  Serial.println("\t'C/c': Display temperature in degrees Celsius");
  Serial.println("\t'F/f': Display temperature in degrees Fahrenheit");
  Serial.println("\t'H/h': Display humidity (0-100%)");
  Serial.println("\t'': Display all available sensor information");

  
  
}

void loop() 
{

  // Print the temperature info in degrees celsius
  tempC = mySensor.readTemperatureC();
  Serial.print("Temperature: "); Serial.print(tempC);
  Serial.println(" C");

  humidity = mySensor.readHumidity();
  Serial.print("Humidity: "); Serial.print(humidity);
  Serial.println("%");

  // ROS stuff
  temp_msg.data = tempC;
  humid_msg.data = humidity;
  pub_temp.publish(&temp_msg);
  pub_humid.publish(&humid_msg);

  //nh.spinOnce(); // Placing this here will not make it work
  
  if(msgReceived) 
  {
    
    // Display all sensor data
    if(inString.length() == 0) 
    {
      tempC = mySensor.readTemperatureC();
      tempF = mySensor.readTemperatureF();
      humidity = mySensor.readHumidity();

      Serial.print("Humidity: "); Serial.print(humidity);
      Serial.println("%");
      Serial.print("Temperature: "); Serial.print(tempC);
      Serial.println(" C");
      Serial.print("Temperature: "); Serial.print(tempF);
      Serial.println(" F");
    }
    // Check for desired reading (degrees C, degrees F, or humidity)
    if(inString.charAt(0) == 'C' || inString.charAt(0) == 'c') 
    {
      tempC = mySensor.readTemperatureC();
      Serial.print("Temperature: "); Serial.print(tempC);
      Serial.println(" C");
    } 
    else if(inString.charAt(0) == 'F' || inString.charAt(0) == 'f') 
    {
      tempF = mySensor.readTemperatureF();
      Serial.print("Temperature: "); Serial.print(tempF);
      Serial.println(" F");
    }
    else if(inString.charAt(0) == 'H' || inString.charAt(0) == 'h') 
    {
      humidity = mySensor.readHumidity();
      Serial.print("Humidity: "); Serial.print(humidity);
      Serial.println("%");
    }
    
    // Reset Serial communication variables
    msgReceived = false;
    inString = "";
  }

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

  nh.spinOnce(); // Place this at the end
}

/*
  SerialEvent occurs whenever new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
 */
void serialEvent() 
{
  // Read from Rx until end of data stream, or until '\n' is encountered
  while (Serial.available() && !msgReceived) 
  {
    inChar = (char)Serial.read();          // Get the next byte
    
    // Check for a newline, indicating the end of the message
    if(inChar == '\n') 
    {
      msgReceived = true;    // Set flag to break out of loop and handle message
    } 
    else 
    {
      inString += inChar;    // Append current character to the input string
    }
  }
}
