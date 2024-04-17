
#include <SPI.h>
#include <Ethernet.h>
#include <P1AM.h>
#include <Servo.h> 
//#include <cstring>
#include <string.h> 
#include <Arduino.h>


void SetStartingDistance(EthernetClient client);
void StretchSeal(double starting_measurement, double ending_measurement, EthernetClient client);

const float mmToInches = 0.0393701; // Conversion factor
const char P1_04AD_CONFIG[] = { 0x40, 0x00, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x21, 0x03, 0x00, 0x00, 0x22, 0x03, 0x00, 0x00, 0x23, 0x03 };


const int maxSubstrings = 10; // Maximum number of substrings
const int maxSubstringLength = 20; // Maximum length of each substring

// Define the pin numbers for the switches
const int left_switchPin = 2; 
const int right_switchPin = 3;
int terminal_speed = 6000;
int starting_speed = 1000;
const unsigned long heartbeatInterval = 10000; // 10 seconds
#define PUL_PIN 0    // Define the PUL pin
#define DIR_PIN 1    // Define the DIR pin

void splitString(const String &input, String output[], int &numSubstrings, char delimiter) {
    int delimiterIndex = -1;
    int lastIndex = 0;
    numSubstrings = 0;

    while ((delimiterIndex = input.indexOf(delimiter, lastIndex)) != -1) {
        if (numSubstrings < maxSubstrings) {
            output[numSubstrings++] = input.substring(lastIndex, delimiterIndex);
            lastIndex = delimiterIndex + 1;
        } else {
            // Array is full, break out of the loop
            break;
        }
    }

    // Add the last substring after the last delimiter
    if (numSubstrings < maxSubstrings) {
        output[numSubstrings++] = input.substring(lastIndex);
    }
}


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { //Use the Reserved MAC printed on the right-side label of your P1AM-ETH.
  0x60, 0x52, 0xD0, 0x07, 0x7F, 0xDF
};



IPAddress ip(192, 168, 1, 177); //IP Address our webpage will be at
// Define the IP address and port for the server
int port = 80; // Change this to your desired port number
EthernetServer server(port);
// Create an Ethernet server
unsigned long connectionTime = millis(); //used for heartbeat timeout

void setup() {

// while(!Serial); 
while(!P1.init()); //Wait for module sign-on
Ethernet.init(5); //CS pin for P1AM-ETH


  // Start the serial communication
  Serial.begin(115200);
P1.configureModule(P1_04AD_CONFIG, 1); //sends the config data to the module in slot 1

  // Initialize the Ethernet shield with a static IP address
  Ethernet.begin(mac, ip);

  // Start the server
  server.begin();

  Serial.print("Server is at ");
  Serial.println(Ethernet.localIP());

  // Set the switch pins as inputs
  pinMode(left_switchPin, INPUT);
  digitalWrite(left_switchPin, HIGH);  // Enable internal pull-up resistor

  pinMode(right_switchPin, INPUT);
  digitalWrite(right_switchPin, HIGH);  // Enable internal pull-up resistor

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}


 double startLocation, endLocation;

void loop() {
  // Wait for a client to connect
  EthernetClient client = server.available();

// if (millis() - connectionTime > 5000) { // Adjust the timeout as needed
//           Serial.println("Client disconnected (timeout)");
//           Serial.println("Millis:" + String(millis()));
//           Serial.println("Connection Time: " + String(connectionTime));
//           client.stop();
//           delay(1000);
//       }
  if (client) {
    Serial.println("Client connected");
    

    // Read data from the client
    String receivedData = "";
    while (client.connected()) {

  
      if (client.available()) {
        char c = client.read();

        //end when new line is sent
        if (c == '\n') {
//           client.write("Received Message");
           Serial.println("Received data: " + receivedData);
           //parse the received data 
          // Vector to store the substrings
          
          String substrings[maxSubstrings];
          int numSubstrings;

          // Split the string using space as the delimiter
           splitString(receivedData, substrings, numSubstrings, ' ');
            receivedData  = "";
           //print substrings
            for (int i = 0; i < numSubstrings; ++i) {
              Serial.println(substrings[i]);
             }
             

             if (substrings[0] == "setup_new_part"){
//                client.stop();//close and then reopen in SetStartingDistance function for scope
                Serial.println("Calling routine to do new part");
                SetStartingDistance(client);
              }
              else if (substrings[0] == "Seal_Data"){
                Serial.println("Calling routine to stretch part");
                double starting_measurement = substrings[3].toDouble();
                double ending_measurement = substrings[6].toDouble();
                double stretching_speed = substrings[8].toDouble();
//                client.stop();
//                Serial.println("Client disconnected");
                StretchSeal(starting_measurement, ending_measurement, stretching_speed, client);
              }

              if (substrings[0] == "heartbeat"){
                connectionTime = millis();
              }
             
           delay(500);
           break;
        }
        else {
          receivedData += c;
        }

       
     
        
      }

    }


    // Close the connection (Commenting this out seems to keep the connection open and doesn't seem to cause issues...)
//   client.stop();
//   Serial.println("Client disconnected");
  }
}




void SetStartingDistance(EthernetClient client)
{
double starting_distance = -1;
float inches = 0;
int inputCounts;
float sensorValue;
float inputVolts;
double distanceInInches;
String receivedData = "";
unsigned long lastHeartbeatTime = millis();

while(1){
   Serial.println("In routine to do new part");

   
    if (client.available()) {
        char c = client.read();
        //end when new line is sent
        if (c == '\n') {
           Serial.println("Received data: " + receivedData);
           if (receivedData == "BREAK"){
            return;
           }
           if (receivedData == "heartbeat"){
              lastHeartbeatTime = millis();
           }
            if (receivedData == "saved_starting"){
                inputCounts = P1.readAnalog(1, 1); //Reading 1 more time
                distanceInInches = inputCounts*19.875/65535.0 +0.1;
                Serial.println("Sending starting position");
                String str = String(distanceInInches, 6);
                char msg[10];
                str.toCharArray(msg, 9);
                client.write(msg);
                delay(500);
                return;
           }

           
           receivedData = "";
      }
      else{
        receivedData += c;
      }
     }

     if (millis() - lastHeartbeatTime > heartbeatInterval) {
        // More than 5 seconds since last heartbeat, return
        Serial.println("Timeout waiting in setup new part");
        return;
    }
    
     // Read the values of the switches
  int left_switchValue = digitalRead(left_switchPin);
  int right_switchValue = digitalRead(right_switchPin);

  int cur_speed = starting_speed;

  while (left_switchValue == LOW) {
    digitalWrite(DIR_PIN, LOW); // away motor

    while ((cur_speed < terminal_speed) && (left_switchValue == LOW)) {
      cur_speed += 200;
      tone(PUL_PIN, cur_speed);
      delay(100);
      left_switchValue = digitalRead(left_switchPin);
    }

    // Update switch value
    left_switchValue = digitalRead(left_switchPin);
    if (left_switchValue != LOW)
    {
      noTone(PUL_PIN);
      break;
    }
    lastHeartbeatTime = millis(); //reset heartbeat time since none sent during movement
  }

  while (right_switchValue == LOW) {
    digitalWrite(DIR_PIN, HIGH); // towards motor

    while ((cur_speed < terminal_speed) &&(right_switchValue == LOW)) {
    cur_speed += 200;
    tone(PUL_PIN, cur_speed);
    delay(100);
    right_switchValue = digitalRead(right_switchPin);
    }

    // Update switch value
    right_switchValue = digitalRead(right_switchPin);
    if (right_switchValue != LOW)
    {
      noTone(PUL_PIN);
      break;
    }
    lastHeartbeatTime = millis(); //reset heartbeat time since none sent during movement
  }
 inputCounts = P1.readAnalog(1, 1); //Reads analog data from slot 1 channel 2 of the analog input module
 inputVolts = 5 * ((float)inputCounts / 65535);  //Convert 13-bit value to Volts
 distanceInInches = inputCounts*19.875/65535.0 +0.1;//manual 0.1 inch offset 
 Serial.print(distanceInInches, 5); // Print the distance in inches up to 2 decimal places
 Serial.println(" inches");

 delay(100);
   }

 } 


 void StretchSeal(double starting_measurement, double ending_measurement, double stretching_speed, EthernetClient client){
  float inches = 0;
  int inputCounts;
  float sensorValue;
  float inputVolts;
  double distanceInInches;
  double threshold = 0.0000;
  double overshoot = 0.05;
  double min_absolute_stop = 5.3000;//5.25;  //can't go further than than this with plates
  double max_absolute_stop = 14.5;//15.100;
  double slow_stop_threshold = 0.02; //how far before end to slow down 


  int stretch_terminal_speed = 1300*stretching_speed; //eq. found by taking data points and finding linear line of best fit
  int return_terminal_speed = 1300*6;
  double dist1;
  double dist2;
  int probetime;

  unsigned long lastHeartbeatTime = millis();
  unsigned long stretch_start_time= 0;
  unsigned long stretch_end_time=0;

  String receivedData = "";
while(1){
   int cur_speed = 2000;

  int left_switch_value = digitalRead(left_switchPin); //away from motor
  int right_switch_value = digitalRead(right_switchPin);// towards motor

  lastHeartbeatTime = millis(); //reset heartbeat time since none sent during movement
  while(right_switch_value != 0){
    Serial.println("Waiting for right switch");
      right_switch_value = digitalRead(right_switchPin);
      delay(100);

      if (client.available()) {
        char c = client.read();
        //end when new line is sent
        if (c == '\n') {
           Serial.println("Received data: " + receivedData);
           if (receivedData == "BREAK"){
            return;
           }
           if (receivedData == "heartbeat"){
              lastHeartbeatTime = millis();
           }
           receivedData = "";
      }
      else{
        receivedData += c;
      }
     }

     if (millis() - lastHeartbeatTime > heartbeatInterval) {
        // More than 5 seconds since last heartbeat, return
        Serial.println("Timeout waiting for left switch");
        return;
    }

      
  } //wait until right switch pressed

//  Serial.print("Left Switch Value: ");
//  Serial.println(left_switch_value, 2);
 
   Serial.println("going to starting position");

    inputCounts = P1.readAnalog(1, 1); //Reads analog data from slot 1 channel 2 of the analog input module
    inputVolts = 5 * ((float)inputCounts / 65535);  //Convert 13-bit value to Volts
    distanceInInches = inputCounts*19.875/65535.0 +0.1;//manual 0.1 inch offset 

//to go to starting position, want to go too far and then come back
     //noTone(PUL_PIN);
     digitalWrite(DIR_PIN, HIGH); // towards from motor
     delay(100);
     tone(PUL_PIN,cur_speed);//change this to make it go faster

     probetime = millis();
     
    while(distanceInInches > (starting_measurement - overshoot)){
//        Serial.println("waiting for limit reached");
      if (distanceInInches <= min_absolute_stop){
        Serial.println("LIMIT REACHED*****");
        noTone(PUL_PIN);
        break;
      }

      if((millis() - probetime) > 100){
        dist2 = distanceInInches;
        if(dist2 == dist1){
          noTone(PUL_PIN);
          delay(2000);
          cur_speed = 2000;

        }
        probetime = millis();
        dist1 = distanceInInches;
      }

      
      //delay(1);
      inputCounts = P1.readAnalog(1, 1); //Reads analog data from slot 1 channel 2 of the analog input module
      inputVolts = 5 * ((float)inputCounts / 65535);  //Convert 13-bit value to Volts
      distanceInInches = inputCounts*19.875/65535.0 +0.1;//manual 0.1 inch offset 
      Serial.println(distanceInInches, 5); // Print the distance in inches up to 2 decimal places
      cur_speed += 1;
      if (cur_speed > return_terminal_speed){
        cur_speed = return_terminal_speed;
      }
      tone(PUL_PIN,cur_speed);//change this to make it go faster
    }

     noTone(PUL_PIN);
     digitalWrite(DIR_PIN, LOW); // away motor
     delay(100);
     cur_speed =2000;
    
    
     
    while(distanceInInches < (starting_measurement)){
      //delay(1);
      inputCounts = P1.readAnalog(1, 1); //Reads analog data from slot 1 channel 2 of the analog input module
      inputVolts = 5 * ((float)inputCounts / 65535);  //Convert 13-bit value to Volts
      distanceInInches = inputCounts*19.875/65535.0 +0.1;//manual 0.1 inch offset 
      Serial.println(distanceInInches, 5); // Print the distance in inches up to 2 decimal places
      cur_speed += 1;
       if (cur_speed > return_terminal_speed){
        cur_speed = return_terminal_speed;
      }
       tone(PUL_PIN, cur_speed);//change this to make it go 
    }
    noTone(PUL_PIN);

  //now we are at the starting position
  inputCounts = P1.readAnalog(1, 1); //Reading 1 more time
  inputVolts = 5 * ((float)inputCounts / 65535); 
  distanceInInches = inputCounts*19.875/65535.0 +0.1;
  Serial.println("Sending starting position");

   String str1 = String(distanceInInches, 6);
   String str2 = "LOG:START: " + str1;
   char msg[22];
   str2.toCharArray(msg, 21);
   client.write(msg);
   delay(500);

lastHeartbeatTime = millis(); //reset heartbeat time since none sent during movement
  while(left_switch_value != 0){
//       Serial.println("Waiting for left switch");
      left_switch_value = digitalRead(left_switchPin);
      delay(100);
      
 if (client.available()) {
        char c = client.read();
        //end when new line is sent
        if (c == '\n') {
           Serial.println("Received data: " + receivedData);
           if (receivedData == "BREAK"){
            return;
           }
           if (receivedData == "heartbeat"){
              lastHeartbeatTime = millis();
           }
           receivedData = "";
      }
      else{
        receivedData += c;
      }
     }

     if (millis() - lastHeartbeatTime > heartbeatInterval) {
        // More than 5 seconds since last heartbeat, return
        Serial.println("Timeout waiting for right switch");
        return;
    }
       
  } //wait until left switch pressed

 
  stretch_start_time = millis();
  
  cur_speed = 2000;
  digitalWrite(DIR_PIN, LOW); // away from motor
  delay(100);
     tone(PUL_PIN, 2000);//change this to make it go faster
     probetime = millis();
    while(distanceInInches < ending_measurement){
      //delay(1);
      inputCounts = P1.readAnalog(1, 1); //Reads analog data from slot 1 channel 2 of the analog input module
      inputVolts = 5 * ((float)inputCounts / 65535);  //Convert 13-bit value to Volts
      distanceInInches = inputCounts*19.875/65535.0 +0.1;//manual 0.1 inch offset 

      if (distanceInInches <= min_absolute_stop){
        noTone(PUL_PIN);
        break;
      }

      if (distanceInInches >= max_absolute_stop){
        Serial.println("LIMIT REACHED*****");
        noTone(PUL_PIN);
        break;
      }

      Serial.println(distanceInInches, 5); // Print the distance in inches up to 2 decimal places      
      if((millis() - probetime) > 100){//subject to change
        dist2 = distanceInInches;
        if(dist2 == dist1){
          noTone(PUL_PIN);
          delay(2000);
          cur_speed = 2000;

        }
        probetime = millis();
        dist1 = distanceInInches;
      }
      cur_speed += 1;
      if (cur_speed > stretch_terminal_speed){
        cur_speed = stretch_terminal_speed;
      }
      if(distanceInInches > ending_measurement - slow_stop_threshold){
        cur_speed = cur_speed - 51;
        if(cur_speed < 2000){
          cur_speed = 2000;
        }
      }
      tone(PUL_PIN,cur_speed);//change this to make it go faster
    }
    noTone(PUL_PIN);

  stretch_end_time = millis();

  long stretch_time_duration = stretch_end_time - stretch_start_time;
  double stretch_time_seconds = stretch_time_duration / 1000.0; // Convert milliseconds to seconds
  Serial.print("Stretch time duration: ");
  Serial.println(stretch_time_seconds);

  delay(500);
  inputCounts = P1.readAnalog(1, 1); //Reading 1 more time
  inputVolts = 5 * ((float)inputCounts / 65535); 
  distanceInInches = inputCounts*19.875/65535.0 +0.1;
  Serial.println("Sending ending position");

   String dst = String(distanceInInches, 6);
   String time_str = String(stretch_time_seconds,6);
   String str4 = "LOG:END: " + dst + ":" + time_str;
   char msg4[28];
   str4.toCharArray(msg4, 27);
   client.write(msg4);
   delay(500);

  
 }
 }
