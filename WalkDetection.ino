
/* 
 * This is the arduino code for a portable wearable device based on the Arduino Pro Mini 
 * for recording accelerometer movement data to create a labeled dataset for normal walking and 
 * non-walking signals. ADXL345 accelerometer module was used and a micro-sd card module
 * was used to log the data continuosly from the sensor. A 9V battery was used to power
 * the system along with 2 leds (red and green) for feedback and 3 buttons for user input.
 *  
 * This example code is in the public domain.

   modified 18 October 2016
   by Mustafa Lokhandwala, Future Foundry.
 */


 
//-------------Import Libraries-----------------//

#include <avr/sleep.h>
#include <avr/power.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

//------------Define LED and BUTTON Pins--------//

#define WAKE_BUTTON 3             //Interrupt button to wake system from deep sleep
#define WALK_BUTTON 5             //Button to toggle walking data sample collection on or off
#define START_STOP_BUTTON 6       //Button to toggle idle mode on and off
#define WALK_LED 9                //led to indicate 'walking' data is being recorded
#define START_LED 7               //led to indicate system has started and is recording 'not-walking' data

//------Assign a unique ID to the sensor--------//

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//------Make a file object to deal with text files------//

File myFile;

int i,j;
unsigned long startSleep=0;                
unsigned long waitBeforeSleeping=10000;        //Define time before entering deep sleep here in milliseconds


//---------------Function to record walking data------------------//

void Walking() {

//  Serial.println("Now Recording Walking Data.");
  while(1)
  {
    myFile = SD.open("Walking.txt", FILE_WRITE);                  //This loop records 1000 samples
    if (myFile) {                                                 //before saving and closing the file.
      for (j=1; j<=1000; j++)
    {
      sensors_event_t event; 
      accel.getEvent(&event);
      myFile.print(event.acceleration.x); myFile.print(", ");
      myFile.print(event.acceleration.y); myFile.print(", ");
      myFile.println(event.acceleration.z);
      delay(1);
    }

    myFile.print("0"); myFile.print(", ");
    myFile.print("0"); myFile.print(", ");
    myFile.println("0");
    myFile.close();

    delay(50);
    
//    Serial.print("File ");Serial.print("Walking");Serial.print(" ");Serial.println("done.");

    if (digitalRead(START_STOP_BUTTON)==LOW)                     //Check for user input after saving
        {
          delay(50);
          digitalWrite(START_LED, LOW);
          digitalWrite(WALK_LED, LOW);
          idle();
        }

      if (digitalRead(WALK_BUTTON)==LOW)
        {
          delay(50);
          digitalWrite(START_LED, HIGH);
          digitalWrite(WALK_LED, LOW);
          notWalking();
        }
   }
    
    else {
      
      errorHandlerSD();                                         //Call SD card error function if error
      
  }  
 }
}

//-------------Function to record not-walking data----------------//

 void notWalking() {

//  Serial.println("Now Recording notWalking Data.");
  while(1)
  {
    myFile = SD.open("Random.txt", FILE_WRITE);
    if (myFile) {
      for (j=1; j<=1000; j++)
    {
      sensors_event_t event; 
      accel.getEvent(&event);
      myFile.print(event.acceleration.x); myFile.print(", ");
      myFile.print(event.acceleration.y); myFile.print(", ");
      myFile.println(event.acceleration.z);
      delay(1);
    }

    myFile.print("0"); myFile.print(", ");
    myFile.print("0"); myFile.print(", ");
    myFile.println("0");    
    myFile.close();

    delay(50);
    
//    Serial.print("File ");Serial.print("Random");Serial.print(" ");Serial.println("done.");

    if (digitalRead(START_STOP_BUTTON)==LOW)
        {
          delay(50);
          digitalWrite(START_LED, LOW);
          digitalWrite(WALK_LED, LOW);
          idle();
        }

      if (digitalRead(WALK_BUTTON)==LOW)
        {
          delay(50);
          digitalWrite(START_LED, HIGH);
          digitalWrite(WALK_LED, HIGH);
          Walking();
        }
   }
    
    else {
      
      errorHandlerSD();
      
    }  
 }
 }


//------------------------Idle mode function--------------------------//


void idle() {
  
//  Serial.println("Now Idling.");
  for(i=1; i<=10; i++)                     //Blink both leds alternatingly 10 times to indicate system is
  {                                        //now in idle mode
    digitalWrite(START_LED, LOW);
    digitalWrite(WALK_LED, HIGH);
    delay(100);
    digitalWrite(START_LED, HIGH);
    digitalWrite(WALK_LED, LOW);
    delay(100);    
  }
  digitalWrite(START_LED, LOW);
  digitalWrite(WALK_LED, LOW);
  startSleep=millis();
  while((millis() - startSleep) < waitBeforeSleeping)  //Loop which checks for user input till time to
  {                                                    //wait before sleep expires
    if (digitalRead(START_STOP_BUTTON)==LOW)
    {
      delay(50);
      digitalWrite(START_LED, HIGH);
      digitalWrite(WALK_LED, LOW);
      notWalking();
    }

    if (digitalRead(WALK_BUTTON)==LOW)
      {
        delay(50);
        digitalWrite(START_LED, HIGH);
        digitalWrite(WALK_LED, HIGH);
        Walking();
      }   
  }
  sleepNow();
}

//--------------------ISR function, does nothing-----------------//

void wakeUp() {
  
}

//-------------------Deep sleep function to maximize battery life-----------------//

void sleepNow()         // here we put the arduino to sleep
{
//    Serial.println("Now Entering Sleep Mode.");
    for(i=1; i<=3; i++)
  {
    digitalWrite(START_LED, HIGH);           //Blinks both led 3 times to indicate device is entering sleep mode
    digitalWrite(WALK_LED, HIGH);
    delay(500);
    digitalWrite(START_LED, LOW);
    digitalWrite(WALK_LED, LOW);
    delay(500);    
  }
    delay(100);

    power_all_disable ();
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin

    attachInterrupt(digitalPinToInterrupt(WAKE_BUTTON), wakeUp, FALLING); // use interrupt 0 (pin 2) and run function
                                                                         // wakeUpNow when pin 2 gets LOW
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    power_all_enable();

//    Serial.println("Waking Up.");
    delay(100);
                               
    detachInterrupt(digitalPinToInterrupt(WAKE_BUTTON));      // disables interrupt 1 on pin 3 so the
                                                              // wakeUp() will not be executed
     idle();                                                         // during normal running time.
 
}


//------------------Function to indicate problem with accelerometer--------------------------//

void errorHandlerAcc() {

//  Serial.println("Error!");
  digitalWrite(WALK_LED, LOW);
  while(1)                                      //red led blinks every 100ms
  {
    digitalWrite(START_LED, LOW);
    delay(100);
    digitalWrite(START_LED, HIGH);
    delay(100);    
  }
}

//------------------Function to indicate problem writing to sdcard--------------------------//

void errorHandlerSD() {
//
//  Serial.println("Error!");
  digitalWrite(WALK_LED, LOW);
  while(1)                                      //red led blinks every 500ms
  {
    digitalWrite(START_LED, LOW);
    delay(500);
    digitalWrite(START_LED, HIGH);
    delay(500);    
  }
}

//--------------------------------void setup---------------------------------------------------//

void setup() {

//  Serial.begin(9600);
//  Serial.println("Now Running Setup.");
  
  pinMode(WALK_BUTTON, INPUT_PULLUP);
  pinMode(START_STOP_BUTTON, INPUT_PULLUP);
  pinMode(WAKE_BUTTON, INPUT_PULLUP);
  pinMode(WALK_LED, OUTPUT);
  pinMode(START_LED, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  while (!Serial) {
  }
//  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
//    Serial.println("initialization failed!");
    errorHandlerSD();
  }
//  Serial.println("initialization done.");

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
//    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    errorHandlerAcc();
  }

  // Set the accelerometer datarate and range
  
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);

  idle();    //system by default boots up in idle mode
  
}

//----------------------------------void loop--------------------------------------------//

void loop() { 
  //does nothing
}


