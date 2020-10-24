/**
 * Author:Ab Kurk
 * version: 1.0
 * date: 4/02/2018
 * Description: 
 * This sketch is part of the guide to putting your Arduino to sleep
 * tutorial. We use the:
 * Adafruit DS3231 RTC
 * Adafruit 5V ready Micro SD break out board
 * Arduino Pro Mini
 * DHT11 or DHT22 humidety/Temperature sensor
 * In this example we use the RTC to wake up the Arduino to log the temp and humidity on to an SD card.
 * After the data has been logged the Arduino goes back to sleep and gets woken up 5 minutes later to 
 * start all over again
 * Link To Tutorial http://www.thearduinomakerman.info/blog/2018/1/24/guide-to-arduino-sleep-mode
 * Link To Project   http://www.thearduinomakerman.info/blog/2018/2/5/wakeup-rtc-datalogger
 */

#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>  //RTC Library https://github.com/JChristensen/DS3232RTC
#include "DHT.h"        // Adafruit DHT Library for the DHT 11 or DHT 22 https://github.com/adafruit/DHT-sensor-library/archive/master.zip
#include <SPI.h>        //SPI library for the sd breakout board. Part of the Arduino IDE
#include <SD.h>         //SD library for the sd breakout board. Part of the Arduino IDE


//Variables needed for the DHT (Humidity/Temperature Sensor
#define DHTPIN 4     // what digital pin we're connected  the DHT sensor to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE); // Creating the DHT object to read from the DHT Sensor
//card reader variabls
File myFile;//Creating the file object so we can write to the SD card
const int chipSelect = 10;// The digital pin needed for the Adafruit SD card breakout board
//RTC Module global variables
const int time_interval=5;// Sets the wakeup intervall in minutes

void setup() {
  Serial.begin(115200);//Start Serial Comunication
  pinMode(LED_BUILTIN,OUTPUT);//We use the led on pin 13 to indecate when Arduino is A sleep
  pinMode(interruptPin,INPUT_PULLUP);//Set pin d2 to input using the buildin pullup resistor
  digitalWrite(LED_BUILTIN,HIGH);//turning LED on
  
  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);
    /*
     * Uncomment the block block to set the time on your RTC. Remember to comment it again 
     * otherwise you will set the time at everytime you upload the sketch
     * /
     /* Begin block
     tmElements_t tm;
    tm.Hour = 00;               // set the RTC to an arbitrary time
    tm.Minute = 00;
    tm.Second = 00;
    tm.Day = 4;
    tm.Month = 2;
    tm.Year = 2018 - 1970;      // tmElements_t.Year is the offset from 1970
    RTC.write(tm);              // set the RTC from the tm structure
     Block end * */
     time_t t; //create a temporary time variable so we can set the time and read the time from the RTC
    t=RTC.get();//Gets the current time of the RTC
    RTC.setAlarm(ALM1_MATCH_MINUTES , 0, minute(t)+time_interval, 0, 0);// Setting alarm 1 to go off 5 minutes from now
    // clear the alarm flag
    RTC.alarm(ALARM_1);
    // configure the INT/SQW pin for "interrupt" operation (disable square wave output)
    RTC.squareWave(SQWAVE_NONE);
    // enable interrupt output for Alarm 1
    RTC.alarmInterrupt(ALARM_1, true);
    
    dht.begin();//Start the DHT sensor
    /**Initializes the SD breakout board. If it is not ready or not connected correct it writes
     * an error message to the serial monitor
     **/
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");
}

void loop() {
 delay(5000);//wait 5 seconds before going to sleep. In real senairio keep this as small as posible
 Going_To_Sleep();
}

void Going_To_Sleep(){
    sleep_enable();//Enabling sleep mode
    attachInterrupt(0, wakeUp, LOW);//attaching a interrupt to pin d2
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
    digitalWrite(LED_BUILTIN,LOW);//turning LED off
    time_t t;// creates temp time variable
    t=RTC.get(); //gets current time from rtc
    Serial.println("Sleep  Time: "+String(hour(t))+":"+String(minute(t))+":"+String(second(t)));//prints time stamp on serial monitor
    delay(1000); //wait a second to allow the led to be turned off before going to sleep
    sleep_cpu();//activating sleep mode
    Serial.println("just woke up!");//next line of code executed after the interrupt 
    digitalWrite(LED_BUILTIN,HIGH);//turning LED on
    temp_Humi();//function that reads the temp and the humidity
    t=RTC.get();
    Serial.println("WakeUp Time: "+String(hour(t))+":"+String(minute(t))+":"+String(second(t)));//Prints time stamp 
    //Set New Alarm
    RTC.setAlarm(ALM1_MATCH_MINUTES , 0, minute(t)+time_interval, 0, 0);
  
  // clear the alarm flag
  RTC.alarm(ALARM_1);
  }

void wakeUp(){
  Serial.println("Interrrupt Fired");//Print message to serial monitor
   sleep_disable();//Disable sleep mode
  detachInterrupt(0); //Removes the interrupt from pin 2;
 
}

//This function reads the temperature and humidity from the DHT sensor
void temp_Humi(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();//reads humidity
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  writeData(h,t,f);//sends the data to the writeData function
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
}
 /**
  * the writeData function gets the humidity (h), temperature in celsius (t) and farenheit (f) as input 
  * parameters. It uses the RTC to create a filename using the current date, and writes the temperature
  * and humidity with a date stamp to this file
  */
  void writeData(float h,float t,float f){
  time_t p; //create time object for time and date stamp
  p=RTC.get();//gets the time from RTC
  String file_Name=String(day(p))+monthShortStr(month(p))+String(year(p))+".txt";//creates the file name we are writing to.
  myFile = SD.open(file_Name, FILE_WRITE);// creates the file object for writing

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to "+ file_Name);
    //appends a line to the file with time stamp and humidity and temperature data
    myFile.println(String(hour(p))+":"+String(minute(p))+" Hum: "+String(h)+"% C: "+String(t)+" F: "+String(f));
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening "+ file_Name);
  }
  //opening file for reading and writing content to serial monitor
  myFile = SD.open(file_Name);
  if (myFile) {
    Serial.println(file_Name);

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening"+ file_Name);
  }
 
 }

