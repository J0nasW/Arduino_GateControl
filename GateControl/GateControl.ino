// GATE CONTROL
// by Jonas Wilinski
//
// This is the sketch to control Gates @ GCHH.
//
// -- TIME ------------------------------------------------------
// Set the date and time by entering the following on the Arduino
// serial monitor:
//  year,month,day,hour,minute,second,
//
// Where
//  year can be two or four digits,
//  month is 1-12,
//  day is 1-31,
//  hour is 0-23, and
//  minute and second are 0-59.
//
// Entering the final comma delimiter (after "second") will avoid a
// one-second timeout and will allow the RTC to be set more accurately.
//
// No validity checking is done, invalid values or incomplete syntax
// in the input will result in an incorrect RTC setting.
//
// Command list:
// 
// settime_yy,mm,dd,hh,mm,ss - For setting the current time on the RTC.
// setalarm_hh,mm - For setting the DeepSleep Alarm to wake the ESP up and do it's thing (in hh:mm)
// motor_1 - Put the Servo in on-position (180°)
// motor_0 - Put the Servo in off-position (0°)
// prod_1 - Go into productivity mode and trigger the deepSleep after 5min of loop.
// prod_0 - Go into experimental mode and don't trigger the deepSleep after 5min of loop.
// deep_ - Go to DeepSleep right away until you wake up
//
//
// Jonas Wilinski 26Jun2020

// Some definitions
#include <avr/sleep.h>      // this AVR library contains the methods that controls the sleep modes
//#include "LowPower.h"
#define interruptPin 1      // Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Time.h>
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/


unsigned long start; // Duration counter
int dauer; // Loop Dauer bis DeepSleep; 600.000ms sind 10min
int remaining;

int prod; // Productivity mode is set off by default.

int actuator; // ...

// RELAIS 1 & 2
int relay_1 = 5;
int relay_2 = 6;

static time_t tLast;
time_t t;
int alarm_hour;
int alarm_minute;
tmElements_t tm;

String serial_input;
String serial_command;
String serial_payload;
int serial_int_payload;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN,OUTPUT);//We use the led on pin 13 to indecate when Arduino is A sleep
    pinMode(interruptPin,INPUT_PULLUP);//Set pin 7 to input using the buildin pullup resistor
    digitalWrite(LED_BUILTIN,HIGH);//turning LED on
    Serial.println("<-- GATE CONTROL IS READY -->");
    
    // For the Relays
    pinMode(relay_1, OUTPUT);
    pinMode(relay_2, OUTPUT);
    digitalWrite(relay_1, LOW);
    digitalWrite(relay_2, LOW);

    prod = 1; //Standard is productivity mode on (1)
    dauer = 300000; //Length of loop - 300.000 = 5min
    actuator = 10000; // 10sec - Duration to open/close the actuator
    
    // setSyncProvider() causes the Time library to synchronize with the
    // external RTC by calling RTC.get() every five minutes by default.
    setSyncProvider(RTC.get);
    Serial << F("RTC Sync");
    if (timeStatus() != timeSet) Serial << F(" FAIL!");
    Serial << endl;

    // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    //RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    //RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, true);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);
}

void loop()
{
  start = millis();

  do {
    SerialInput();
  
    t = now();
    if (t != tLast) {
        tLast = t;
        printDateTime(t);
        if (second(t) == 0) {
            float c = RTC.temperature() / 4.;
            float f = c * 9. / 5. + 32.;
            Serial << F("  ") << c << F(" C.  ");
            remaining = dauer-(millis()-start);
            remaining = remaining / 1000;
            Serial.print(" Seconds until productivity: ");
            Serial.println(remaining);
        }
        Serial << endl;
    }
  
    
  } while (millis() - start < dauer); // 300000 Millisekunden sind 5 Min

  if (prod == 1) {
    productivity();
  } else if (prod == 0) {
    Serial.println("PROD WAS 0 BUT WE WILL CARRY ON ANYWAY!!!");
  }
}


void SerialInput() {
  // -- INPUT CHECKS --------------------------------------------
    
    //GET THE INPUT
    if (Serial.available() > 0) {
      // read the incoming byte:
      serial_input = Serial.readString();
      // say what you got:
      Serial.print("I received: ");
      Serial.println(serial_input);
    

    serial_command = split(serial_input, '_',0);

    Serial.print("Command received: ");
    Serial.println(serial_command);

    if (serial_command == "settime") {
        serial_payload = split(serial_input, '_',1);
        Serial.print("Settime payload received: ");
        Serial.println(serial_payload);

        int y = split(serial_payload, ',',0).toInt();
        if (y >= 100 && y < 1000)
            Serial << F("Error: Year must be two digits or four digits!") << endl;
        else {
            if (y >= 1000)
                tm.Year = CalendarYrToTm(y);
            else    // (y < 100)
                tm.Year = y2kYearToTm(y);
            tm.Month = split(serial_payload, ',',1).toInt();
            tm.Day = split(serial_payload, ',',2).toInt();
            tm.Hour = split(serial_payload, ',',3).toInt();
            tm.Minute = split(serial_payload, ',',4).toInt();
            tm.Second = split(serial_payload, ',',5).toInt();
            t = makeTime(tm);
            RTC.set(t);        // use the time_t value to ensure correct weekday is set
            setTime(t);
            Serial << F("RTC set to: ");
            printDateTime(t);
            Serial << endl;
        }
    } else if (serial_command == "setalarm") {
        serial_payload = split(serial_input, '_',1);
        Serial.print("Alarm payload received: ");
        Serial.println(serial_payload);

        alarm_hour = split(serial_payload, ',',0).toInt();
        alarm_minute = split(serial_payload, ',',1).toInt();

        
        if (alarm_hour >=24) {
          Serial << F("Error: Hours must be between 0 and 23") << endl;
          exit;
        }
        if (alarm_minute >59) {
          Serial << F("Error: Minutes must be between 0 and 59") << endl;
          exit;
        }

        RTC.setAlarm(ALM1_MATCH_HOURS, 0, alarm_minute, alarm_hour, 0);
        // clear the alarm flags
        RTC.alarm(ALARM_1);
        RTC.alarmInterrupt(ALARM_1, true);
        
        Serial << F("RTC ALARM 01 set to: ");
        printI00(alarm_hour, ':');
        printI00(alarm_minute, ' ');
        Serial << endl;
    } else if (serial_command == "motor") {
        serial_payload = split(serial_input, '_',1);
        Serial.print("Actuator payload received: ");
        Serial.println(serial_payload);
        serial_int_payload = serial_payload.toInt();

        if (serial_int_payload == 1) {
          Serial.print("Attempting to open Actuator...");
          digitalWrite(relay_1, HIGH);
          digitalWrite(relay_2, LOW);
          delay(actuator); //Wait
          digitalWrite(relay_1, LOW);
          digitalWrite(relay_2, LOW);
          Serial.println("Actuator opened.");
        } else if (serial_int_payload == 0) {
          Serial.print("Attempting to close Actuator...");
          digitalWrite(relay_1, LOW);
          digitalWrite(relay_2, HIGH);
          delay(actuator); //Wait
          digitalWrite(relay_1, LOW);
          digitalWrite(relay_2, LOW);
          Serial.println("Actuator closed.");
          }
        
        
    } else if (serial_command == "deep") {
        serial_payload = split(serial_input, '_',1);
        Serial.print("DeepSleep payload received: ");
        Serial.println(serial_payload);

        Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal (Hopefully RTC Alarm Signal)");
        Serial << F("RTC ALARM 01 will wake me up at: ");
        printI00(alarm_hour, ':');
        printI00(alarm_minute, ' ');
        Serial << endl;
        productivity();
        
    } else if (serial_command == "prod") {
        serial_int_payload = split(serial_input, '_',1).toInt();
        Serial.print("Productivity payload received: ");
        Serial.println(serial_int_payload);

        if (serial_int_payload == 0) {
            Serial.println("Going into experimental mode. (prod=0)");
            prod = 0;
          } else if (serial_int_payload == 1) {
            Serial.println("Going into productive mode. (prod=1)");
            Serial.println("Hopefully you set the alarm right. Currently, it is:");
            Serial << F("RTC ALARM 01: ");
            printI00(alarm_hour, ':');
            printI00(alarm_minute, ' ');
            Serial << endl;
            prod = 1;
        }
        
    } else if (serial_command != "") {
      Serial.println("No valid input.");
    }
    serialFlush();
}
}


void productivity() {
  
  Serial.print("Attempting to open Actuator...");
  digitalWrite(relay_1, HIGH);
  digitalWrite(relay_2, LOW);
  delay(actuator); //Wait
  digitalWrite(relay_1, LOW);
  digitalWrite(relay_2, LOW);
  Serial.println("Actuator opened.");
  delay(actuator); //Wait
  Serial.print("Attempting to close Actuator...");
  digitalWrite(relay_1, LOW);
  digitalWrite(relay_2, HIGH);
  delay(actuator); //Wait
  digitalWrite(relay_1, LOW);
  digitalWrite(relay_2, LOW);
  Serial.println("Actuator closed.");

  Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal.");
  Serial << F("RTC ALARM 01 will wake me up at: ");
  printI00(alarm_hour, ':');
  printI00(alarm_minute, ' ');
  Serial << endl;
  Going_To_Sleep();
}

void Going_To_Sleep(){
    sleep_enable();//Enabling sleep mode
    attachInterrupt(3, wakeUp, LOW);//attaching a interrupt to pin d2
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
    digitalWrite(LED_BUILTIN,LOW);//turning LED off
    delay(1000); //wait a second to allow the led to be turned off before going to sleep
    sleep_cpu();//activating sleep mode
    Serial.println("just woke up!");//next line of code executed after the interrupt 
    digitalWrite(LED_BUILTIN,HIGH);//turning LED on
    delay(1000); //wait a second to allow the led to be turned off before going to sleep
    setSyncProvider(RTC.get);
    Serial << F("RTC Sync");
    if (timeStatus() != timeSet) Serial << F(" FAIL!");
    Serial << endl;
  }

void wakeUp()
{
  Serial.println("Interrrupt Fired");//Print message to serial monitor
  sleep_disable();//Disable sleep mode
  detachInterrupt(3); //Removes the interrupt from pin 2;
  
}

// --- SOME HELPING FUNCTIONS -------------------------------

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
  Serial.println("Serial has been flushed!");
}  

String split(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}




// print date and time to Serial
void printDateTime(time_t t)
{
    printDate(t);
    Serial << ' ';
    printTime(t);
}

// print time to Serial
void printTime(time_t t)
{
    printI00(hour(t), ':');
    printI00(minute(t), ':');
    printI00(second(t), ' ');
}

// print date to Serial
void printDate(time_t t)
{
    printI00(day(t), 0);
    Serial << monthShortStr(month(t)) << _DEC(year(t));
}

// Print an integer in "00" format (with leading zero),
// followed by a delimiter character to Serial.
// Input value assumed to be between 0 and 99.
void printI00(int val, char delim)
{
    if (val < 10) Serial << '0';
    Serial << _DEC(val);
    if (delim > 0) Serial << delim;
    return;
}
