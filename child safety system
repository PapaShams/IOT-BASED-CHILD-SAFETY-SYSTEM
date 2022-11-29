#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL0xI3sHmk"
#define BLYNK_DEVICE_NAME "GPS"
//#define USE_ARDUINO_INTERRUPTS false

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#include <EEPROM.h>
//sender phone number with country code
//const String PHONE = "ENTER_PHONE_HERE";
const int totalPhoneNo = 5;
String phoneNo[totalPhoneNo] = { "", "", "", "", "" };
int offsetPhone[totalPhoneNo] = { 0, 13, 26, 39, 52 };
String tempPhone = "";
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//GPS Module TX pin to NodeMCU D1
//GPS Module RX pin to NodeMCU D2
#define rxGPS 13  //D1
#define txGPS 12  //D2
SoftwareSerial neogps(rxGPS, txGPS);
TinyGPSPlus gps;
BlynkTimer timer;
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//GSM Module TX pin to NodeMCU D3
//GSM Module RX pin to NodeMCU D4
#define rxGSM 0  //D3
#define txGSM 2  //D4
SoftwareSerial sim800(rxGSM, txGSM);
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
String smsStatus;
String senderNumber;
String receivedDate;
String msg;
String text;
String phoneNumber;
String callerID = "";

float latitude;   //Storing the Latitude
float longitude;  //Storing the Longitude
float velocity;   //Variable  to store the velocity
float sats;       //Variable to store no. of satellites response
String bearing;   //Variable to store orientation or direction of GPS

const byte RATE_SIZE = 4;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
float temperature;

double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int b = 0;
int Num = 100;  //calculate SpO2 by this sampling interval
String state = "Reading Data.";

double ESpO2 = 95.0;     //initial value of estimated SpO2
double FSpO2 = 0.7;      //filter factor for estimated SpO2
double frate = 0.95;     //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000  // wait for this time(msec) to output SpO2
#define SCALE 88.0       //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5       //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000  // if red signal is lower than this , it indicates your finger is not on the sensor
#define USEFIFO

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#define button1 14  //Button pin, on the other pin it's wired with GND
int status = 0;     //Button state
boolean DEBUG_MODE = 1;

char auth[] = "3Za1IQRq9azIU0_rsSdc_SPPIG_hUs1q";  //Blynk Authentication Token
char ssid[] = "Icon Fiber Dennis Shamwamama";      // WiFi SSID
char pass[] = "Hooster99";                         // WiFi Password

//unsigned int move_index;         // moving index, to be used later
unsigned int move_index = 1;  // fixed location for now

/*******************************************************************************
 * setup function
 ******************************************************************************/
void setup() {
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  Serial.begin(115200);
  Serial.println("NodeMCU USB serial initialize");
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  sim800.begin(9600);
  Serial.println("SIM800L serial initialize");
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  neogps.begin(9600);
  Serial.println("NEO6M serial initialize");
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  sim800.listen();
  neogps.listen();
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  //commit 512 bytes of ESP8266 flash (for "EEPROM" emulation)
  //this step actually loads the content (512 bytes) of flash
  //into a 512-byte-array cache in RAM
  EEPROM.begin(512);
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  Serial.println("List of Registered Phone Numbers");
  for (int i = 0; i < totalPhoneNo; i++) {
    phoneNo[i] = readFromEEPROM(offsetPhone[i]);
    if (phoneNo[i].length() != 13) {
      phoneNo[i] = "";
      Serial.println(String(i + 1) + ": empty");
    } else {
      Serial.println(String(i + 1) + ": " + phoneNo[i]);
    }
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  smsStatus = "";
  senderNumber = "";
  receivedDate = "";
  msg = "";
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  delay(9000);
  sim800.println("AT+CMGF=1");  //SMS text mode
  delay(1000);
  sim800.println("AT+CLIP=1");  //Enable Caller ID
  delay(500);
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  Blynk.begin(auth, ssid, pass);
  //timer.setInterval(1000L, checkGPS);  // every 5s check if GPS is connected, only really needs to be done once

  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("Sensor not detected!!! ");
    state = "Sensor error!!!";
    //while (1);
  }
  state = "Reading Data.";
  byte ledBrightness = 0x7F;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;     //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;           //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 800;       //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;       //Options: 69, 118, 215, 411
  int adcRange = 16384;       //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings

  particleSensor.enableDIETEMPRDY();
}




/*******************************************************************************
 * loop function
 ******************************************************************************/
void loop() {

  if (digitalRead(button1) == HIGH && status == 0) {  //And if it's pressed
    Serial.println("Button pressed");                 //Shows this message on the serial monitor
                                                      // delay(200);                         //Small delay to avoid detecting the button press many times

    sendLocation(phoneNumber);  //And this function is called
    status = 1;
  }
  if (digitalRead(button1) == LOW && status == 1) {
    status = 0;
  }
  //sim800.listen();
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  while (sim800.available()) {
    parseData(sim800.readString());
    //displayInfo();
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  while (Serial.available()) {
    sim800.println(Serial.readString());
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  while (neogps.available() > 0) {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(neogps.read()))
      displayInfo();
  }  
  sensorDataSend();
  Blynk.run();
  timer.run();
}  //main loop ends

//////////////////////////////////////////////

void sensorDataSend() {
  uint32_t ir, red, green;
  double fred, fir;
  double SpO2 = 0;  //raw SpO2 before low pass filtered

#ifdef USEFIFO
  particleSensor.check();  //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {  //do we have new data
#ifdef MAX30105
    red = particleSensor.getFIFORed();  //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();    //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR();  //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed();  //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif


    //routine for hearbeat rate
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    b++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);  //average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate);     //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered);         //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);              //square sum of alternate component of IR level
    if ((b % SAMPLING) == 0) {                              //slow down graph plotting speed for arduino Serial plotter by thin out
      if (millis() > TIMETOBOOT) {

        if (ir < FINGER_ON) {
          //Serial.print("Finger not detected!!");
          state = "Finger not detected!";
          //Serial.println();
        }  //indicator for finger error
        else {
          state = "Reading Data.";
          temperature = particleSensor.readTemperature();

          //Serial.print(" Oxygen % = ");
          //Serial.print(ESpO2);
          Blynk.virtualWrite(V0, ESpO2);
          //Serial.print(" Avg BPM= ");
          //Serial.print(beatAvg);
          //Serial.print(" BPM=");
          //Serial.print(beatsPerMinute);
          Blynk.virtualWrite(V6, beatsPerMinute);
          //Serial.print(" Temp=");
          //Serial.print(temperature);
          Blynk.virtualWrite(V5, temperature);
          //Serial.println();
        }
      }
    }

    if ((b % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);

      SpO2 = -23.3 * (R - 0.4) + 100;                //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;  //low pass filter

      sumredrms = 0.0;
      sumirrms = 0.0;
      b = 0;
      break;
    }
    particleSensor.nextSample();  //We're finished with this sample so move to next sample
  }
#endif
}


void displayInfo() {
  if (gps.location.isValid()) {
    sats = gps.satellites.value();    //get number of satellites
    latitude = (gps.location.lat());  //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
    velocity = gps.speed.kmph();                          //get velocity
    bearing = TinyGPSPlus::cardinal(gps.course.value());  // get the direction


    // Serial.print("SATS:  ");
    // Serial.println(sats);  // float to x decimal places
    // Serial.print("LATITUDE:  ");
    // Serial.println(latitude, 6);  // float to x decimal places
    // Serial.print("LONGITUDE: ");
    // Serial.println(longitude, 6);
    // Serial.print("SPEED: ");
    // Serial.print(velocity);
    // Serial.println("kmph");
    // Serial.print("DIRECTION: ");
    // Serial.println(bearing);

    Blynk.virtualWrite(V1, String(latitude, 6));
    Blynk.virtualWrite(V2, String(longitude, 6));
    Blynk.virtualWrite(V3, sats);
    Blynk.virtualWrite(V4, velocity);
    //myMap.location(move_index, latitude, longitude, "GPS_Location");
  }

  Serial.println();
}




/*******************************************************************************
 * parseData function:
 * this function parse the incomming command such as CMTI or CMGR etc.
 * if the sms is received. then this function read that sms and then pass 
 * that sms to "extractSms" function. Then "extractSms" function divide the
 * sms into parts. such as sender_phone, sms_body, received_date etc.
 ******************************************************************************/
void parseData(String buff) {
  Serial.println(buff);

  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  //HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
  if (buff.indexOf("RING") > -1) {
    boolean flag = 0;
    String callerID = "";
    //----------------------------------------------------------------------
    //+CLIP: "03001234567",129,"",0,"",0
    if (buff.indexOf("+CLIP:")) {
      unsigned int index, index1;

      index = buff.indexOf("\"");
      index1 = buff.indexOf("\"", index + 1);
      //get phone like this format 3001234567
      callerID = buff.substring(index + 2, index1);
      callerID.trim();
      //debugPrint("caller ID: "+callerID);
      //Serial.println(temp.length());
      if (callerID.length() == 13) {
        //number formate xxx-yyy-zzzzzzz
        //+923001234567
        flag = comparePhone(callerID);
      }
      //--------------------------------------------------------------------
      else if (callerID.length() == 10) {
        //number formate yyyy-zzzzzzz
        //3001234567
        flag = compareWithoutCountryCode(callerID);
        callerID = "0" + callerID;
      }
    }
    //----------------------------------------------------------------------
    if (flag == 1) {
      sim800.println("ATH");
      delay(1000);
      sendLocation(callerID);
    } else {
      sim800.println("ATH");
      debugPrint("The phone number is not registered.");
    }
    //----------------------------------------------------------------------
    return;
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

  unsigned int len, index;
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //Remove sent "AT Command" from the response string.
  index = buff.indexOf("\r");
  buff.remove(0, index + 2);
  buff.trim();
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  if (buff != "OK") {
    index = buff.indexOf(":");
    String cmd = buff.substring(0, index);
    cmd.trim();

    buff.remove(0, index + 2);
    //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
    if (cmd == "+CMTI") {
      //get newly arrived memory location and store it in temp
      index = buff.indexOf(",");
      String temp = buff.substring(index + 1, buff.length());
      temp = "AT+CMGR=" + temp + "\r";
      //get the message stored at memory location "temp"
      sim800.println(temp);
    }
    //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
    else if (cmd == "+CMGR") {
      extractSms(buff);
      //-------------------------------------------------------
      if (msg.equals("r") && phoneNo[0].length() != 13) {
        writeToEEPROM(offsetPhone[0], senderNumber);
        phoneNo[0] = senderNumber;
        String text = "Number is Registered: ";
        text = text + senderNumber;
        debugPrint(text);
        Reply("Number is Registered", senderNumber);
      }
      //-------------------------------------------------------
      if (comparePhone(senderNumber)) {
        doAction(senderNumber);
        //sendLocation();
      }
      //-------------------------------------------------------
    }
    //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
  } else {
    //The result of AT Command is "OK"
  }
}





/*******************************************************************************
 * doAction function:
 * Performs action according to the received sms
 ******************************************************************************/
void doAction(String phoneNumber) {
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  if (msg == "send location") {
    sendLocation(phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  if (msg == "r2=") {
    //mmmmmmm
    Serial.println(offsetPhone[1]);
    writeToEEPROM(offsetPhone[1], tempPhone);
    phoneNo[1] = tempPhone;
    String text = "Phone2 is Registered: ";
    text = text + tempPhone;
    debugPrint(text);
    Reply(text, phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "r3=") {
    writeToEEPROM(offsetPhone[2], tempPhone);
    phoneNo[2] = tempPhone;
    String text = "Phone3 is Registered: ";
    text = text + tempPhone;
    Reply(text, phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "r4=") {
    writeToEEPROM(offsetPhone[3], tempPhone);
    phoneNo[3] = tempPhone;
    String text = "Phone4 is Registered: ";
    text = text + tempPhone;
    Reply(text, phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "r5=") {
    writeToEEPROM(offsetPhone[4], tempPhone);
    phoneNo[4] = tempPhone;
    String text = "Phone5 is Registered: ";
    text = text + tempPhone;
    Reply(text, phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "list") {
    String text = "";
    if (phoneNo[0])
      text = text + phoneNo[0] + "\r\n";
    if (phoneNo[1])
      text = text + phoneNo[1] + "\r\n";
    if (phoneNo[2])
      text = text + phoneNo[2] + "\r\n";
    if (phoneNo[3])
      text = text + phoneNo[3] + "\r\n";
    if (phoneNo[4])
      text = text + phoneNo[4] + "\r\n";

    debugPrint("List of Registered Phone Numbers: \r\n" + text);
    Reply(text, phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "del=1") {
    writeToEEPROM(offsetPhone[0], "");
    phoneNo[0] = "";
    Reply("Phone1 is deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "del=2") {
    writeToEEPROM(offsetPhone[1], "");
    phoneNo[1] = "";
    debugPrint("Phone2 is deleted.");
    Reply("Phone2 is deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "del=3") {
    writeToEEPROM(offsetPhone[2], "");
    phoneNo[2] = "";
    debugPrint("Phone3 is deleted.");
    Reply("Phone3 is deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "del=4") {
    writeToEEPROM(offsetPhone[3], "");
    phoneNo[3] = "";
    debugPrint("Phone4 is deleted.");
    Reply("Phone4 is deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  else if (msg == "del=5") {
    writeToEEPROM(offsetPhone[4], "");
    phoneNo[4] = "";
    debugPrint("Phone5 is deleted.");
    Reply("Phone5 is deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  if (msg == "del=all") {
    writeToEEPROM(offsetPhone[0], "");
    writeToEEPROM(offsetPhone[1], "");
    writeToEEPROM(offsetPhone[2], "");
    writeToEEPROM(offsetPhone[3], "");
    writeToEEPROM(offsetPhone[4], "");
    phoneNo[0] = "";
    phoneNo[1] = "";
    phoneNo[2] = "";
    phoneNo[3] = "";
    phoneNo[4] = "";
    offsetPhone[0] = NULL;
    offsetPhone[1] = NULL;
    offsetPhone[2] = NULL;
    offsetPhone[3] = NULL;
    offsetPhone[4] = NULL;
    debugPrint("All phone numbers are deleted.");
    Reply("All phone numbers are deleted.", phoneNumber);
  }
  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  smsStatus = "";
  senderNumber = "";
  receivedDate = "";
  msg = "";
  tempPhone = "";
}





/*******************************************************************************
 * extractSms function:
 * This function divide the sms into parts. such as sender_phone, sms_body, 
 * received_date etc.
 ******************************************************************************/
void extractSms(String buff) {
  unsigned int index;

  index = buff.indexOf(",");
  smsStatus = buff.substring(1, index - 1);
  buff.remove(0, index + 2);

  senderNumber = buff.substring(0, 13);
  buff.remove(0, 19);

  receivedDate = buff.substring(0, 20);
  buff.remove(0, buff.indexOf("\r"));
  buff.trim();

  index = buff.indexOf("\n\r");
  buff = buff.substring(0, index);
  buff.trim();
  msg = buff;
  buff = "";
  msg.toLowerCase();

  //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
  String tempcmd = msg.substring(0, 3);
  if (tempcmd.equals("r1=") || tempcmd.equals("r2=") || tempcmd.equals("r3=") || tempcmd.equals("r4=") || tempcmd.equals("r5=")) {

    tempPhone = msg.substring(3, 16);
    msg = tempcmd;
    //debugPrint(msg);
    //debugPrint(tempPhone);
  }
  //NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
}




/*******************************************************************************
 * Reply function
 * Send an sms
 ******************************************************************************/
void Reply(String text, String phoneNumber) {
  //return;
  sim800.print("AT+CMGF=1\r");
  delay(1000);
  sim800.print("AT+CMGS=\"");     //open quotes
  sim800.print("+254704680502");  //insert number
  sim800.print("\"\r\n");         //close quotes.. return
  //sim800.print("AT+CMGS=\"phoneNo[i]\"\r");
  delay(1000);
  sim800.print(text);
  delay(100);
  sim800.write(0x1A);  //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(3000);

  sim800.print("AT+CMGS=\"");     //open quotes
  sim800.print("+254708755511");  //insert number
  sim800.print("\"\r\n");         //close quotes.. return
  //sim800.print("AT+CMGS=\"phoneNo[i]\"\r");
  delay(1000);
  sim800.print(text);
  delay(100);
  sim800.write(0x1A);  //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");

  /*************************************************************
   * only upto 70 messages can be stored in sim800l memory
   * after the memory is full, then no new sms will be received
   * until you free up the sms memory by deleting some sms.
   * that's why the below written command deletes all the sms
   * from the memory automatically. 
   *************************************************************/
  //sim800.print("AT+CMGD=1,4");


  smsStatus = "";
  senderNumber = "";
  receivedDate = "";
  msg = "";
  tempPhone = "";
}


/*******************************************************************************
 * writeToEEPROM function:
 * Store registered phone numbers in EEPROM
 ******************************************************************************/
void writeToEEPROM(int addrOffset, const String &strToWrite) {
  byte len = 13;  //strToWrite.length();
  //EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++) {
    //meee
    //Serial.print(addrOffset + i);
    //Serial.println(strToWrite[addrOffset + i]);
    EEPROM.write(addrOffset + i, strToWrite[i]);
  }

  // actually write the content of byte-array cache to
  // hardware flash.  flash write occurs if and only if one or more byte
  // in byte-array cache has been changed, but if so, ALL 512 bytes are
  // written to flash
  EEPROM.commit();
}



/*******************************************************************************
 * readFromEEPROM function:
 * Store phone numbers in EEPROM
 ******************************************************************************/
String readFromEEPROM(int addrOffset) {
  int len = 13;
  char data[len + 1];
  for (int i = 0; i < len; i++) {
    data[i] = EEPROM.read(addrOffset + i);
  }
  data[len] = '\0';
  return String(data);
}




/*******************************************************************************
 * comparePhone function:
 * compare phone numbers stored in EEPROM
 ******************************************************************************/
boolean comparePhone(String number) {
  boolean flag = 0;
  //--------------------------------------------------
  for (int i = 0; i < totalPhoneNo; i++) {
    phoneNo[i] = readFromEEPROM(offsetPhone[i]);
    if (phoneNo[i].equals(number)) {
      //Serial.println(phoneNo[i]);
      flag = 1;
      break;
    }
  }
  //--------------------------------------------------
  return flag;
}




/*******************************************************************************
 * compareWithoutCountryCode function:
 * compare phone numbers stored in EEPROM
 ******************************************************************************/
boolean compareWithoutCountryCode(String number) {
  boolean flag = 0;
  //--------------------------------------------------
  for (int i = 0; i < totalPhoneNo; i++) {
    phoneNo[i] = readFromEEPROM(offsetPhone[i]);
    //remove first 3 digits (country code)
    phoneNo[i].remove(0, 3);
    //Serial.println("meee1: "+phoneNo[i]);
    if (phoneNo[i].equals(number)) {
      //Serial.println(phoneNo[i]);
      flag = 1;
      break;
    }
  }
  //--------------------------------------------------
  return flag;
}



/*******************************************************************************
 * debugPrint function:
 * compare phone numbers stored in EEPROM
 ******************************************************************************/
void debugPrint(String text) {
  if (DEBUG_MODE == 1)
    Serial.println(text);
}





/*******************************************************************************
 * setup function
 ******************************************************************************/
void sendLocation(String phoneNumber) {
  //-----------------------------------------------------------------
  // Can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
        break;
      }
    }
  }
  //-----------------------------------------------------------------

  //-----------------------------------------------------------------
  //If newData is true
  if (newData) {
    newData = false;
    String latitude = String(gps.location.lat(), 6);
    String longitude = String(gps.location.lng(), 6);
    String speed = String(gps.speed.kmph());

    String text = "Latitude= " + latitude;
    text += "\n\r";
    text += "Longitude= " + longitude;
    text += "\n\r";
    text += "Speed= " + String(gps.speed.kmph()) + " km/h";
    text += "\n\r";
    text += "Altitude= " + String(gps.altitude.meters()) + " meters";
    text += "\n\r";
    text += "Connected Satellites= " + String(gps.satellites.value());
    text += "\n\r";
    text += "http://maps.google.com/maps?q=loc:" + latitude + "," + longitude;

    debugPrint(text);
    //delay(300);
    ///*
    Reply(text, phoneNumber);
    //*/
  }
  //-----------------------------------------------------------------
}
