#include <WiFi.h>
#include <PubSubClient.h>
#include "ThingSpeak.h"
#include <WiFiUdp.h>
#include <NTPClient.h>               
#include <TimeLib.h> 
#include <LiquidCrystal_PCF8574.h>
#include <Adafruit_ADS1X15.h>
#include <Filters.h> 
#include <movingAvg.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "HX711.h"
// #include "HX710B.h"
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define HX710_SCK_1 25
#define HX710_SDI_1 26
#define HX710_SCK_2 32
#define HX710_SDI_2 33
#define SCT_DET 17 
#define YFS401 16  

const char* ssid = "Alvin";               // your network SSID (name) 
const char* password = "0568098725";          // your network password

const char* mqtt_server = "192.168.0.8";  // IP of the MQTT broker
const char* PWR_topic = "Node1/measurement/IRMS";
const char* Water_topic = "Node1/measurement/Water";
const char* LPG_topic = "Node1/measurement/LPG";
const char* LPG_LEAK = "Node1/Status/LPG_Leak";
const char* mqtt_username = "BR_UTILITY"; // MQTT username
const char* mqtt_password = "UTILITYPASS"; // MQTT password
const char* clientID = "client_livingroom"; // MQTT client ID


const float FACTOR = 50;
const float multiplier = 0.0825F;             //0.2527F;//0.0625F
float IRMS, P;
bool SCT_flag = 1;

unsigned long YFSpreviousMillis = 0;
float calibrationFactor = 90;                   //You can change according to your datasheet of YFFSXX
volatile byte pulseCount =0;  
float flowRate = 0.0;
unsigned int flowMilliLitres =0;
unsigned long totalMilliLitres = 0;
bool YFS_flag = 1;

//Flow direction is fixed from PG1 --> PG2
uint32_t PG1_Raw, PG2_Raw = 0;
float Pdiff = 0;
double Q = 0;
const float ADC_multiplier = 0.0011;      //0.0011920929
const float A1 = 0;
const float A2 = 0;                       //A1>A2
const float RHO = 1.8315;                 // Flud Density
float Kv = 0;                             //Flow Factor
float S = 0;                              // Specific Gravity
bool MassFlow_flag = 1;

uint16_t LPG_STAT = 0;
unsigned long MQ = 0;
const unsigned long MQ_LIM = 20000;

unsigned long SDC_prev_time = 0;
unsigned short SDC_update_time = 5000;
unsigned int readingID = 0;
String SD_CONFIG_SSID = "";
String SD_CONFIG_PWD = "";
String SD_CONFIG_TSPKCHNUM = "";
String SD_CONFIG_WAPI = "";
String SD_CONFIG_RAPI = "";
String SD_CONFIG_MQTT_SERVER = "";  // IP of the MQTT broker
String SD_CONFIG_TOPIC_PWR = "";
String SD_CONFIG_TOPIC_WATER = "";
String SD_CONFIG_TOPIC_LPG = "";
String SD_CONFIG_TOPIC_LPG_LEAK = "";
String SD_CONFIG_MQTT_USR = ""; // MQTT username
String SD_CONFIG_MQTT_PWD = ""; // MQTT password
String SD_CONFIG_CLIENT_ID = ""; // MQTT client ID
bool SD_flag = 1;
static char charbuff[512];

bool WIFI_flag = 1;
long WIFI_CONN_Timeout = 0;

unsigned long ChannelNumber = 2081108;                    // These parameters are for thingspeak details
const char * WriteAPIKey = "I9RGBNTOC02L5PTO";
const char * readAPIKey = "P3HI4H0E0UY0R86A"; 
unsigned long TS_prev_time = 0;
unsigned short TS_update_time = 20000;

char Time[ ] = "TIME:00:00:00";                           // These parameters are for network time
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;

unsigned long lcdpreviousMillis = 0;                      // These parameters are for lcd configuration
const unsigned short int lcddelay= 2500;
byte lcdstate = 5;
bool sweepdir = 1;

int AVG_ARR_INT[4] = {0,0,0,0};
float AVG_ARR_FLOAT[4] = {0,0,0,0};

double Press_Diff(void);
void pulseCounter(void);
float Irm(void);
void lcdsweep(void);
void Tspk_Update(int);
void LCDDISPLAY(void);
void Tspk_Update(int);
void Sensortest(void);
void time_update(void);
void SDClogger(void);
void SDC_SYSconfig(void);
void StageValues(void);
void Paramupdate(char *, short);

TaskHandle_t Task1;
// TaskHandle_t Task2;
movingAvg AVG_IRMS(10);
movingAvg AVG_flowRate(10);
movingAvg AVG_Q(10);
WiFiClient  client;
WiFiUDP ntpUDP;
LiquidCrystal_PCF8574 lcd(0x3F);
Adafruit_ADS1115 ADS;
HX711 PGuage1;
HX711 PGuage2;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 19800, 60000);

WiFiClient wifiClient;            // Initialise the WiFi and MQTT Client objects
PubSubClient MQTTclient(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker


void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);  // initialize the lcd
  lcd.setBacklight(255);
  if(!SD.begin()){
      Serial.println(F("Card Mount Failed"));
      SD_flag = 0;
  }
  else{
    SD_flag = 1;
  }
  lcd.setCursor(0, 0);
  lcd.print("   EXECUTING");
  lcd.setCursor(0, 1);
  lcd.print("   config.exe");
  SDC_SYSconfig();
  delay(3000);
  pinMode(SCT_DET,INPUT);  //Define the pin mode
  pinMode(YFS401,INPUT);
  WiFi.mode(WIFI_STA);
  if(WiFi.status() != WL_CONNECTED){
    lcd.clear();
    Serial.print("Attempting in connecting to ");
    Serial.print(ssid);
    lcd.setCursor(0, 0);
    lcd.print("Searching -> N/W");
    lcd.setCursor(0, 1);
    lcd.print(ssid);
    lcd.setCursor(15, 1);
    WiFi.begin(ssid, password); 
    lcd.blink();
    WIFI_CONN_Timeout = millis();  
    while(WiFi.status() != WL_CONNECTED){                   // attempts to connect to the provided SSID and PWD for 60 seconds. Moves ahead after the timeout
      delay(2000);  
      Serial.print(".");
      if((millis() - WIFI_CONN_Timeout ) > 60000 ){         // Case definitionfor timeout
        WIFI_flag = 0;
        break;
      }
    } 
    lcd.noBlink();
    if(WIFI_flag){                                          // If connected display the details
      Serial.println("\nConnected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP()); 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connected... IP");
      lcd.setCursor(1, 1);
      lcd.print(WiFi.localIP());
    }
    else{
      Serial.println("\n Not Connected... Moving ahead");
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("   Connection");
      lcd.setCursor(4, 1);
      lcd.print("time-out");
    }
  }
  MQTTclient.setServer(mqtt_server, 1883);
  MQTTclient.setCallback(callback);
  delay(3000);
  ThingSpeak.begin(client);                                   // Starting thinks speak client
  timeClient.begin();                                         // Starting the WEB UDP time Client to fetch the time data from web
  ADS.begin();                                                // initialising the 16 bit ADC
  ADS.setGain(GAIN_TWO);

  PGuage1.begin(HX710_SDI_1, HX710_SCK_1);
  PGuage2.begin(HX710_SDI_2, HX710_SCK_2);
  PGuage1.set_offset(0);                                      // Setting tare offset to 0 for pressure Guage 1
  PGuage2.set_offset(0);                                      // Setting tare offset to 0 for pressure Guage 2
  PGuage1.set_scale(419.44f);                                 // Setting scale factor to 419.438 for PG 1
  PGuage2.set_scale(419.44f);                                 // Setting scale factor to 419.438 for PG 2

  AVG_IRMS.begin();                                         
  AVG_flowRate.begin();                                      
  AVG_Q.begin();                                            
 
  Sensortest();
  LCDDISPLAY();
  xTaskCreatePinnedToCore(LoopA, "Task1", 10000, NULL, 1, &Task1, 0);  // xTCPTC(Task Fn, Task name, Task Params, priority, task handle/constructor, which core)
  delay(500); 
  // put your setup code here, to run once:
  delay(3000);
}

void LoopA( void * pvParameters ){
  Serial.print("LoopA is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {
    if((WiFi.status() != WL_CONNECTED) || (WiFi.status() == WL_CONNECTION_LOST)){         // checks for the event of WiFi disconnection
      WiFi.begin(ssid, password);                            // set for the next iteration
      if(WIFI_flag){
        Serial.println(F("\n Not Connected... Moving ahead"));
      }
      WIFI_flag = 0;
    }
    else{
        if(!WIFI_flag)
        {
          Serial.println(F("\nConnected."));
          Serial.print(F("IP address: "));
          Serial.println(WiFi.localIP());
          WIFI_flag = 1;
        }
      }
    
    if(WIFI_flag){                                                                      // if WiFi is available then push data to thinks speak
      if((millis() - TS_prev_time)>= TS_update_time){                                   // thinksspeak update is set once every 20s. 15s is the minimum 
        StageValues();                                                                  // converts the measured value in integer to a valid float
        Serial.println(F("Push to Thinksspeak"));

        ThingSpeak.setField(1, AVG_ARR_FLOAT[0]);
        ThingSpeak.setField(2, AVG_ARR_FLOAT[1]);
        ThingSpeak.setField(3, AVG_ARR_FLOAT[2]);
        ThingSpeak.setField(4, LPG_STAT);
        Tspk_Update(ThingSpeak.writeFields(ChannelNumber, WriteAPIKey));
        TS_prev_time = millis();
        AVG_IRMS.reset();
        AVG_flowRate.reset();
        AVG_Q.reset();
        Serial.println(F("Thinksspeak Stack push ececuted"));
      }
    }
    else{
      if(SD.cardType() == CARD_NONE){
        Serial.println(F("No SD card attached"));
        Serial.println(F("Data sample is lost"));
        SD_flag = 0;
      } 
      else{
        if((millis() - SDC_prev_time)>= SDC_update_time){                 // Sdcard logging update is set once every 5s
          StageValues();                                                  // converts the measured value in integer to a valid float
          SDClogger();                                                    // function to log the measured details to SD card
          AVG_IRMS.reset();                                               // Flush the existing/running moving average filter for Current
          AVG_flowRate.reset();                                           // Flush the existing/running moving average filter for water flow
          AVG_Q.reset();                                                  // Flush the existing/running moving average filter for Pressure difference
          SDC_prev_time = millis();
        }
      }
    }
    delay(1000);
  } 
}

void loop() {
  // Serial.print("Task running on core ");
  // Serial.println(xPortGetCoreID());
  time_update();
  Sensortest();
  if(SCT_flag){
    IRMS = Irms();
    AVG_ARR_INT[0] = AVG_IRMS.reading(IRMS*100);
    P = 230.0 * IRMS;
    Serial.print(F("Average RMS Current: "));
    Serial.print(IRMS, 3);
    Serial.print(F(" A.\t\t"));
    Serial.print(F("Power:"));
    Serial.print(P, 3);
    Serial.println(F(" W"));
  }
  else{
    Serial.println(F("SCT disconnected"));
  } 
  LCDDISPLAY();

  Serial.print(F("Measuring Flow:"));
  YFSpreviousMillis = millis(); 
  attachInterrupt(digitalPinToInterrupt(YFS401), pulseCounter, FALLING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(YFS401));
  flowRate = ((1000.0 / (millis() - YFSpreviousMillis)) * pulseCount) / calibrationFactor;
  Serial.println(flowRate);
  AVG_ARR_INT[1] = AVG_flowRate.reading(flowRate*100); // multiplyiing 100 to compensate the conversion to int from float
  Serial.print(F("AVG flow rate:"));
  Serial.println(AVG_ARR_INT[1]);
  pulseCount = 0;
  flowMilliLitres = (flowRate   / 60) * (millis() - YFSpreviousMillis); //2000 is the total time elapsed in each succesive measurement
  Serial.print(F("flowMilliLitres:"));
  Serial.println(flowMilliLitres);

  MQ = ADS.readADC_SingleEnded(2);  
  Serial.print(F("RAW LPG ADC = "));
  Serial.println(MQ);
  if( MQ >= MQ_LIM){
    Serial.println(F("LPG leak Detected..."));
    LPG_STAT = 1;
  } 
  else{
    Serial.println(F("No LPG leak"));
    LPG_STAT = 0;
  }

  Q = Press_Diff();
  AVG_ARR_INT[2] = AVG_Q.reading(Q);
  Serial.print(F("LPG Flow rate --> "));
  Serial.println(Q);
  
  
  LCDDISPLAY();
  delay (1000);
}



double Press_Diff(){    // returns the flow rate as 32 bit 
  PG1_Raw = PGuage1.get_units(10);
  PG2_Raw = PGuage2.get_units(10);
  Serial.print(F("change in pressure ==>"));
  Serial.println(PG1_Raw - PG2_Raw);
  //return(Kv * sqrt((PG1_Raw - PG2_Raw)/S));     //Flow rate =Flow factor * (sqrt(pressure difference / Specific Gravity))
  return(A1*sqrt((2*(PG1_Raw - PG2_Raw))/(RHO * (sq((A1/A2))-1))));
} 

void pulseCounter(){     // ISR for YFS
  pulseCount++;
}

float Irms(){            // ISR for SCT100 returns IRMS a float
  float voltage, Curr_temp;
  float Curr_sum = 0;
  long t_start = millis();
  uint8_t counter = 0;
  while (millis() - t_start < 500){
    voltage = ADS.readADC_Differential_0_1() * multiplier;
    //Serial.println(voltage);
    Curr_temp = voltage * FACTOR;
    Curr_temp /= 1000.0;
    Curr_sum += sq(Curr_temp);
    counter = counter +1;
    delay(4);
  }
  // Serial.print("Samples taken for IRMS:");
  // Serial.println(counter);
  return(sqrt(Curr_sum / counter));
}

void LCDDISPLAY(){       // displays info @ lcd only when called and a defined time has elapsed. It cycles through diffrent datasets.
  if(millis()-lcdpreviousMillis >= lcddelay){
    lcdsweep();
    switch(lcdstate){
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F(" IRMS -> "));
      lcd.print(IRMS);
      lcd.print(F("A"));
      lcd.setCursor(0,1);
      lcd.print(F("POWER -> "));
      lcd.print(P);
      lcd.print(F("W"));
      lcdstate = 2;
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Liquid Flow Rate"));
      lcd.setCursor(3,1);
      lcd.print(flowRate);
      lcd.setCursor(8, 1);
      lcd.print(F("L/Min"));
      lcdstate = 3;
      break;
    case 3:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F(" LPG Mass Flow "));
      lcd.setCursor(3,1);
      lcd.print(Q);     
      lcd.setCursor(8, 1);
      lcd.print(F("SCCM"));
      lcdstate = 4;
      break;
    case 4:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(Time);
      lcd.setCursor(0, 1);
      lcd.print(Date);
      lcdstate = 5;
      break;
    case 5:
      // Sensortest();
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print(F("SCT-"));
      lcd.print(SCT_flag);
      lcd.print(F("   YFS-"));
      lcd.print(YFS_flag);
      lcd.setCursor(1,1);
      lcd.print(F("SDC-"));
      lcd.print(SD_flag);
      lcd.print(F("   LPG-"));
      lcd.print(MassFlow_flag);
      lcdstate = 1;
      break;
    default:
      lcd.clear();
      lcdstate = 1;
      break;
    } 
    lcdpreviousMillis = millis(); 
  }
}

void Sensortest(){       // test all connected devices and sets error flags according to it.
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println(F("No SD card attached"));
      SD_flag = 0;
  }
  else{
    SD_flag = 1;
  }
  if(!digitalRead(SCT_DET)){
    SCT_flag = 0;
    Serial.println(F("No SCT attached"));
  }
  else{
    SCT_flag = 1;
  }
  if(digitalRead(YFS401)){
    YFS_flag = 0;
    Serial.println(F("No YFS attached"));
  }
  else{
    YFS_flag = 1;
  }

  if ((PGuage1.is_ready() != 1) || (PGuage2.is_ready() != 1)){ 
    MassFlow_flag = 0;
    if((PGuage1.is_ready() != 1) && (PGuage2.is_ready() != 1)){
      Serial.println(F("Both HX710B not Found !"));
    }
    else if(PGuage1.is_ready() != 1){
      Serial.println(F("HX710B 1 not Found !"));
    }
    else{
      Serial.println(F("HX710B 2 not Found !"));
    }  
  }
  else{
    MassFlow_flag = 1;
  }
  
  Serial.print(F("SCT ->"));
  Serial.print(SCT_flag ); 
  Serial.print(F("     "));
  Serial.print(F("YFS ->"));
  Serial.print(YFS_flag );
  Serial.print(F("     "));
  Serial.print(F("SDC ->"));
  Serial.print(SD_flag);
  Serial.print(F("     ")); 
  Serial.print(F("LPG ->"));
  Serial.println(MassFlow_flag); 

}

void StageValues(){
  for(short i =0 ; i< sizeof(AVG_ARR_INT) / sizeof(AVG_ARR_INT[0]); i++){
    AVG_ARR_FLOAT[i] = (float) (((float) AVG_ARR_INT[i])/100);
    // Serial.println(AVG_ARR_FLOAT[i]);
  }
}

void Tspk_Update(int STATUS){ //
  if(STATUS == 200){
      Serial.println(F("Channel update successful."));
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(STATUS));
    }
}

void lcdsweep(){
  if(sweepdir == 1){
   for(short i=0; i<=16; i++){     // Performs the shifting function
      lcd.print(F(" "));
      lcd.setCursor(i,0);
      lcd.print(F(" "));
      lcd.setCursor(i,1);
      delay(20);
    }
    sweepdir = 0;
  }
  else{
    for(short i=16; i>=0; i--){     // Performs the shifting function
      lcd.print(F(" "));
      lcd.setCursor(i,0);
      lcd.print(F(" "));
      lcd.setCursor(i,1);
      delay(20);
    }
    sweepdir = 1;
  }
}

void time_update(){
  timeClient.update();
  unsigned long unix_epoch = timeClient.getEpochTime();    // Get Unix epoch time from the NTP server
  second_ = second(unix_epoch);
 
  minute_ = minute(unix_epoch);
  hour_   = hour(unix_epoch);
  day_    = day(unix_epoch);
  month_  = month(unix_epoch);
  year_   = year(unix_epoch);

  Time[12] = second_ % 10 + 48;
  Time[11] = second_ / 10 + 48;
  Time[9]  = minute_ % 10 + 48;
  Time[8]  = minute_ / 10 + 48;
  Time[6]  = hour_   % 10 + 48;
  Time[5]  = hour_   / 10 + 48;

  Date[5]  = day_   / 10 + 48;
  Date[6]  = day_   % 10 + 48;
  Date[8]  = month_  / 10 + 48;
  Date[9]  = month_  % 10 + 48;
  Date[13] = (year_   / 10) % 10 + 48;
  Date[14] = year_   % 10 % 10 + 48;
}

void SDClogger(){
  String dataMessage;
  dataMessage.reserve(128);
  File file = SD.open("/data.txt");
    if(!file) {
      Serial.println(F("File doens't exist"));
      Serial.println(F("Creating file..."));
      writeFile(SD, "/data.txt", "| Reading ID |    Date    |   Time   |  IRMS  |  Water  |  Mass Flow  |  SCT  |  YFS  |  Gauge  |  LPG LEAK |\r\n");
    }
    else {
      Serial.println(F("File already exists"));
      // dataMessage = "|     " + String(readingID) + "     | " + String(Date).substring(5) + " | " + String(Time).substring(5) + " |  " + String(IRMS) + "  |  " + String(flowRate) + "   |     " + String(Q) + "  |     " + String(SCT_flag) + "   |   " + String(YFS_flag) + "   |    " + String(MassFlow_flag) + "    |     " + String(LPG_STAT) + "     |" +"\r\n";
      dataMessage = "|     " + String(readingID) + "     | " + String(Date).substring(5) + " | " + String(Time).substring(5) + " |  " + String(AVG_ARR_FLOAT[0]) + "  |  " + String(AVG_ARR_FLOAT[1]) + "   |     " + String(AVG_ARR_FLOAT[2]) + "  |     " + String(SCT_flag) + "   |   " + String(YFS_flag) + "   |    " + String(MassFlow_flag) + "    |     " + String(LPG_STAT) + "     |" +"\r\n";
      Serial.print(F("Save data: "));
      Serial.println(dataMessage);
      appendFile(SD, "/data.txt", dataMessage.c_str());  
      readingID++;
    }
    file.close();
}

void SDC_SYSconfig(){
  Serial.println("SYS_Config.exe initiated");
  if(SD_flag){
  ReadSet(SD, "/config.txt");
  Serial.println("SYS_Config.exe Completed execution");
  }
  
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void ReadSet(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static char Databuff[256];   
  int index = 0;
  bool RD_Begin = 0;
  short lines = 0;
  if(file){
    short len = file.size();
    int next = 0;
    while ((next = file.read()) != -1)
    {
      char nextChar = (char) next;
      if (nextChar == '\n')
      {
        Databuff[index] = '\0';
        Paramupdate(Databuff, index);
        index = 0;
        lines += 1;
      }
      else
      {
        Databuff[index] = nextChar;
        index += 1;
      }
    }
    
    Serial.print("Succefully Read ");
    Serial.println(lines);
    Serial.print(" Lines, and Parameters are updated");
    file.close();
      
  } 
  else {
    Serial.println("Failed to open file for reading");
  }
}

void Paramupdate(char *buff, short length){
  // char arr_hold[length];
  String bufferstr = "";                           n 
  bufferstr.reserve(length + 1);                     
  for(short i=0; i<=length; i++){
    // bufferstr = bufferstr + buff[i];
    bufferstr.concat(buff[i]);
  }
  String Parameter = "";                            
  String Value = "";                                
  if(bufferstr.startsWith("`")){                   
    Parameter.concat(bufferstr.substring(1,bufferstr.indexOf('=')));
    Value.concat(bufferstr.substring(bufferstr.indexOf('"')+1,bufferstr.lastIndexOf('"')));

    if(Parameter.substring(0,Parameter.length()-1) == "ssid"){              
      SD_CONFIG_SSID.reserve(Value.length()+1);                             
      SD_CONFIG_SSID.concat(Value.substring(0,Value.length()));           
      ssid = SD_CONFIG_SSID.c_str();                                       
      Serial.println("Updated ssid");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "password"){
      SD_CONFIG_PWD.reserve(Value.length()+1);
      SD_CONFIG_PWD.concat(Value.substring(0,Value.length()));
      password = SD_CONFIG_PWD.c_str();
      Serial.println("Updated PWD");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "ChannelNumber"){
      SD_CONFIG_TSPKCHNUM.reserve(Value.length()+1);
      SD_CONFIG_TSPKCHNUM.concat(Value.substring(0,Value.length()));
      ChannelNumber = atol(SD_CONFIG_TSPKCHNUM.c_str());                     
      Serial.println("Updated CHN THSPK");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "WriteAPIKey"){
      SD_CONFIG_WAPI.reserve(Value.length()+1);
      SD_CONFIG_WAPI.concat(Value.substring(0,Value.length()));
      WriteAPIKey = SD_CONFIG_WAPI.c_str();
      Serial.println("Updated WAPI");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "readAPIKey"){
      SD_CONFIG_RAPI.reserve(Value.length()+1);
      SD_CONFIG_RAPI.concat(Value.substring(0,Value.length()));
      readAPIKey = SD_CONFIG_RAPI.c_str();
      Serial.println("Updated RAPI");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "MQTT_Server"){
      SD_CONFIG_MQTT_SERVER.reserve(Value.length()+1);
      SD_CONFIG_MQTT_SERVER.concat(Value.substring(0,Value.length()));
      mqtt_server = SD_CONFIG_MQTT_SERVER.c_str();
      Serial.println("MQTT_SERVER Address");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "Topic_PWR"){
      SD_CONFIG_TOPIC_PWR.reserve(Value.length()+1);
      SD_CONFIG_TOPIC_PWR.concat(Value.substring(0,Value.length()));
      PWR_topic = SD_CONFIG_TOPIC_PWR.c_str();
      Serial.println("Updated Power usage topic");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "Topic_Water"){
      SD_CONFIG_TOPIC_WATER.reserve(Value.length()+1);
      SD_CONFIG_TOPIC_WATER.concat(Value.substring(0,Value.length()));
      Water_topic = SD_CONFIG_TOPIC_WATER.c_str();
      Serial.println("Updated Water flow toipc");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "Topic_LPG"){
      SD_CONFIG_TOPIC_LPG.reserve(Value.length()+1);
      SD_CONFIG_TOPIC_LPG.concat(Value.substring(0,Value.length()));
      LPG_topic = SD_CONFIG_TOPIC_LPG.c_str();
      Serial.println("Updated LPG Flow topic");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "Topic_LPG_Leak"){
      SD_CONFIG_TOPIC_LPG_LEAK.reserve(Value.length()+1);
      SD_CONFIG_TOPIC_LPG_LEAK.concat(Value.substring(0,Value.length()));
      LPG_LEAK = SD_CONFIG_TOPIC_LPG_LEAK.c_str();
      Serial.println("Updated Leak Status Topic");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "MQTT_USRNM"){
      SD_CONFIG_MQTT_USR.reserve(Value.length()+1);
      SD_CONFIG_MQTT_USR.concat(Value.substring(0,Value.length()));
      mqtt_username = SD_CONFIG_MQTT_USR.c_str();
      Serial.println("Updated MQTT User Name");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "MQTT_PASS"){
      SD_CONFIG_MQTT_PWD.reserve(Value.length()+1);
      SD_CONFIG_MQTT_PWD.concat(Value.substring(0,Value.length()));
      mqtt_password = SD_CONFIG_MQTT_PWD.c_str();
      Serial.println("Updated MQTT Password");
    }
    else if(Parameter.substring(0,Parameter.length()-1) == "MQTT_Client_ID"){
      SD_CONFIG_CLIENT_ID.reserve(Value.length()+1);
      SD_CONFIG_CLIENT_ID.concat(Value.substring(0,Value.length()));
      clientID = SD_CONFIG_CLIENT_ID.c_str();
      Serial.println("Updated CLIENT_ID");
    }
    else{                                                                 // exception for the update if any
      Serial.print("Parameter \" ");
      Serial.print(Parameter);
      Serial.println(" \" is out of bound");
    }
  }
  else{
    Serial.println("Invalid line");
  }
} 





void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      // digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!MQTTclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (MQTTclient.connect("ESP8266Client")) {
      Serial.println("connected");
      MQTTclient.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTclient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

