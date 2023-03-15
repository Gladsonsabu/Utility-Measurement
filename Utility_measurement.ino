#include <WiFi.h>
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

#define HX710_SCK_1 25
#define HX710_SDI_1 26
#define HX710_SCK_2 32
#define HX710_SDI_2 33
#define SCT_DET 17 
#define YFS401 16  

const char* ssid = "My sam S8";               // your network SSID (name) 
const char* password = "agesldwaw1";          // your network password

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
const unsigned long MQ_LIM = 0;

unsigned long SDC_prev_time = 0;
unsigned short SDC_update_time = 5000;
unsigned int readingID = 0;
String dataMessage;
bool SD_flag = 1;

bool WIFI_flag = 1;

unsigned long ChannelNumber = 2058553;                    // These parameters are for thingspeak details
const char * WriteAPIKey = "YHSFZ0TC8CPU3W7W";
const char * readAPIKey = "34W6LGLIFXD56MPM"; 
unsigned long TS_prev_time = 0;
unsigned short TS_update_time = 15000;

char Time[ ] = "TIME:00:00:00";                           // These parameters are for network time
char Date[ ] = "DATE:00/00/2000";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;

unsigned long lcdpreviousMillis = 0;                      // These parameters are for lcd configuration
const unsigned short int lcddelay= 2500;
byte lcdstate = 5;
bool sweepdir = 1;

float AVG_ARR[4] = {0,0,0,0};

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
long WIFI_CONN_Timeout = 0;
void setup() {
  Serial.begin(115200);
  pinMode(SCT_DET,INPUT);  //Define the pin mode
  pinMode(YFS401,INPUT);
  lcd.begin(16, 2);  // initialize the lcd
  lcd.setBacklight(255);
  WiFi.mode(WIFI_STA);
  if(WiFi.status() != WL_CONNECTED){
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
    while(WiFi.status() != WL_CONNECTED){
      delay(2000);  
      Serial.print(".");
      if((millis() - WIFI_CONN_Timeout ) > 60000 ){
        WIFI_flag = 0;
        break;
      }
    } 
    lcd.noBlink();
    if(WIFI_flag){
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
  delay(3000);
  ThingSpeak.begin(client);
  timeClient.begin();
  ADS.setGain(GAIN_TWO);
  ADS.begin();

  PGuage1.begin(HX710_SDI_1, HX710_SCK_1);
  PGuage2.begin(HX710_SDI_2, HX710_SCK_2);
  AVG_IRMS.begin();
  AVG_flowRate.begin();
  AVG_Q.begin();

  if(!SD.begin()){
      Serial.println(F("Card Mount Failed"));
      SD_flag = 1;
  }
 
  Sensortest();
  LCDDISPLAY();
  xTaskCreatePinnedToCore(LoopA, "Task1", 10000, NULL, 1, &Task1, 0);  // xTCPTC(Task Fn, Task name, Task Params, priority, task handle/constructor, which core)
  delay(500); 
  // xTaskCreatePinnedToCore(LoopB, "Task2", 10000, NULL, 1, &Task2, 1);  
  // delay(500); 
  
  // put your setup code here, to run once:
  delay(3000);
}

void LoopA( void * pvParameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());

  for(;;)
  {
    if((WiFi.status() != WL_CONNECTED) || (WiFi.status() == WL_CONNECTION_LOST)){
      WiFi.begin(ssid, password);                            // set for the next iteration
      Serial.println(F("\n Not Connected... Moving ahead"));
      WIFI_flag = 0;
    }
    else{
      Serial.println(F("\nConnected."));
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP());
      WIFI_flag = 1;
    }

    if(WIFI_flag){
      if((millis() - TS_prev_time)>= TS_update_time){
        ThingSpeak.setField(1, AVG_ARR[0]);
        ThingSpeak.setField(2, AVG_ARR[1]);
        ThingSpeak.setField(3, AVG_ARR[2]);
        ThingSpeak.setField(4, LPG_STAT);
        Tspk_Update(ThingSpeak.writeFields(ChannelNumber, WriteAPIKey));
        TS_prev_time = millis();
        AVG_IRMS.reset();
        AVG_flowRate.reset();
        AVG_Q.reset();
      }
    }
    else{
      if(SD.cardType() == CARD_NONE){
        Serial.println(F("No SD card attached"));
        Serial.println(F("Data sample is lost"));
        SD_flag = 0;
      } 
      else{
        if((millis() - SDC_prev_time)>= SDC_update_time){
          SDClogger();
          AVG_IRMS.reset();
          AVG_flowRate.reset();
          AVG_Q.reset();  
          SDC_prev_time = millis();
        }
      }
    }
  } 
}

void loop() {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  time_update();
  if(digitalRead(SCT_DET)){
    SCT_flag = 1;
    IRMS = Irms();
    AVG_ARR[0] = AVG_IRMS.reading(IRMS);
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
    SCT_flag = 0;
  } 
  LCDDISPLAY();

  Serial.print(F("Measuring Flow:"));
  YFSpreviousMillis = millis(); 
  attachInterrupt(digitalPinToInterrupt(YFS401), pulseCounter, FALLING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(YFS401));
  flowRate = ((1000.0 / (millis() - YFSpreviousMillis)) * pulseCount) / calibrationFactor;
  // //flowMilliLitres = (flowRate   / 60) * 2000; //2000 is the total time elapsed in each succesive measurement
  // //totalMilliLitres += flowMilliLitres;
  Serial.println(flowRate);
  AVG_ARR[1] = AVG_flowRate.reading(flowRate);
  pulseCount = 0;
  flowMilliLitres = (flowRate   / 60) * (millis() - YFSpreviousMillis); //2000 is the total time elapsed in each succesive measurement
  Serial.print(F("flowMilliLitres:"));
  Serial.println(flowMilliLitres);

  MQ = ADS.readADC_SingleEnded(2);
  if( MQ >= MQ_LIM){
    Serial.println(F("LPG leak Detected..."));
    LPG_STAT = 1;
  } 
  else{
    Serial.println(F("No LPG leak"));
    LPG_STAT = 0;
  }

  Q = Press_Diff();
  AVG_ARR[2] = AVG_Q.reading(Q);
  Serial.print(F("LPG Flow rate --> "));
  Serial.println(Q);
  

  
  LCDDISPLAY();
  delay (1000);
  LCDDISPLAY();
  delay (1000);
  LCDDISPLAY();
  delay (1000);
  LCDDISPLAY();
  delay (1000);
}



double Press_Diff(){    // returns the flow rate as 32 bit 
  PG1_Raw = PGuage1.read_average(5);
  PG2_Raw = PGuage2.read_average(5);
  Serial.print(F("change in pressure ==>"));
  Serial.println((PG1_Raw - PG2_Raw)* ADC_multiplier );
  //return(Kv * sqrt((PG1_Raw - PG2_Raw)/S));     //Flow rate =Flow factor * (sqrt(pressure difference / Specific Gravity))
  return(A1*sqrt((2*(PG1_Raw - PG2_Raw)*ADC_multiplier)/(RHO * (sq((A1/A2))-1))));
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
  // SD_flag = 0;
  // SCT_flag = 0;
  // YFS_flag = 0;
  // MassFlow_flag = 0;
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
  if(!digitalRead(SCT_DET)){
    SCT_flag = 0;
    Serial.println(F("No SCT attached"));
  }
  if(digitalRead(YFS401)){
    YFS_flag = 0;
    Serial.println(F("No YFS attached"));
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
  
  Serial.print(F("SCT ->"));
  Serial.println(SCT_flag );
  Serial.print(F("YFS ->"));
  Serial.println(YFS_flag );
  Serial.print(F("SDC ->"));
  Serial.println(SD_flag); 
  Serial.print(F("LPG ->"));
  Serial.println(MassFlow_flag); 

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
  File file = SD.open("/data.txt");
    if(!file) {
      Serial.println(F("File doens't exist"));
      Serial.println(F("Creating file..."));
      writeFile(SD, "/data.txt", "Reading ID, Date, Hour, IRMS, Water, Mass Flow, SCT, YFS, Gauge \r\n");
    }
    else {
      Serial.println(F("File already exists"));
      dataMessage = String(readingID) + "," + String(Date) + "," + String(Time) + "," + String(IRMS) + "," + String(flowRate) + "," + String(Q) + "," + String(SCT_flag) + "," + String(YFS_flag) + "," + String(MassFlow_flag) + "\r\n";
      Serial.print(F("Save data: "));
      Serial.println(dataMessage);
      appendFile(SD, "/data.txt", dataMessage.c_str());  
      readingID++;
    }
    file.close();
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
