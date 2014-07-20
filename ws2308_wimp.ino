/*
 Weather Station using WS2308/Electric Imp
 By: David Watts
 Date: July 15th, 2014
*/ 
 
#include <Flash.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
//#include <EthernetUdp.h>
#include <util/crc16.h>
#include <avr/pgmspace.h>
#include "ws2308_wimp.h"

// Some macro definitions
#ifdef PRINT_RESPONSES
  #define PRINTRESP(item) Serial.print(item)
  #define PRINTRESPLN(item) Serial.println(item)
#else
  #define PRINTRESP(item)
  #define PRINTRESPLN(item)
#endif

// Program defines
#define READWS_INTERVAL             120000
#define LED_INTERVAL                30000

// ws2308 data lengths
#define LENGTH_UTC              5
#define LENGTH_DATE             5
#define LENGTH_TEMPF            5
#define LENGTH_HUMIDITY         5
#define LENGTH_DEWPTF           5
//#define LENGTH_BAROMPA          5
#define LENGTH_BAROMHG          5
#define LENGTH_WINDSPEEDMPH     5
//#define LENGTH_WINDDIR          5
#define LENGTH_RAININ           5
#define LENGTH_DAILYRAININ      5

#define EEPROM_UTC              20
#define EEPROM_DATE              0
#define EEPROM_TEMPF            80
#define EEPROM_DEWPTF          160
#define EEPROM_HUMIDITY        140
#define EEPROM_BAROMHG         180
#define EEPROM_RAININ          100
#define EEPROM_DAILYRAIN       120
#define EEPROM_WINDSPEEDMPH     60
#define EEPROM_WINDDIR          40
#define EEPROM_WINDGUSTMPH     200

// ws2308 commands
#define COMMAND_UTC             0x82,0x8A,0x82,0x82,0xCE  //0x0200 3 bytes
#define COMMAND_DATE            0x82,0x8A,0x92,0x82,0xCE  //0x0240 3 bytes
#ifdef INDOOR_ONLY
  #define COMMAND_TEMPF           0x82,0x8E,0x92,0x9A,0xCE//0xCA  //0x0346 2 bytes
  #define COMMAND_HUMIDITY        0x82,0x8E,0xBE,0xAE,0xCE//0xC6  //0x03FB 1 bytes
#else
  #define COMMAND_TEMPF           0x82,0x8E,0x9E,0x8E,0xCE//0xCA  //0x0373 2 bytes
  #define COMMAND_HUMIDITY        0x82,0x92,0x86,0xA6,0xCE//0xC6  //0x0419 1 bytes
#endif
#define COMMAND_DEWPTF          0x82,0x8E,0xB2,0xBA,0xCE//0xCA  //0x03CE 2 bytes   //D2
//#define COMMAND_BAROMPA         0x82,0x96,0xBA,0x8A,0xCE  //0x05E2 3 bytes
#define COMMAND_BAROMHG         0x82,0x96,0xB6,0xB6,0xCE  //0x05DD 3 bytes
#define COMMAND_WINDSPEEDMPH    0x82,0x96,0x8A,0x9E,0xCE  //0x0527 3 bytes  //F2
//#define COMMAND_WINDDIR         0x82,0x96,0x8A,0x9E,0xCE  //0x0527 3 bytes  //F2
#define COMMAND_RAININ          0x82,0x92,0xAE,0x92,0xCE  //0x04B4 3 bytes
#define COMMAND_DAILYRAININ     0x82,0x92,0xA6,0x9E,0xCE  //0x0497 3 bytes

// Useful weather calculation macros
#define ALPHA_WIND    0.9
#define ALPHA_TEMP    0.1
#define ALPHA_HUMIDITY    0.1
#define WIND_SPEED_MAX    100.0 //mph
#define ZERO_F        "0.0"
 
#define LEDHIGH digitalWrite(ledPin, HIGH)
#define LEDLOW digitalWrite(ledPin, LOW)
#define ERRORHIGH digitalWrite(errorPin, HIGH)
#define ERRORLOW digitalWrite(errorPin, LOW)


//A weather station dec
weather_data_t weatherWs2308 = { 
  "12:00:00",
  0, //hour
  0, //minute
  0, //second
  "2012-01-01",
  10, //year
  1, //month
  1, //day
  69.8, //temp
  50.0, //dewpt
  70, //humidity
  29.55, //barom
  0.0, // rain1hr
  0.0, //rain24hr
  0.0, //windspeed
  0.0, //windgust
  0, //winddir
  0, //readErr
  0, //Calc error
  0 //Post error
};

// Serial port for ws2308
SoftwareSerial mySerial(2, 3);

const char maxWindInit[4]="0.0";

const int button1 = 6;
const int button2 = 7;
const int errorPin = 7;
const int ledPin = 8;
const int rts = 4;
const int dtr = 5;

unsigned long read_update=0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0; 

//Chkmem
uint8_t * heapptr, * stackptr;


/****************************************************************************
 * Setup
 */
void setup() {

  LEDLOW;
  // initialize serial communication:
  Serial.begin(115200);
  delay(100);
  // Init weatherstation serial object
  mySerial.begin(2400);

  // initialize the digital pins
  pinMode(ledPin, OUTPUT);
  pinMode(errorPin, OUTPUT);
  pinMode(rts, OUTPUT);
  pinMode(dtr, OUTPUT);

  // Init weatherstation comm port
  digitalWrite(rts, HIGH);
  digitalWrite(dtr, LOW);
  delay(2000);
  digitalWrite(rts, LOW);
  digitalWrite(dtr, HIGH);

  // Flash some management LEDs
  for (int i=0;i<5;i++){
    LEDHIGH; 
    delay(100); 
    ERRORHIGH; 
    LEDLOW; 
    delay(100);
    ERRORLOW;
  }

  // Long watchdog as the ws2308 can be a bit sluggish
  wdt_enable(WDTO_8S); 
}

/****************************************************************************/
void loop() { 

  wdt_reset();  // Reset watchdog each cycle
  
  // Update the current number of millis since boot
  currentMillis = millis();   
  if (currentMillis < read_update) {
    read_update = currentMillis;
  } 

  // Read weather station cycle
  if ((currentMillis - read_update) > READWS_INTERVAL){   
    weatherWs2308.lastReadErrors = readws(&weatherWs2308);
    read_update = currentMillis;
    weatherWs2308.windGust=0.0; // Reset max wind speed
    weatherWs2308.readError = 0x0000; // Reset errors
  }
   
  // Now see what the imp has sent us
  if(Serial.available())
  {
    byte incoming = Serial.read();
    if(incoming == '!') {  // Ready for some weather data  
      reportWeather(&weatherWs2308); 
    }
    else if(incoming == '@') { //Special character from Imp indicating midnight local time
      midnightReset(); //Reset a bunch of variables like rain and total rain
    }
    else if(incoming == '#') { //Special character from Imp indicating a hardware reset
      delay(10000); //This will cause the system to reset because we don't pet the dog
    }
  }

  // Update the indicator and error LEDs
  if(currentMillis - previousMillis > LED_INTERVAL) { 
    previousMillis = currentMillis;
    LEDHIGH; 
    delay(500); 
    LEDLOW;
  }
  
  delay(100);
}


/****************************************************************************
 * Read weather station data
 */
byte readws(weather_data_t* weatherData){
  byte error = 0;
  int time = 0;
  while (time<10) {
    if (!((weatherWs2308.readError>>0)&0x1)) error+=!getUtc(&weatherWs2308);
    if (!((weatherWs2308.readError>>4)&0x1)) error+=!getDate(&weatherWs2308);
    if (!((weatherWs2308.readError>>8)&0x1)) error+=!getTempf(0,&weatherWs2308);
    if (!((weatherWs2308.readError>>9)&0x1)) error+=!getTempf(1,&weatherWs2308);
    if (!((weatherWs2308.readError>>10)&0x1)) error+=!getHumidity(&weatherWs2308);
    if (!((weatherWs2308.readError>>11)&0x1)) error+=!getBaromhg(&weatherWs2308);
    if (!((weatherWs2308.readError>>12)&0x1)) error+=!getRainin(0,&weatherWs2308);
    if (!((weatherWs2308.readError>>13)&0x1)) error+=!getRainin(1,&weatherWs2308);
    if (!((weatherWs2308.readError>>14)&0x1)) error+=!getWindSpeed(&weatherWs2308);
    time++;
  }
  return error;
}

/****************************************************************************
 * Report the weather
 */
void reportWeather(weather_data_t* weatherData)
{
  //Serial.print("$,date=");
  //Serial.print(weatherWs2308.date);
  //Serial.print(",utc=");
  //Serial.print(weatherWs2308.utc);
  Serial.print("$,winddir=");
  Serial.print(weatherWs2308.windDir);
  Serial.print(",windspeedmph=");
  Serial.print(weatherWs2308.windSpeed, 1);
  Serial.print(",windgustmph=");
  Serial.print(weatherWs2308.windGust, 1);
  Serial.print(",humidity=");
  Serial.print(weatherWs2308.humidity, 1);
  Serial.print(",tempf=");
  Serial.print(weatherWs2308.temp, 1);
  Serial.print(",dewptf=");
  Serial.print(weatherWs2308.dewPt, 1);  
  Serial.print(",rainin=");
  Serial.print(weatherWs2308.rain1hr, 2);
  Serial.print(",dailyrainin=");
  Serial.print(weatherWs2308.rain24hr, 2);
  Serial.print(",baromin=");
  Serial.print(weatherWs2308.barometer, 2);
  Serial.print(",");
  Serial.println("#,");
  
  //Test string
  //Serial.println("$,winddir=270,windspeedmph=0.0,windgustmph=0.0,windgustdir=0,windspdmph_avg2m=0.0,winddir_avg2m=12,windgustmph_10m=0.0,windgustdir_10m=0,humidity=998.0,tempf=-1766.2,rainin=0.00,dailyrainin=0.00,-999.00,batt_lvl=16.11,light_lvl=3.32,#,");
}

/****************************************************************************
 * Weather station data fetch - UTC
 */
byte getUtc(weather_data_t* weatherData){
  String dataStr="";  
  char inChar[3];
  int time_array[3];
  int time=0;
  int data_utc[LENGTH_UTC];
  int command_utc[] = {  COMMAND_UTC  };
  int crc_array[2];
  int last_hour = weatherData->hour;
  int last_minute = weatherData->minute;  

  for (int i=0;i<1;i++) {  
    while ((read_data(command_utc,data_utc)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_utc);
    }
  }

  if (time>4) {
    return 0;
  }  
  else {    
    weatherData->hour = (data_utc[2]>>4)*10 + (data_utc[2]&0xF);
    weatherData->minute = (data_utc[1]>>4)*10 + (data_utc[1]&0xF);
    weatherData->second = (data_utc[0]>>4)*10 + (data_utc[0]&0xF);
    itoa(weatherData->hour,inChar,10);
    if (weatherData->hour<10) dataStr.concat("0");
    dataStr.concat(inChar);
    dataStr.concat(":");
    itoa(weatherData->minute,inChar,10);
    if (weatherData->minute<10) dataStr.concat("0");
    dataStr.concat(inChar);
    dataStr.concat(":");
    itoa(weatherData->second,inChar,10);
    if (weatherData->second<10) dataStr.concat("0");
    dataStr.concat(inChar);    

    //Do chk on data
    if ((weatherData->hour>=0) && (weatherData->hour<24) &&
      ((weatherData->hour>=last_hour) || ((last_hour==23) && (weatherData->hour==0))) &&
      (weatherData->minute>=0) && (weatherData->minute<60) &&
      ((weatherData->minute>=last_minute) || ((last_minute==59) && (weatherData->minute==0))) && 
      (weatherData->second>=0) && (weatherData->second<60)) {
      for (int c=0;c<dataStr.length();c++) {
        weatherData->utc[c]=dataStr[c];
      }
      //writemem(dataStr,EEPROM_UTC);
      weatherData->readError = (weatherData->readError | (0x0001));      
      return 1;
    } 
    else {
      return 0;
    } 

  }   

}  

/****************************************************************************
 * Weather station data fetch - date
 */
byte getDate(weather_data_t* weatherData){
  String dataStr="";  
  char inChar[3];
  //int date_array[3];
  int time=0;
  int data_date[LENGTH_DATE];
  int command_date[] = {
    COMMAND_DATE  };
  int crc_array[2];
  int last_year = weatherData->year;
  int last_month = weatherData->month;
  int last_day = weatherData->day;  
  
  for (int i=0;i<2;i++) {  
    while ((read_data(command_date,data_date)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_date);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {
    return 0;
  }
  else {

    weatherData->year = (data_date[2]>>4)*10 + (data_date[2]&0xF);
    weatherData->month = (data_date[1]>>4)*10 + (data_date[1]&0xF);
    weatherData->day = (data_date[0]>>4)*10 + (data_date[0]&0xF);
    dataStr.concat("20");
    itoa(weatherData->year,inChar,10);
    if (weatherData->year<10) dataStr.concat("0");
    dataStr.concat(inChar);
    dataStr.concat("-");
    itoa(weatherData->month,inChar,10);
    if (weatherData->month<10) dataStr.concat("0");
    dataStr.concat(inChar);
    dataStr.concat("-");
    itoa(weatherData->day,inChar,10);
    if (weatherData->day<10) dataStr.concat("0");
    dataStr.concat(inChar);

    //Do chk on data
    //do check on data
    if ((weatherData->year<30) && (weatherData->year>10) && (weatherData->year>=last_year) &&
      (weatherData->month>0) && (weatherData->month<=12) &&
      ((weatherData->month>=last_month) || ((last_month==12) && (weatherData->month==1))) && 
      (weatherData->day>0) && (weatherData->day<=31) &&
      ((weatherData->day>=last_day) || ((last_day==28) && (weatherData->day==1)) ||
      ((last_day==29) && (weatherData->day==1)) || ((last_day==30) && (weatherData->day==1)) ||
      ((last_day==31) && (weatherData->day==1))) ) {
      for (int c=0;c<dataStr.length();c++) {
        weatherData->date[c]=dataStr[c];
      }
      //writemem(dataStr,EEPROM_DATE);
      weatherData->readError = (weatherData->readError | (0x0001<<4));     
      return 1;
    } 
    else {
      return 0;
    } 
    //writemem(dataStr,EEPROM_DATE);

  }   

}

/****************************************************************************
 * Weather station data fetch - tempf
 */
byte getTempf(int item,weather_data_t* weatherData){
  //String dataStr="";  
  char inChar[3];
  char chk_data[7];
  int time=0;
  int command_tempf[] = {
    COMMAND_TEMPF  };
  int data_tempf[LENGTH_TEMPF];  
  int eeprom_addr = EEPROM_TEMPF;
  //float *previousTempValue;
  //previousTempValue = &previousTemp;
  int ftemp_width=0;
  int crc_array[2];

  if (item==1) {
    command_tempf[2] = 0xB2; 
    command_tempf[3] = 0xBA;
    eeprom_addr = EEPROM_DEWPTF;
    //previousTempValue=&previousDewpt;
  }


  for (int i=0;i<2;i++) {  
    while ((read_data(command_tempf,data_tempf)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_tempf);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {    
    return 0;
  }  
  else {

    int tempf = (   (data_tempf[1]>>4)*1000 + (data_tempf[1]&0xF)*100 +
      (data_tempf[0]>>4)*10 + (data_tempf[0]&0xF) - 3000);
    itoa(tempf,chk_data,10);
    float fdata=(atof(chk_data)*0.0180+32.000);
    
    if ((fdata<123.0) && (fdata>-9.4)) {
      if (item==1) {
        weatherData->dewPt = fdata;
      } 
      else {
        weatherData->temp = fdata;
        //Serial.print("WS2308 Reading temp");
      }
      weatherData->readError = (weatherData->readError | (0x0001<<(8+item)));
      return 1;
    } 
    else {
      return 0;
    }    
  }

}

/****************************************************************************
 * Weather station data fetch - humidity
 */
byte getHumidity(weather_data_t* weatherData){

  char inChar[3];
  char chk_data[3];
  int time=0;
  int command_humidity[] = {
    COMMAND_HUMIDITY  };
  int data_humidity[LENGTH_HUMIDITY];
  int ftemp_width=0;  
  int crc_array[2];

  for (int i=0;i<2;i++) {  
    while ((read_data(command_humidity,data_humidity)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_humidity);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {    
    return 0;
  }
  else {

    int idata = ((data_humidity[0]>>4)*10) + (data_humidity[0]&0xF);
    
    if ((idata<100) && (idata>0)) {
      weatherData->humidity = idata;
      weatherData->readError = (weatherData->readError | (0x0001<<10));
      return 1;
    } 
    else {
      return 0;
    }
  }
  
}

/****************************************************************************
 * Weather station data fetch - pressure
 */
byte getBaromhg(weather_data_t* weatherData){

  char inChar[3];
  char chk_data[7];
  int time=0;
  int command_baromhg[] = {
    COMMAND_BAROMHG  };
  int data_baromhg[LENGTH_BAROMHG];  
  int ftemp_width=0;
  int crc_array[2];

  for (int i=0;i<2;i++) {  
    while ((read_data(command_baromhg,data_baromhg)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<6) {
      crc_array[i] = checkcrc(data_baromhg);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {    
    return 0;
  }
  else { 
    //Fill the chk_data array
    int baromhg = (  (data_baromhg[2]&0xF)*10000 + 
      (data_baromhg[1]>>4)*1000 + (data_baromhg[1]&0xF)*100 +
      (data_baromhg[0]>>4)*10 + (data_baromhg[0]&0xF) );               
    itoa(baromhg,chk_data,10);
    float fdata=(atof(chk_data)*0.01);
    if ((fdata<32.06) && (fdata>25.69)) {
      weatherData->barometer=fdata;
      weatherData->readError = (weatherData->readError | (0x0001<<11));
      return 1;
    } 
    else {
      return 0;
    } 
  }
}

/****************************************************************************
 * Weather station data fetch - rain values
 */
byte getRainin(int item, weather_data_t* weatherData){

  char inChar[3];
  char chk_data[8];
  int time=0;
  int command_rainin[] = {
    COMMAND_RAININ  };
  int data_rainin[LENGTH_RAININ];  
  int eeprom_addr = EEPROM_RAININ;
  int ftemp_width=0;
  int crc_array[2];

  if (item==1) {
    command_rainin[2] = 0xA6; 
    command_rainin[3] = 0x9E;
    eeprom_addr = EEPROM_DAILYRAIN;
  }

  for (int i=0;i<2;i++) {  
    while ((read_data(command_rainin,data_rainin)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_rainin);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {    
    return 0;
  }
  else {

    int rainin = (  (data_rainin[2]>>4)*100000 + (data_rainin[2]&0xF)*10000 +
      (data_rainin[1]>>4)*1000 + (data_rainin[1]&0xF)*100 +
      (data_rainin[0]>>4)*10 + (data_rainin[0]&0xF) );
    itoa(rainin,chk_data,10);    
    float fdata=(atof(chk_data)*0.000393700787);
    if ((fdata<100.00) && (fdata>=0.00)) {
      if (item==1) {
        weatherData->rain24hr = fdata;
      } 
      else {
        weatherData->rain1hr = fdata;
      }
      weatherData->readError = (weatherData->readError | (0x0001<<(12+item)));
      return 1;
    } 
    else {
      return 0;
    }   
  }
}

/****************************************************************************
 * Weather station data fetch - wind speed updateAverage, max check
 */
byte getWindSpeed(weather_data_t* weatherData){

  char wind_array[4];
  char speed_array[6];
  int time=0;
  int command_windspeedmph[] = {
    COMMAND_WINDSPEEDMPH  };
  int data_windspeedmph[LENGTH_WINDSPEEDMPH];  
  int ftemp_width=0;
  int crc_array[2];

  //Read the wind data weather twice and check the crc's are the same
  for (int i=0;i<2;i++) {
    //read ws for wind data memory location  
    while ((read_data(command_windspeedmph,data_windspeedmph)!=1)&&(time<5)) {
      delay(500);
      time++;
    }
    if (time<5) {
      crc_array[i] = checkcrc(data_windspeedmph);
      //Serial.println(crc_array[i]);
    }
  }

  if (crc_array[0] != crc_array[1]) {    
    return 0;
  }
  else { 
    //Convert int data to float value mph
    int windSpeed = ((data_windspeedmph[2]&0xF)<<8)+data_windspeedmph[1];
    itoa(windSpeed,speed_array,10);  //int to char array
    float fdata=(atof(speed_array)*0.2237); //char array to float
    //Check for wind sensor spikes
    if (fdata<WIND_SPEED_MAX) {
      //smooth the wind speed data
      weatherData->windSpeed = updateAverage(fdata,weatherData->windSpeed,ALPHA_WIND);     
      //Check if current period max value has been exceeded
      if (fdata > weatherData->windGust) {
        weatherData->windGust = fdata;    
      }  
      weatherData->windDir = ((data_windspeedmph[2]&0xF0)>>4)*22.5;
      weatherData->readError = (weatherData->readError | (0x0001<<14));
      return 1;
    }
    else {
      return 0;
    }
  }
}

/***************************************************************************
 * LOw level read data from Ws2308
 * Assumes on num of data bytes to receive is 3 - can now reduce size of storage buffers
 * Returns 1 if successdully read data
 * Retruns -1 if any check bytes fail
 */
byte read_data(int command[], int placeholder[]) {

  int incomingByte=0;      // a variable to read incoming serial data into
  int time=0;
  int* cmdPtr = command;
  int* dataPtr = placeholder;
  int chk_sum=0;
  int num_bytes=3;
  int byte_no=0;
  // First send reset command and check for response
  while ((incomingByte!=2) && (time<20)){
    time++;
    mySerial.write(byte(06));
    delay(20); 
    if ((mySerial.available() > 0) && (time<20)) {
      delay(30);
      time++;
      incomingByte = mySerial.read();            
    }      
  }

  mySerial.flush();
  if (time>18){
    return 0; //null character - end of string
  }

  //Begin command transmission  
  for (int tmp=0; tmp<5; tmp++){

    mySerial.write(byte(*(cmdPtr+tmp)));
    delay(20); //time to write data to port
    time=0;

    //Read data while while available and in time
    while ((mySerial.available() > 0) && time<200 ) {
      time++;          
      incomingByte = mySerial.read();        
      delay(20); //20

        //Address byte checks
      if ((tmp<4) && (tmp!=((incomingByte & B11110000)>>4))) { //use a shift here
        return 0;
        break;
      }

      //Data bytes receive
      if (tmp==4){
        //Get the number of words being sent - can we sent the same number of bytes to simlpify?
        if ((byte_no==0)&&(incomingByte!=0x33)) {
          return 0;
          break;
        } 
        //Add the data bytes for comparison against checksum
        if ((byte_no>0)&&(byte_no<=num_bytes)) {
          chk_sum += incomingByte;
          //Store the data bytes to the array
          *dataPtr=incomingByte;          
          dataPtr++;
        }
        byte_no++;
      }

    } //End read data

  } //End for loop
  wdt_reset();
  // Checksum - number of bytes
  if ((chk_sum & B11111111) != incomingByte) {
    return 0;
  }
  else {
    return 1; //Successfully read data
  }

}

/****************************************************************************/
String floatToString(float fdata) {

  char fdata_array[7];
  int ftemp_width=0;

  //float fdata = previousWindSpeed;
  if (fdata<1000){
    ftemp_width=5;
  }
  if (fdata<100){
    ftemp_width=4;
  }
  if (fdata<10){
    ftemp_width=3;
  }
  dtostrf(fdata,ftemp_width,2,fdata_array);
  return fdata_array;

}

/****************************************************************************/
String intToString(int idata) {

  char idata_array[6];
  int itemp_width=0;

  //float fdata = previousWindSpeed;
  if (idata<1000){
    itemp_width=5;
  }
  if (idata<100){
    itemp_width=4;
  }
  if (idata<10){
    itemp_width=3;
  }
  dtostrf(idata,itemp_width,1,idata_array);
  itoa(idata,idata_array,10);
  return idata_array;

}

/****************************************************************************/
float updateAverage(float newValue, float oldValue, float alpha)
{
  float updateValue = newValue-oldValue;
  updateValue = oldValue + alpha*updateValue;
  return updateValue;
}

/****************************************************************************/
int checkcrc(int int_array[])
{
  uint8_t crc = 0, i;
  for (i = 0; i < 5; i++)
    crc = _crc_ibutton_update(crc, int_array[i]);
  return crc; // must be 0
}

/****************************************************************************/
void check_mem(int tag) {
  stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
  heapptr = stackptr;                     // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
  Serial << "\r\n" << F("Tag ") << tag << "\t" << F("Heapptr ") << (*heapptr) << "\t" << F("Stackptr ") << (*stackptr) << "\r\n";
}

/****************************************************************************/
int availableMemory() 
{
  int size = 1024;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}

/****************************************************************************/
void print_responses(int block_data[], int length) {
  char inChar[3];
  for (int i=0;i<length;i++){
    sprintf(inChar, "%02X", block_data[i]);    
    PRINTRESP(inChar);
    PRINTRESP(" ");
  }
  PRINTRESPLN("");
}


/****************************************************************************/
void midnightReset()
{
  //When the imp tells us it's midnight, reset the total amount of rain and gusts
  //dailyrainin = 0; //Reset daily amount of rain
  //windgustmph = 0; //Zero out this minute's gust
  //minutes = 0; //Reset minute tracker
  //seconds = 0;
  //lastSecond = millis(); //Reset variable used to track minutes
}

