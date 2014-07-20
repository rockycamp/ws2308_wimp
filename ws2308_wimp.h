
//Structure to hold the data for a weather station

/* WS2308 serial is a bit hit-and-miss so useful to track read errors
 * MSB -> LSB
 * 15         |  14         |  13        |  12
 * windgust   |  windspeed  |  rain24hr  |  rain1hr
 * 11         |  10         |  9         |  8
 * barometer  |  humidity   |  dewpt     |  temp
 * 7          |  6          |  5         |  4
 * reserved   |  reserved   |  reserved  |  Date
 * 3          |  2          |  1         |  0
 * reserved   |  reserved   |  reserved  |  UTC
 */
 
typedef struct {
  char utc[9];
  int hour;
  int minute;
  int second;
  char date[11];
  int year;
  int month;
  int day;
  float temp;
  float dewPt;
  int humidity;
  float barometer;
  float rain1hr;
  float rain24hr;
  float windSpeed;
  float windGust;
  int windDir;
  uint16_t readError;
  byte lastReadErrors;
  byte postErrors;
} weather_data_t;

