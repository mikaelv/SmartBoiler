#include <LiquidCrystal.h>
//#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t
#include "EEPROM.h"


// Features
//#define LOG_TO_SD 1
#define WIFI_ENABLED
#define LCD_ENABLED 1
//#define DHT11_ENABLED 1
#define DS18B20_ENABLED 1
//#define TMP36_ENABLED 1

#ifdef LOG_TO_SD
#include <SD.h>
#endif

#include <Time.h>
#include <TimeAlarms.h>
#include <SoftwareSerial.h>

// ID of Hardware Serial port for pprintf related functions
const uint8_t SERIAL0 = 255;

// Size of the buffers 
#define CMDBUFSZ 64      // used to read user commands from sesend AT commands
#define BUFSZ_PPRINTF 64 // buffer used to write things with pprintf
#define BUFSZ_SPF 64     // static buffer used in spf (sprintf with F macro)


#define SSID "VM819361-2G"
#define SSPWD "rfrrwexy"

SoftwareSerial wifiSerial(2, 3); // RX, TX

#ifdef LCD_ENABLED
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
#endif
#ifdef DHT11_ENABLED
#include "dht11.h"
dht11 DHT11;
const int DHT11PIN = 4;
#endif

#ifdef DS18B20_ENABLED
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS A1
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif


#define RELAY_HEATING_PIN A2
#define RELAY_WATER_PIN A3
#ifdef TMP36_ENABLED
#define TEMP_PIN  A1
#endif
//#define VREF_PIN  A5 // connect to 3.3v for compensating voltage dips when the relay is on, see http://www.reddit.com/r/arduino/comments/1zcr4l/trouble_with_reading_sensor_values_and_use_relay/
const byte NBSAMPLES = 20;
#define CHIPSEL_PIN 10 // SD Card Chip select

#define KEY_RIGHT 0
#define KEY_UP 1
#define KEY_DOWN 2
#define KEY_LEFT 3
#define KEY_SELECT 4


// File management
char FileName[] = "datalog.txt";
boolean isLoggingData = false;
int linesWritten = 0;


// Index: AlarmId, value: temperature in degree C
byte alarmTemperatures[dtNBR_ALARMS];
byte desiredTemperature = 20;
float currentTemperature = 0;







#ifdef LOG_TO_SD
/** Log to SD card every minute */
void logToSD(float temp) {
  static unsigned long lastWriteTime = 0;
  unsigned long currentTime = millis();
  if (isLoggingData && (currentTime - lastWriteTime) >= 60000) {
    // Light up led and write one line in the log file
    digitalWrite(13, HIGH);
    File dataFile = SD.open(FileName, FILE_WRITE);
    dataFile.print(getDateTimeSec());
    dataFile.print('\t');
    dataFile.print((long)RTC.get);
    dataFile.print('\t');
    dataFile.println(temp);
    dataFile.close();
    digitalWrite(13, LOW);
    lastWriteTime = currentTime;
    linesWritten++;
    Serial.print(F("."));
  }
}
#endif

/** Wait for a OK response on the Wifi Serial 
 * @return 1 if OK, 0 if NOK
*/
boolean wifiWaitOK(int waitDuration) {
  char buf[CMDBUFSZ];
  buf[0] = 0;
  
  boolean foundOK = false;
  int totalWaited = -1;

  int i = 0;
 
  // Wait until the buffer ends with OK 
  while (!foundOK && totalWaited < waitDuration) {
    // Read one line
    while (wifiSerial.available()) {
      char c = wifiSerial.read();
      buf[i++] = c;
      buf[i] = 0;
      Serial.write(c); // debug
      if (c == '\n'  || i >= CMDBUFSZ-1)
        i = 0;
    }
    foundOK = ((strcmp(buf, "OK\r\n") == 0) || (strcmp(buf, "no change\r\n") == 0));
    if (!foundOK) {
      delay(100);
      totalWaited += 100;
    }
  }
  
  return foundOK;

}

/** 
 * Sends an AT command
 * @param waitDuration how much time do we wait before getting OK
 * @return true if the command succeeds (OK)
 */
boolean sendCmdToWifiModule(const __FlashStringHelper *cmd, int waitDuration, boolean debug) { 
  wifiSerial.print(F("AT+"));
  wifiSerial.print(cmd);
  wifiSerial.print(F("\r\n"));

  boolean foundOK = wifiWaitOK(waitDuration);
  if (!foundOK) {
    Serial.print(F("Error when sending command to ESP8266: "));
    Serial.println(cmd);
  }
  return foundOK;
}


/** Print something to the remote terminal through Wifi.
  * Everything is also printed to Serial0 for debugging
  */
void wifiPrint(byte port, char* data, boolean newline) {
  Serial.print(F("AT+CIPSEND=0,"));
  wifiSerial.print(F("AT+CIPSEND=0,")); // TODO use port number
  
  int len = strlen(data);
  if (newline) 
    len +=1;
  Serial.print(len);
  Serial.print(F("\r\n"));
  wifiSerial.print(len);
  wifiSerial.print(F("\r\n"));
  
  wifiWaitOK(0);
  
  int i = 0;
  while (data[i] != 0) {
    Serial.write(data[i]);
    wifiSerial.write(data[i]);
    i++;
  }
  
  if (newline) {
    Serial.write('\n');
    wifiSerial.print('\n');
  }
  
  wifiWaitOK(0);
}



/** Static sprintf that can use the F() macro to save RAM 
  * !! take care of not using a first result after a second call to this method
  */
char* spf(const __FlashStringHelper *fmt, ...) {
  static char buf[BUFSZ_SPF];
  va_list args;
  va_start (args, fmt);
  #ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char*) fmt, args);
  #else
  vsnprintf(buf, sizeof(buf), (const char*) fmt, args); // for the rest of the world
  #endif
  va_end(args);
  
  return buf;
}

static void portprintf(byte port, const char *fmt, boolean newline, va_list args) {
  char buf[BUFSZ_PPRINTF]; // resulting string limited to 64 chars

  #ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), fmt, args); // progmem for AVR
  #else
  vsnprintf(buf, sizeof(buf), fmt, args); // for the rest of the world
  #endif
  
  if (port == SERIAL0) {
    Serial.print(buf);
    if (newline) Serial.print('\n');
  }
  else 
    wifiPrint(port, buf, newline);
}


void initWifi() {
  wifiSerial.begin(9600);
  wifiSerial.print(F("ATE0\r\n")); // Disable echo

  //sendCmdToWifiModule("RST", 10000, false);
  if (sendCmdToWifiModule(F("CWJAP=\""SSID"\",\""SSPWD"\""), 10000, false)) {
    Serial.print(F("Connected to "));
    Serial.println(F(SSID));
    if (sendCmdToWifiModule(F("CIPMUX=1"), 0, false)) {
      if (sendCmdToWifiModule(F("CIPSERVER=1,80"), 0, false)) {
        Serial.println(F("Listening to port 80"));
        sendCmdToWifiModule(F("CIFSR"), 1000, true);
        // Does not work: sendCmdToWifiModule(F("CIPSTAMAC?"), 1000, true);
      }
    }
  }
}






void pprintfln(byte port, const __FlashStringHelper *fmt, ...) {
  va_list args;
  va_start (args, fmt);
  portprintf(port, (const char *)fmt, true, args);
  va_end(args);
}

void pprintf(byte port, const __FlashStringHelper *fmt, ...) {
  va_list args;
  va_start (args, fmt);
  portprintf(port, (const char *)fmt, false, args);
  va_end(args);
}



int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void printFreeRam(int port) {
  pprintfln(port, F("Free RAM : %d"), freeRam());
}
  

/*
void onSendDataToTCP0() {
  char *arg = SCmd.next();
  if (arg != NULL) {
    pprintf(0, F("%s\r\n"), arg);
  }
}*/


/** return a formatted date/time with seconds */
char* getDateTimeSec() {
  byte d = day();
  byte m = month();
  int y = year();
  byte hh = hour();
  byte mm = minute();
  byte ss = second();
  return spf(F("%02d/%02d/%02d %02d:%02d:%02d"), d, m, y, hh, mm, ss);
}

char* getDateTime(time_t time) {
  byte d = day(time);
  byte m = month(time);
  int y = year(time);
  byte hh = hour(time);
  byte mm = minute(time);
  return spf(F("%02d/%02d/%02d %02d:%02d"), d, m, y, hh, mm);
}

#ifdef DS18B20_ENABLED
void readTemperatureDS18B20() {
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = millis();
  if ((currentTime - lastReadTime) >= 5000) {
    lastReadTime = currentTime;
    sensors.requestTemperatures();
    currentTemperature = sensors.getTempCByIndex(0);
  }
}
#endif



#ifdef TMP36_ENABLED
/** Takes the mean of NBSAMPLES readings to have more stability */
void readTemperatureTMP36() {
  static int values[NBSAMPLES];
  static byte firstPos = 0;
  static byte count = 0;
  static int total = 0;
  static int average = 0;

  delay(100);
  int val = analogRead(TEMP_PIN);  
  if (abs(average - val) > 100 && count == NBSAMPLES) {
    pprintfln(SERIAL0, F("Ignoring value: %d, average: %d"), val, average);
    return; // ignore crazy values once we have enough samples to have a good average
  }

  total += val;

  //pprintf(SERIAL0, F("tot: %d, cnt: %d, val: %d, %d=get[%d], "), total, count, val, values[firstPos], firstPos);
  
  if (count == NBSAMPLES) {
    // The buffer is full, remove the first element from the total
    total -= values[firstPos]; // TODO it looks like the total does not reflect the content of the buffer
    firstPos = (firstPos + 1) % NBSAMPLES;
  }
  else
    count++;

  // Rotates the buffer: when we hit the count, we restart at 0
  byte lastPos = (firstPos + count -1) % NBSAMPLES;
  values[lastPos] = val; 
  pprintfln(SERIAL0, F("set[%d]=%d"), lastPos, val);
  
  
  // Calculates the new temperature
  average = total / count;
  
  // Keyes KY-013
  /*long a = 1023 - average;
  long beta = 4090;
  float tempC = beta /(log((1025.0 * 10 / a - 10) / 10) + beta / 298.0) - 273.0;*/
  /*float temp;
  temp = log(((10240000/average) - 10000));
  temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp );
  temp = temp - 273.15;// Convert Kelvin to Celcius */

  // TMP36
  float voltage = (average / 1024.0) * 1.1;
  float temp = (voltage - 0.5) * 100;  
  currentTemperature = temp; 
}
#endif




void onGetTemp(byte port) {
  pprintfln(port, F("%s\t%d"), getDateTimeSec(), (int)(currentTemperature*100));
}

#ifdef DHT11_ENABLED
void onGetHumTemp(byte port) {
  pprintfln(port, F("%s\t%d\t%d"), getDateTimeSec(), (int)(DHT11.temperature*100), (int)(DHT11.humidity*100));
}
#endif



void printAlarm(byte port, byte id) {
  time_t alarm = Alarm.readNextTrigger(id);
  byte dayOfWeek = dayOfWeek(alarm); // doesn't print when called directly !??
  pprintfln(port, F("Program %d: %d %02d:%02d %d, next trigger: %s"), 
    id, 
    dayOfWeek, hour(alarm), minute(alarm), alarmTemperatures[id], 
    getDateTime(alarm));
}

void onAlarm() {
  AlarmId id = Alarm.getTriggeredAlarmId();
  pprintf(SERIAL0, F("Triggered "));
  printAlarm(SERIAL0, id);
  desiredTemperature = alarmTemperatures[id];
}



void onListAlarms(byte port) {
  pprintfln(port, F("Programs:"));
  for (byte i = 0; i < dtNBR_ALARMS; i++) {
    if (Alarm.isAllocated(i)) {
      printAlarm(port, i);
    }
  }  
}

/** Save permanently to EEPROM */
void onSaveAlarms(byte port) {
  byte alarmCount = 0;
  for (byte id = 0; id < dtNBR_ALARMS; id++) {
    if (Alarm.isAllocated(id)) {
      alarmCount++;
      time_t alarm = Alarm.readNextTrigger(id);
      byte dayOfWeek = dayOfWeek(alarm);
      byte hh = hour(alarm);
      byte mm = minute(alarm);
      byte temp = alarmTemperatures[id];
      EEPROM.write(id*4, dayOfWeek); 
      EEPROM.write(id*4+1, hh);
      EEPROM.write(id*4+2, mm);
      EEPROM.write(id*4+3, temp);
    } else {
      EEPROM.write(id*4, 0); // 0 means unallocated
    }
  }
  pprintfln(port, F("Saved %d programs to EEPROM"), alarmCount);
}

void onCreateAlarm(byte port, byte dayOfWeek, byte hh, byte mm, byte temperature) {
  AlarmId id = Alarm.alarmRepeat((timeDayOfWeek_t)dayOfWeek, hh, mm, 0, onAlarm);
  if (id < dtNBR_ALARMS) {
    alarmTemperatures[id] = temperature;
    printAlarm(port, id);
  } else {
    // TODO check 255 dtINV,mALID_ALARM_ID
    pprintfln(port, F("Error: id > dtNBR_ALARMS"));
  }
}


void onLoadAlarms(byte port) {
  for (byte id = 0; id < dtNBR_ALARMS; id++) {
    byte dayOfWeek = EEPROM.read(id*4);
    if (dayOfWeek != 0) { // This is a valid alarm
      byte hh = EEPROM.read(id*4+1);
      byte mm = EEPROM.read(id*4+2);
      byte temp = EEPROM.read(id*4+3);
      Alarm.free(id);
      onCreateAlarm(port, dayOfWeek, hh, mm, temp);
    }
  }  
}






// Show available commands
void showCommands(byte port) {
  pprintfln(port, F("Available commands"));
  pprintfln(port, F(" ?                            - This command list"));
  pprintfln(port, F(" t                            - Get current temperature"));
  pprintfln(port, F(" p dow hh mm temp             - Program weekly temperature"));
  pprintfln(port, F(" l                            - List all programs"));
  pprintfln(port, F(" d id                         - Delete a program"));
  pprintfln(port, F(" i                            - Load programs from EEPROM"));
  pprintfln(port, F(" s                            - Save programs to EEPROM"));
  pprintfln(port, F(" n temp                       - Set temperature now"));
  pprintfln(port, F(" datetime YYYY MM DD HH mm ss - Set date/time"));
  pprintfln(port, F(" log1                         - Start logging to file"));
  pprintfln(port, F(" log0                         - Stop logging to file"));
  pprintfln(port, F(" logp                         - Print logged data"));
  pprintfln(port, F(" logd                         - Delete logged data"));
  //pprintfln(port, F(" w AT????????XXXX             - Send a command to ESP8266"));
  //pprintfln(port, F(" w0 ????                      - Send data to TCP connection 0"));
  printFreeRam(port);
}


/** Called when we receive a command over wifi or serial */
void onCommand(byte port, char *cmd) {
  switch (cmd[0]) {
    case 't': onGetTemp(port); break;
    #ifdef DHT11_ENABLED
    case 'h': onGetHumTemp(port); break;
    #endif
    
    case 'p':
      byte dow, hh, mm, temp;
      if (sscanf(cmd, "p %d %d:%d %d", &dow, &hh, &mm, &temp))
        onCreateAlarm(port, dow, hh, mm, temp);
      break;
      
    case 'l': onListAlarms(port); break;
    case 's': onSaveAlarms(port); break;
    case 'i': onLoadAlarms(port); break;
    case 'd':
      byte id;
      if (sscanf(cmd, "d %d", &id)) {
        Alarm.free(id);
        onListAlarms(port); 
      }
      break;
      
    case 'n': 
      if (sscanf(cmd, "n %d", &desiredTemperature))
        pprintfln(port, F("Temperature set to %d"), desiredTemperature);
      break;
      
    default: showCommands(port);
  }
}

/** Detect commands sent over Wifi */
void readWifiCommand() {
  static char buf[CMDBUFSZ];
  static int  bufPos = 0;
  static int nbOfWifiCharsToRead = 0;
  static boolean isReadingWifiData = false;


  while (wifiSerial.available()) {
    char c = wifiSerial.read();
    buf[bufPos++] = c;
    buf[bufPos] = 0;
    Serial.print(c); // debug
    
    if (isReadingWifiData) {
      nbOfWifiCharsToRead--;
      
      if (nbOfWifiCharsToRead <= 0 || bufPos >= CMDBUFSZ-1) {
        isReadingWifiData = false;
        // TODO call a function which scans the command
        Serial.print(F("Received wifi command:["));
        Serial.print(buf);
        Serial.print(']');
        onCommand(0, buf);
      }
    } else if (c == ':' || c == '\n' || c == '\r' || bufPos >= CMDBUFSZ-1) {
      bufPos = 0;
      if (sscanf(buf, "+IPD,0,%d", &nbOfWifiCharsToRead)) {
        isReadingWifiData = true;
      }  // else data is not important
    }    
  }
}

/** Detect commands sent over serial */
void readSerialCommand() {
  static char buf[CMDBUFSZ];
  static int  bufPos = 0;

  while (Serial.available()) {
    char c = Serial.read();
    buf[bufPos++] = c;
    buf[bufPos] = 0;
    
    if (c == '\n' || c == '\r' || bufPos >= CMDBUFSZ-1) {
      bufPos = 0;
      onCommand(SERIAL0, buf);
    }    
  }
}

  







// Called when a received command has no attached function
void onUnknownCommand()
{
  Serial.println(F("This command is unknown!"));
  showCommands(SERIAL0);
}

// Callback function that shows a list of commands
void onCommandList() {
  showCommands(SERIAL0);
}

/*
void onSendWifiCommand() {
  char *arg = SCmd.next();
  if (arg != NULL) {
    wifiSerial.print(arg);
    wifiSerial.print("\r\n");
  }
}

*/





/** Takes YYYY, MM, DD, HH, MM, SS and sets the RTC */
/*void onSetDateTime() {
  rtc.adjust(DateTime(cmdMessenger.readInt16Arg(), cmdMessenger.readInt16Arg(), cmdMessenger.readInt16Arg(),
             cmdMessenger.readInt16Arg(), cmdMessenger.readInt16Arg(), cmdMessenger.readInt16Arg()));
  Serial.println(getDateTimeSec());
}*/

#ifdef LOG_TO_SD
void onStartWriteLog() {
  if (isLoggingData) {
    Serial.println(F("File is already open"));
  } else {
    isLoggingData = true;
    linesWritten = 0;
    Serial.print(F("Writing to "));
    Serial.println(FileName);

  }
}

void onStopWriteLog() {
  if (isLoggingData) {
      isLoggingData = false;
      Serial.print(F("Wrote "));
      Serial.print(linesWritten);
      Serial.println(F(" lines"));
  } else {
    Serial.println(F("File is not open"));
  }

}

void onDeleteLog() {
  if (isLoggingData) {
    Serial.println(F("File is already open"));
  } else {
    SD.remove(FileName);
    Serial.print(FileName);
    Serial.println(F(" deleted"));
  }
}

void onReadLog() {
  File dataFile = SD.open(FileName, FILE_READ);
  
  // TODO if (dataFile) ??
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }

  dataFile.close();
}


void initSDCard() {
  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(CHIPSEL_PIN)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));
}

#endif

#ifdef DHT11_ENABLED
/** Read the DHT11 sensor. We can only read every 2 seconds */
void readDht11() {
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = millis();
  if ((currentTime - lastReadTime) >= 2000) {
    lastReadTime = currentTime;
    int chk = DHT11.read(DHT11PIN);
    if (chk != DHTLIB_OK)
      pprintfln(SERIAL0, F("DHT11 checksum: %d"), chk);
  }
}
#endif


/** Get the current pressed key */
byte getKey(unsigned int input) {
  static const int adc_key_val[5] = {50, 200, 400, 600, 800 };
  
  int k;
  for (k = 0; k < 5; k++) {
    if (input < adc_key_val[k])
      return k;
  }   
  if (k >= 5) 
    k = -1;  // No valid key pressed
  return k;
}

byte getKeyPress() {
  static byte oldKey = -1;
  
  byte key = getKey(analogRead(A0));
  if (key != oldKey) {  // if keypress is detected
    delay(50); // wait for debounce time
    key = getKey(analogRead(A0));
    if (key != oldKey) {
       oldKey = key;
       if (key >= 0)
         return key;
    }
  }
  return -1;
}



void setup() {
  Serial.begin(9600);
  printFreeRam(SERIAL0);
  //Wire.begin();
  
  #ifdef LOG_TO_SD
  initSDCard();
  #endif
  
  #ifdef WIFI_ENABLED
  initWifi();
  #endif
  pinMode(13, OUTPUT);
  pinMode(RELAY_HEATING_PIN, OUTPUT);
  pinMode(RELAY_WATER_PIN, OUTPUT);
  
  //analogReference(EXTERNAL); // Analog ref is 3.3v
  //analogReference(INTERNAL); // Analog ref is 1.1v => Max temp is (1.1 - 0.5) * 100 = 60 °

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  #ifdef LCD_ENABLED
  lcd.begin(16, 2);
  #endif

  showCommands(SERIAL0);
  onLoadAlarms(SERIAL0);
  
  #ifdef LOG_TO_SD
  onStartWriteLog();
  #endif
  
  #ifdef DS18B20_ENABLED
  sensors.begin();
  #endif
  
  
  // TODO read alarms from file or eeprom
}





void loop() {
  delay(20);

  #ifdef DS18B20_ENABLED
  readTemperatureDS18B20();
  #elif TMP36_ENABLED
  readTemperatureTMP36();
  #elif DHT11_ENABLED
  readDht11();
  #endif

  // Read buttons
  byte keyPress = getKeyPress();
  if (keyPress != -1) {
    if (keyPress == KEY_UP)
      desiredTemperature++;
    else if (keyPress == KEY_DOWN)
      desiredTemperature--;
  }


  #ifdef LCD_ENABLED
  // Display stuff on LCD
  lcd.setCursor(0,0);
  time_t alarm = Alarm.getNextTrigger();
  lcd.print(spf(F("%02d:%02d      %02d:%02d"), hour(), minute(), hour(alarm), minute(alarm)));
  lcd.setCursor(0,1);
  lcd.print(currentTemperature);
  lcd.print("\xDF "); // ° 7E: ->
  lcd.print((int)desiredTemperature);
  lcd.print("\xDF "); // ° ->
  byte nextAlarmId = Alarm.getNextAlarm();
  if (nextAlarmId != -1) {
    lcd.print(alarmTemperatures[nextAlarmId]);
    lcd.print((char)0xDF);
  }
  
  #endif
  
  // Read commands
  #ifdef WIFI_ENABLED
  readWifiCommand();
  #endif
  readSerialCommand();


  // service alarms
  Alarm.delay(0);

  
  // Manage temperature
  if (currentTemperature < desiredTemperature - 0.5) {
    digitalWrite(RELAY_HEATING_PIN, LOW);
  } else if (currentTemperature > desiredTemperature + 0.5) {
    digitalWrite(RELAY_HEATING_PIN, HIGH);
  }
  
  #ifdef LOG_TO_SD
  logToSD(currentTemperature);
  #endif

}




