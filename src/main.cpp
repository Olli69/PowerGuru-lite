#include <Arduino.h>
#include <math.h> //round
#include <EEPROM.h>

#include <LittleFS.h>

//#include <DNSServer.h> // for captive portal

#ifdef ESP32 // not fully implemented with ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266) // tested only with ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif

//features enabled
#define QUERY_POWERGURU_ENABLED
#define METER_SHELLY3EM_ENABLED
#define SENSOR_DS18B20_ENABLED
#define INVERTER_FRONIUS_SOLARAPI_ENABLED // can read Fronius inverter solarapi
#define INVERTER_SMA_MODBUS_ENABLED  // can read SMA inverter Modbus TCP 
#define RTC_DS3231_ENABLED

#define TARIFF_STATES_FI // add Finnish tariffs (yösähkö,kausisähkö) to active states

#define OTA_UPDATE_ENABLED

#define RTCMEMORYSTART 65
#define eepromaddr 0
#define WATT_EPSILON 0


#define NETTING_PERIOD_MIN 5 // should be 60, later 15

// Powerguru states generated internally
#define STATE_BUYING 1001
#define STATE_SELLING 1002
#define STATE_EXTRA_PRODUCTION 1010
#define STATE_EXTRA_PRODUCTION_BNOON 1011
#define STATE_EXTRA_PRODUCTION_ANOON 1012

#define STATE_DAYENERGY_FI 130
#define STATE_NIGHTENERGY_FI 131
#define STATE_WINTERDAY_FI 140
#define STATE_WINTERDAY_NO_FI 141


#include <ESPAsyncWebServer.h>
/* TODO: and caveats
-error log, file which can be forwarded?
- different error cases, no connection e.g.
- millis exceed long ... - problem???
- backup for no ntp (rtcmem?), eg. boot or short break
- channel up/down could be also stored to rtc, only 1 bit needed...



*/

// https://werner.rothschopf.net/202011_arduino_esp8266_ntp_en.htm
#include <time.h>

time_t now; // this is the epoch
tm tm_struct;


#define MY_NTP_SERVER "europe.pool.ntp.org"
// for timezone https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
//#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"
// TODO: parametriksi jotenkin
#define MY_TZ "EET-2EEST,M3.5.0/3,M10.5.0/4"

// DNSServer dnsServer;
AsyncWebServer server_web(80);

// client
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

#if ARDUINO_ESP8266_MAJOR < 3
#pragma message("This sketch requires at least ESP8266 Core Version 3.0.0")
#endif

// https://werner.rothschopf.net/microcontroller/202112_arduino_esp_ntp_rtc_en.htm

bool rtc_found = false;
#ifdef RTC_DS3231_ENABLED
#include <RTClib.h>
#include <coredecls.h>
#define  I2CSDA_GPIO 0 
#define  I2CSCL_GPIO 12


RTC_DS3231 rtc;
/*
   ESP8266 has no timegm, so we need to create our own...

   Take a broken-down time and convert it to calendar time (seconds since the Epoch 1970)
   Expects the input value to be Coordinated Universal Time (UTC)

   Parameters and values:
   - year  [1970..2038]
   - month [1..12]  ! - start with 1 for January
   - mday  [1..31]
   - hour  [0..23]
   - min   [0..59]
   - sec   [0..59]
   Code based on https://de.wikipedia.org/wiki/Unixzeit example "unixzeit"
*/
int64_t getTimestamp(int year, int mon, int mday, int hour, int min, int sec)
{
  const uint16_t ytd[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}; /* Anzahl der Tage seit Jahresanfang ohne Tage des aktuellen Monats und ohne Schalttag */
  int leapyears = ((year - 1) - 1968) / 4
                  - ((year - 1) - 1900) / 100
                  + ((year - 1) - 1600) / 400; /* Anzahl der Schaltjahre seit 1970 (ohne das evtl. laufende Schaltjahr) */
  int64_t days_since_1970 = (year - 1970) * 365 + leapyears + ytd[mon - 1] + mday - 1;
  if ( (mon > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) )
    days_since_1970 += 1; /* +Schalttag, wenn Jahr Schaltjahr ist */
  return sec + 60 * (min + 60 * (hour + 24 * days_since_1970) );
}
/*
    print time of RTC to Serial
*/
void printRTC()
{
  DateTime dtrtc = rtc.now();          // get date time from RTC i
  if (!dtrtc.isValid())
  {
    Serial.println(F("E103: RTC not valid"));
  }
  else
  {
    time_t newTime = getTimestamp(dtrtc.year(), dtrtc.month(), dtrtc.day(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
    Serial.print(F("RTC:"));
    Serial.print(newTime);
    Serial.print(", temperature:");
    Serial.println(rtc.getTemperature());
  }
}

/*
   set date/time of external RTC
*/
void setRTC()
{
  Serial.println(F("setRTC --> from internal time"));
  time_t now;                          // this are the seconds since Epoch (1970) - seconds GMT
  tm tm;                               // the structure tm holds time information in a more convient way
  time(&now);                          // read the current time and store to now
  gmtime_r(&now, &tm);                 // update the structure tm with the current GMT
  rtc.adjust(DateTime(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec));
}
void time_is_set(bool from_sntp) {
  if (from_sntp)                       // needs Core 3.0.0 or higher!
  {
    Serial.println(F("The internal time is set from SNTP."));
    setRTC();
    printRTC();
  }
  else
  {
    Serial.println(F("The internal time is set."));
  }
}

/*
    Sets the internal time
    epoch (seconds in GMT)
    microseconds
*/
void setInternalTime(uint64_t epoch = 0, uint32_t us = 0)
{
  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = us;
  settimeofday(&tv, NULL);
}


void getRTC()
{
  Serial.println(F("getRTC --> update internal clock"));
  DateTime dtrtc = rtc.now();          // get date time from RTC i
  if (!dtrtc.isValid())
  {
    Serial.print(F("E127: RTC not valid"));
  }
  else
  {
    time_t newTime = getTimestamp(dtrtc.year(), dtrtc.month(), dtrtc.day(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
    setInternalTime(newTime);
    //Serial.print(F("UTC:")); Serial.println(newTime);
    printRTC();
  }
}

#endif

// Non-volatile memory https://github.com/CuriousTech/ESP-HVAC/blob/master/Arduino/eeMem.cpp
#ifdef INVERTER_SMA_MODBUS_ENABLED
#include <ModbusIP_ESP8266.h>
//Modbus registry offsets
#define SMA_DAYENERGY_OFFSET 30535 
#define SMA_TOTALENERGY_OFFSET 30529 
#define SMA_POWER_OFFSET 30775
#endif

#ifdef OTA_UPDATE_ENABLED
unsigned long server_ota_started;
#include <AsyncElegantOTA.h>
AsyncWebServer server_OTA(80);
#endif


#ifdef SENSOR_DS18B20_ENABLED
// see: https://randomnerdtutorials.com/esp8266-ds18b20-temperature-sensor-web-server-with-arduino-ide/
#include <OneWire.h>
#include <DallasTemperature.h> // tätä ei ehkä välttämättä tarvita, jos käyttäisi onewire.h:n rutineeja

#define ONEWIRE_DATA_GPIO 13
#define ONEWIRE_VOLTAGE_GPIO 14

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONEWIRE_DATA_GPIO);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
float ds18B20_temp_c;
#endif


#ifdef QUERY_POWERGURU_ENABLED
const char *pg_state_cache_filename = "/pg_state_cache.json";
#endif

#define USE_POWER_TO_ESTIMATE_ENERGY_SECS 120 //use power measurement to estimate

unsigned process_interval_s = 30; // process interval
unsigned long sensor_last_refresh = -process_interval_s * 1000; // start reading as soon as you get to first loop

time_t recording_period_start = 0;    // first period: boot time, later period starts
time_t current_period_start = 0;
time_t previous_period_start = 0;
time_t energym_read_last =0;
time_t started = 0;
bool period_changed = true;
bool restart_required = false;

// data strcuture limits
#define CHANNELS 2
#define CHANNEL_TARGETS_MAX 3
#define CHANNEL_STATES_MAX 10
#define ACTIVE_STATES_MAX 20

#define MAX_ID_STR_LENGTH 30
#define MAX_URL_STR_LENGTH 70

//Energy metering type
#define ENERGYM_NONE 0
#define ENERGYM_SHELLY3EM 1
#define ENERGYM_FRONIUS_SOLAR 2
#define ENERGYM_SMA_MODBUS_TCP 3
#define ENERGYM_MAX 3

// Type texts for config ui
const char *energym_strings[] PROGMEM = {"none", "Shelly 3EM", "Fronius Solar API", "SMA Modbus TCP"};

#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
// inverter productuction info fields
unsigned long inverter_total_period_init = 0;
unsigned long energy_produced_period = 0;
unsigned long power_produced_period_avg = 0;
#endif


typedef struct
{
  uint16_t upstates[CHANNEL_STATES_MAX];
  float target;
  bool target_active;
} target_struct;

typedef struct
{
  target_struct target[CHANNEL_TARGETS_MAX]; 
  uint8_t gpio;
  bool is_up;
} channel_struct;

// TODO: add fixed ip, subnet?
typedef struct
{
  int check_value;
  bool sta_mode;
  char wifi_ssid[MAX_ID_STR_LENGTH];
  char wifi_password[MAX_ID_STR_LENGTH];
  char http_username[MAX_ID_STR_LENGTH];
  char http_password[MAX_ID_STR_LENGTH];
  channel_struct ch[CHANNELS];
#ifdef QUERY_POWERGURU_ENABLED
  char pg_host[MAX_ID_STR_LENGTH];
  unsigned int pg_port;
  uint16_t pg_cache_age;
#endif
#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
uint32_t base_load_W; // production above base load is "free" to use/store
#endif
#ifdef OTA_UPDATE_ENABLED
  bool next_boot_ota_update;
#endif
  byte energy_meter_type;
  char energy_meter_host[MAX_ID_STR_LENGTH];
  unsigned int energy_meter_port;
  byte energy_meter_id;
  float lat;
  float lon;
} settings_struct;

// this stores settings also to eeprom
settings_struct s;

uint16_t active_states[ACTIVE_STATES_MAX];

//parse char array to uint16_t array (e.g. states, ip address)
//note: current version alter str_in, so use copy in calls if original still needed
void str_to_uint_array(const char *str_in, uint16_t array_out[CHANNEL_STATES_MAX], char *separator)
{
  char *ptr = strtok((char *)str_in, separator);
  byte i = 0;

  for (int ch_state_idx = 0; ch_state_idx < CHANNEL_STATES_MAX; ch_state_idx++)
  {
    array_out[ch_state_idx] = 0;
  }

  while (ptr)
  {
    Serial.print(atol(ptr));
    Serial.print(",");
    array_out[i] = atol(ptr);
    ptr = strtok(NULL, separator);
    i++;
    if (i == CHANNEL_STATES_MAX)
    {
      break;
    }
  } 
  return;
}

// reads sessing from eeprom
void readFromEEPROM()
{
  EEPROM.get(eepromaddr, s);
  Serial.print(F("readFromEEPROM:"));
}

// writes settigns to eeprom
void writeToEEPROM()
{
  // channel
  EEPROM.put(eepromaddr, s); // write data to array in ram
  EEPROM.commit();
  Serial.print(F("writeToEEPROM:"));
}

// from https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/CaptivePortal/CaptivePortal.ino
/*class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

void handleRequest(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>");
    response->print("<p>This is out captive portal front page.</p>");
    response->printf("<p>You were trying to reach: http://%s%s</p>", request->host().c_str(), request->url().c_str());
    response->printf("<p>Try opening  <a href='http://%s'>this link %s</a> instead</p>", WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
    response->print("</body></html>");
    request->send(response);
  }
};
*/


void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

String httpGETRequest(const char *url, const char *cache_file_name)
{
  WiFiClient client;
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(client, url);

  // Serial.println(url);
  // Serial.println(cache_file_name);
  //  Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0)
  {
    payload = http.getString();
    // write to a cache file
    if (strlen(cache_file_name) > 0)
    {
      // Delete existing file, otherwise the configuration is appended to the file
      LittleFS.remove(cache_file_name);

      // Open file for writing
      File cache_file = LittleFS.open(cache_file_name, "w");
      if (!cache_file)
      {
        Serial.println(F("Failed to create file:"));
        Serial.print(cache_file_name);
        Serial.print(", ");
        Serial.println(cache_file);
        return String("");
      }
      int bytesWritten = cache_file.print(http.getString());
      Serial.print(F("Wrote to cache file bytes:"));
      Serial.println(bytesWritten);

      if (bytesWritten > 0)
      { // write failed
        cache_file.close();
      }
    }
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    http.end();
    return String("");
  }
  // Free resources
  http.end();
  return payload;
}

#ifdef SENSOR_DS18B20_ENABLED

//TODO: reset (Voltage low) if value not within range
void read_sensor_ds18B20()
{
  sensors.requestTemperatures();
  ds18B20_temp_c = sensors.getTempCByIndex(0);
}
#endif

#ifdef METER_SHELLY3EM_ENABLED
unsigned last_period = 0;
long last_period_last_ts = 0;
long meter_read_ts = 0;
float energyin_prev = 0;
float energyout_prev = 0;
float energyin = 0;
float energyout = 0;


void get_values_shelly3m( float &netEnergyInPeriod , float &netPowerInPeriod) {
  netEnergyInPeriod = (energyin - energyout - energyin_prev + energyout_prev);
  if ((meter_read_ts - last_period_last_ts) != 0)
  {
    netPowerInPeriod = round(netEnergyInPeriod * 3600.0 / ((meter_read_ts - last_period_last_ts)));
  }
  else
  {
    netPowerInPeriod = 0;
  }
}


bool read_meter_shelly3em()
{
  if (strlen(s.energy_meter_host) == 0)
    return false;
  DynamicJsonDocument doc(2048); 

  String url = "http://" + String(s.energy_meter_host) + ":" + String(s.energy_meter_port) + "/status";
  DeserializationError error = deserializeJson(doc, httpGETRequest(url.c_str(), ""));
  Serial.println(url);

  if (error)
  {
    Serial.print(F("Shelly meter deserializeJson() failed: "));
    Serial.println(error.f_str());
    return false; 
  }

  meter_read_ts = doc["unixtime"];
  unsigned now_period = int(meter_read_ts / (NETTING_PERIOD_MIN * 60));

  if (last_period != now_period and last_period > 0)
  { // new period
    Serial.println(F("Shelly - new period"));
    last_period = now_period; // riittäiskö ..._ts muutt
    // from previous call
    last_period_last_ts = meter_read_ts;
    energyin_prev = energyin;
    energyout_prev = energyout;
  }

  float power_tot = 0;
  Serial.println("  ");
  int idx = 0;
  float power[3];
  energyin = 0;
  energyout = 0;
  for (JsonObject emeter : doc["emeters"].as<JsonArray>())
  {
    power[idx] = (float)emeter["power"];
    power_tot += power[idx];
    // float current = emeter["current"];
    //  is_valid = emeter["is_valid"];
    if (emeter["is_valid"])
    {
      energyin += (float)emeter["total"];
      energyout += (float)emeter["total_returned"];
    }
    idx++;
  }
  // first query since boot
  if (last_period == 0)
  {
    last_period = now_period - 1;
    last_period_last_ts = meter_read_ts - process_interval_s; // estimate
    energyin_prev = energyin;
    energyout_prev = energyout;
  }
  return true;
}
#endif



#ifdef INVERTER_FRONIUS_SOLARAPI_ENABLED

// new version under construction
bool read_inverter_fronius_data(long int &total_energy, long int &current_power)
{
  //  globals updated: inverter_total_period_init
  if (strlen(s.energy_meter_host) == 0)
    return false;

  time(&now);
  StaticJsonDocument<64> filter;

  JsonObject filter_Body_Data = filter["Body"].createNestedObject("Data");
  filter_Body_Data["DAY_ENERGY"] = true; // instead of TOTAL_ENERGY
  filter_Body_Data["PAC"] = true;

  StaticJsonDocument<256> doc;
  String inverter_url = "http://" + String(s.energy_meter_host) + ":" + String(s.energy_meter_port) + "/solar_api/v1/GetInverterRealtimeData.cgi?scope=Device&DeviceId=1&DataCollection=CumulationInverterData";
  Serial.println(inverter_url);

  DeserializationError error = deserializeJson(doc, httpGETRequest(inverter_url.c_str(), ""), DeserializationOption::Filter(filter));

  if (error)
  {
    Serial.print(F("Fronius inverter deserializeJson() failed: "));
    Serial.println(error.f_str());
    energy_produced_period = 0;
    power_produced_period_avg = 0;
    return false;
  }

  for (JsonPair Body_Data_item : doc["Body"]["Data"].as<JsonObject>())
  {

    if (Body_Data_item.key() == "PAC")
    {
      Serial.print(", PAC:");
      Serial.print((long)Body_Data_item.value()["Value"]);
      current_power = Body_Data_item.value()["Value"]; //update and return new value
    }
    // use DAY_ENERGY (more accurate) instead of TOTAL_ENERGY
    if (Body_Data_item.key() == "DAY_ENERGY")
    {
      Serial.print("DAY_ENERGY:");
      total_energy = Body_Data_item.value()["Value"]; //update and return new value    
    }
  }
  Serial.println();

  return true;
} // read_inverter_fronius
#endif

#ifdef INVERTER_SMA_MODBUS_ENABLED

ModbusIP mb; // ModbusIP object
#define REG_COUNT 2
uint16_t buf[REG_COUNT];
uint16_t trans;
// IPAddress remote(84,231,164,210);
// IPAddress remote();

bool cb(Modbus::ResultCode event, uint16_t transactionId, void *data)
{ // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS)
  {
    if (event == Modbus::EX_TIMEOUT)
    {
      Serial.println("EX_TIMEOUT");
    }
    else
    {
      Serial.print("Request result: 0x");
      Serial.println(event, HEX);
    }
    //  mb.disconnect( remote);
  }
  else {
    Serial.println("Modbus read succesfull");
  }

  return true;
}

long int get_mbus_value(IPAddress remote, const int reg_offset, uint16_t reg_num, uint8_t modbusip_unit)
{
  long int combined;
  uint16_t trans = mb.readHreg(remote, reg_offset, buf, reg_num, cb, modbusip_unit);

  while (mb.isTransaction(trans))
  { // Check if transaction is active
    mb.task();
    delay(10);
  }
  
  if (reg_num == 1)
  {
    combined = buf[0];
  }
  else if (reg_num == 2)
  {
    combined = buf[0] * (65536) + buf[1];
  /*  Serial.print("DEBUG get_mbus_value:");
    Serial.print(buf[0]);
    Serial.print("#");
    Serial.println(buf[1]);*/
    if (buf[0] ==32768) { //special case
      combined = 0; 
    }
  }
  else
  {
    combined = 0;
  }
  return combined;
}

bool read_inverter_sma_data(long int &total_energy, long int &current_power)
{
  long int alku = millis();
  // IPAddress remote(); // veikkola.duckdns.org 84.231.164.210
  // IPAddress remote(84,231,164,210);
  // remote.fromString(s.energy_meter_host);
  // tässä voisi olla ip
  uint16_t ip_octets[CHANNEL_STATES_MAX];
  char host_ip[16];
  strcpy(host_ip, s.energy_meter_host);// seuraava kutsu sotkee, siksi siksi kopio
  str_to_uint_array(host_ip, ip_octets, ".");

  IPAddress remote(ip_octets[0], ip_octets[1], ip_octets[2], ip_octets[3]);
  /*
  Serial.print("Parsed IP: ");
  Serial.print(ip_octets[0]);
  Serial.print(".");
  Serial.print(ip_octets[1]);
  Serial.print(".");
  Serial.print(ip_octets[2]);
  Serial.print(".");
  Serial.print(ip_octets[3]);
*/
  uint16_t ip_port = s.energy_meter_port;
  uint8_t modbusip_unit = s.energy_meter_id;

  Serial.print("ip_port, modbusip_unit: " );
  Serial.print(ip_port);
  Serial.println(modbusip_unit);
  

  if (!mb.isConnected(remote))
  {
    Serial.print("Connecting Modbus TCP");
    bool cresult = mb.connect(remote, ip_port);
    Serial.println(cresult);
  }

  if (mb.isConnected(remote))
  { // Check if connection to Modbus Slave is established
    total_energy = get_mbus_value(remote, SMA_TOTALENERGY_OFFSET, 2, modbusip_unit);
    Serial.print(" total energy Wh:");
    Serial.print(total_energy);
    
    current_power = get_mbus_value(remote, SMA_POWER_OFFSET, 2, modbusip_unit);
    Serial.print(", current power W:");
    Serial.println(current_power);
/*
    Serial.print("kesti:");
    Serial.println(millis() - alku);
    Serial.println();*/
    mb.disconnect(remote); // disconect in the end

    return true;
  }
  else
  {
    Serial.println("NOT CONNECTED");
    return false;
  }

} // read_inverter_sma_data
#endif

void read_inverter()
{
  // global: recording_period_start
  // three globals updated: inverter_total_period_init, energy_produced_period, power_produced_period_avg

  long int total_energy = 0;
  long int current_power = 0;

  bool reakOk=false;
  if (s.energy_meter_type==ENERGYM_FRONIUS_SOLAR)
    reakOk = read_inverter_fronius_data(total_energy, current_power);
  else if (s.energy_meter_type==ENERGYM_SMA_MODBUS_TCP)
    reakOk = read_inverter_sma_data(total_energy, current_power);

  if (reakOk)
  {
    time(&energym_read_last);
    
    if (period_changed)
    {
      Serial.println("PERIOD CHANGED");
      inverter_total_period_init = total_energy; // global
      }
    energy_produced_period = total_energy - inverter_total_period_init;
    long int time_since_recording_period_start = now - recording_period_start;

    if (time_since_recording_period_start > USE_POWER_TO_ESTIMATE_ENERGY_SECS) // in the beginning of period use current power to estimate energy generated
      power_produced_period_avg = energy_produced_period * 3600 / time_since_recording_period_start;
    else {
        power_produced_period_avg = current_power;
    }
      

    Serial.printf("energy_produced_period: %ld , time_since_recording_period_start: %ld , power_produced_period_avg: %ld , current_power:  %ld\n", energy_produced_period, time_since_recording_period_start, power_produced_period_avg,current_power);
  }

} // read_inverter


#ifdef QUERY_POWERGURU_ENABLED
bool is_cache_file_valid(const char *cache_file_name, unsigned long max_age_sec)
{
  if (!LittleFS.exists(cache_file_name))
  {
    Serial.println(F("No cache file. "));
    return false;
  }
  File cache_file = LittleFS.open(cache_file_name, "r");
  if (!cache_file)
  { // failed to open the file, retrn empty result
    Serial.println(F("Failed to open cache file. "));
    return false;
  }
  StaticJsonDocument<16> filter;
  filter["ts"] = true; // first get timestamp field

  StaticJsonDocument<50> doc_ts;

  DeserializationError error = deserializeJson(doc_ts, cache_file, DeserializationOption::Filter(filter));
  cache_file.close();

  if (error)
  {
    Serial.print(F("Powerguru server deserializeJson() failed: "));
    Serial.println(error.f_str());
    return false;
  }

  unsigned long ts = doc_ts["ts"];
  time(&now);
 
  unsigned long age = now - ts;

  if (age > max_age_sec)
  {
    return false;
  }
  else
  {
    return true;
  }
}
#endif

// returns next index ie number of elements
byte get_internal_states(uint16_t state_array[CHANNEL_STATES_MAX])
{
  time(&now);
  time_t now_suntime = now + s.lon * 240;
  byte sun_hour = int((now_suntime % (3600 * 24)) / 3600);
  //byte sun_minute = int((now_suntime % (3600)) / 60);
  // clean old
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    state_array[i] = 0;
  }
  // add internally generated states, see https://github.com/Olli69/PowerGuru/blob/main/docs/states.md
  byte idx = 0;
  state_array[idx++] = 1;                // 1 is always on
  state_array[idx++] = 100 + tm_struct.tm_hour; // time/hour based

#ifdef METER_SHELLY3EM_ENABLED
  if (s.energy_meter_type == ENERGYM_SHELLY3EM) {
    state_array[idx++] = (energyin - energyout - energyin_prev + energyout_prev) > 0 ? STATE_BUYING : STATE_SELLING;
    }
#endif

#ifdef TARIFF_STATES_FI
  /*
  130 STATE_DAYENERGY_FI	päiväsähkö, daytime 07-22:, every day
  131 STATE_NIGHTENERGY_FI	yösähkö, 22-07, every day
  140 STATE_WINTERDAY_FI	kausisähkö talvipäivä, Nov 1- Mar 31 07-22, Mon-Sat
  141 STATE_WINTERDAY_NO_FI	kausisähkö, other time
  */

  // päiväsähkö/yösähkö (Finnish day/night tariff)
  if (6 < tm_struct.tm_hour && tm_struct.tm_hour < 22)
  { // day
    state_array[idx++] = STATE_DAYENERGY_FI;
  }
  else
  {
    state_array[idx++] = STATE_NIGHTENERGY_FI;
  }
  // Finnish seasonal tariff, talvipäivä/winter day
  if ((6 < tm_struct.tm_hour && tm_struct.tm_hour < 22) && (tm_struct.tm_mon > 9 || tm_struct.tm_mon < 3) && tm_struct.tm_wday != 0)
  {
    state_array[idx++] = STATE_WINTERDAY_FI;
  }
  else
  {
    state_array[idx++] = STATE_WINTERDAY_NO_FI;
  }
#endif


#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
  // TODO: tsekkaa miksi joskus nousee ylös lyhyeksi aikaa vaikkei pitäisi
  if (power_produced_period_avg > (s.base_load_W + WATT_EPSILON)) { //"extra" energy produced, more than estimated base load 
    state_array[idx++] = STATE_EXTRA_PRODUCTION;
     if (sun_hour<12)
      state_array[idx++] = STATE_EXTRA_PRODUCTION_BNOON;
    else
      state_array[idx++] = STATE_EXTRA_PRODUCTION_ANOON;
  }

#endif
  return idx;
}

void refresh_states(time_t current_period_start)
{
  // get first internal states, then add  more from PG server
  byte idx = get_internal_states(active_states);

#ifndef QUERY_POWERGURU_ENABLED
  return; // fucntionality disabled
#endif
  if (strlen(s.pg_host) == 0)
    return;

  time(&now);
  localtime_r(&now, &tm_struct);

  Serial.print(" refresh_states ");
  Serial.print(F("  current_period_start: "));
  Serial.println(current_period_start);

  StaticJsonDocument<16> filter;
  char start_str[11];
  itoa(current_period_start, start_str, CHANNEL_STATES_MAX);
  filter[(const char *)start_str] = true;

  StaticJsonDocument<200> doc;
  DeserializationError error;

  // TODO: what happens if cache is expired and no connection to state server

  if (is_cache_file_valid(pg_state_cache_filename, s.pg_cache_age))
  {
    Serial.println(F("Using cached data"));
    File cache_file = LittleFS.open(pg_state_cache_filename, "r");
    error = deserializeJson(doc, cache_file, DeserializationOption::Filter(filter));
    cache_file.close();
  }
  else
  {
    Serial.println(F("Cache not valid. Querying..."));
 
    // String url_to_call = String(s.pg_url) + "&states=";
    String url_to_call = "http://" + String(s.pg_host) + ":" + String(s.pg_port) + "&states=";
    String url_states_part = ",";
    char state_str_buffer[8];
    // char *ptr = 0;

    // add only used states to to state query
    for (int channel_idx = 0; channel_idx < CHANNELS; channel_idx++)
      for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++)
      {
        for (int ch_state_idx = 0; ch_state_idx < CHANNEL_STATES_MAX; ch_state_idx++)
        {
          {
            if (s.ch[channel_idx].target[target_idx].upstates[ch_state_idx] == 0)
            {
              break; // no more states in this target
            }
            if (s.ch[channel_idx].target[target_idx].upstates[ch_state_idx] >= 1000)
            {
              snprintf(state_str_buffer, 8, ",%u,", s.ch[channel_idx].target[target_idx].upstates[ch_state_idx]);
              Serial.print(state_str_buffer);
              if (strstr(url_states_part.c_str(), state_str_buffer))
              {
                Serial.println(" already in the url");
              }
              else
              {
                Serial.println("not found in url, adding");
                url_states_part += String(s.ch[channel_idx].target[target_idx].upstates[ch_state_idx]) + ",";
              }
            }
          }
        }
      }
    url_states_part.replace(" ", "");
    url_to_call += url_states_part;
    Serial.println(url_to_call);
    error = deserializeJson(doc, httpGETRequest(url_to_call.c_str(), pg_state_cache_filename), DeserializationOption::Filter(filter));
  }

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  JsonArray state_list = doc[start_str];

  for (unsigned int i = 0; i < state_list.size(); i++)
  {
    active_states[idx++] = (uint16_t)state_list[i];
    if (idx == CHANNEL_STATES_MAX)
      break;
  }
  // DEBUG
  Serial.print("Active states:");
  for (unsigned int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    if (active_states[i] == 0)
      break;
    Serial.print(active_states[i]);
    Serial.print(",");
  }
  Serial.println();
}

// https://github.com/me-no-dev/ESPAsyncWebServer#send-large-webpage-from-progmem-containing-templates
const char setup_form_html[] PROGMEM = R"===(<html>
<head>
<style>
        body {background-color: #11744a3b ;margin:1.6em; font-size:20px;font-family: Verdana, Geneva, Tahoma, sans-serif;}
        p {font-size: 1em;}
        .code {max-width: 250px;}
        .btn {background-color: white;}
      form, #fld, input{
        width:100%%;
      }
      h3 {margin-block-start: 2em;}
      input,select {font-size:2em;}
      input[type=checkbox] {width:50px;height:50px;}
      input[type=submit] {margin-top:30px;}
      .inpnum {text-align: right;}
      h1,h2,h3 {clear:both;}
      .fld {margin-top: 10px;clear:both;}
      .secbr {padding-top: 30px;clear:both;}
      .fldh { float:left; width:44%%; margin-right:2%%; }
      .fldshort {float:left; width:20%%;margin-right:2%%; }   
      .fldtiny {float:left; width:17%%;margin-right:2%%;  }
      .fldlong {float:left; width:70%%;margin-right:2%%; }
    </style>
</head>
<body onload="emtChanged(%emt%);">
<script>
function emtChanged(val) {
  idx = parseInt(val);
 // alert("The input value has changed. The new value is: " + val + "  " + [1,2,3].indexOf(idx));
  var emhd =  document.querySelector('#emhd');
  var empd =  document.querySelector('#empd');
  var emidd =  document.querySelector('#emidd');
  var baseld =  document.querySelector('#baseld');
 
  if ([1,2,3].indexOf(idx) > -1) {
    emhd.style.display = "block";
    empd.style.display = "block";
  }
  else {
    emhd.style.display = "none";
    empd.style.display = "none";
    }
  if ([2,3].indexOf(idx) > -1) 
    baseld.style.display = "block";
  else
    baseld.style.display = "none";

  if ([3].indexOf(idx) > -1) 
    emidd.style.display = "block";
  else
    emidd.style.display = "none";
}
function beforeSubmit() {
 document.querySelector('#ts').value = Date.now()/1000;
}
</script>
<form method="post" onsubmit="beforeSubmit();">
<div class="secbr"><h3>Status</h3></div>
%metered_values%
<div class="fld"><div>Active states: %states%</div></div>
<div class="secbr"><h3>PowerGuru Server</h3></div>
<div class="fld">
<div class="fldh">host:<input name="pg_host" type="text" value="%pg_host%"></div>
<div class="fldtiny">port:<input name="pg_port" type="text" value="%pg_port%"></div>
<div class="fldshort">Max cache age:<input class="inpnum" name="pg_cache_age" type="text" value="%pg_cache_age%"></div>
</div>

<br>
%energy_meter_fields%

<div class="fldshort" id="baseld">base load (W):<input name="base_load_W" class="inpnum" type="text" value="%base_load_W%"></div>


<div class="secbr"><h3>Channel 1</h3></div>
<div class="fld"><div>Current status: %up_ch0%</div></div>
%target_ch_0_0%
%target_ch_0_1%
%target_ch_0_2%
<div class="fldshort">gpio: <input name="gpio_ch0" type="text" value="%gpio_ch0%"></div>
<div class="secbr"><h3>Channel 2</h3></div>
<div class="fld"><div>Current status: %up_ch1%</div></div>
%target_ch_1_0%
%target_ch_1_1%
%target_ch_1_2%
<div class="fldshort">gpio: <input name="gpio_ch1" type="text" value="%gpio_ch1%"></div>

<div class="secbr"><h3>Location</h3></div>
<div class="fld"><div class="fldtiny">lat:<input name="lat" type="text" value="%lat%"></div><div class="fldtiny">lon:<input name="lon" type="text" value="%lon%"></div></div>

<div class="secbr"><h3>WiFi</h3></div>
<div class="fld"><input type="checkbox" id="sta_mode" name="sta_mode" value="sta_mode" %sta_mode%><label for="sta_mode"> Connect to existing wifi network</label></div>
<div class="fld"><div>Wifi SSID</div><div><input name="wifi_ssid" type="text" value="%wifi_ssid%"></div></div>
<div class="fld"><div>Wifi password</div><div><input name="wifi_password"  type="password" value="%wifi_password%"></div></div>
<div class="fld"><h3>Admin access</h3></div>
<div class="fld"><div>Access uid</div><div><input  name="http_username"  type="text" value="%http_username%" readonly></div></div>

<div class="fld"><div>Access password</div><div><input name="http_password" type="password" value="%http_password%"></div></div>

<div class="secbr">
<input type="checkbox" id="ota_next" name="ota_next" value="ota_next"><label for="ota_next">Firmware update.</label>
<input type="checkbox" id="timesync" name="timesync" value="timesync"><label for="timesync">Sync with workstation time.</label></div>
<input type="hidden" id="ts" name="ts">
<br><input type="submit" value="Save">  
</form>
</body></html>)===";

String state_array_string(uint16_t state_array[CHANNEL_STATES_MAX])
{
  String states = String();
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    if (state_array[i] > 0)
    {
      states += String(state_array[i]);
      if (i + 1 < CHANNEL_STATES_MAX && (state_array[i + 1] > 0))
        states += String(",");
    }
    else
      break;
  }
  return states;
}

void get_channel_target_fields(char *out, int channel_idx, int target_idx)
{
  String states = state_array_string(s.ch[channel_idx].target[target_idx].upstates);
  char float_buffer[10];
  dtostrf(s.ch[channel_idx].target[target_idx].target, 3, 1, float_buffer);
  snprintf(out, 400, "<div><div  class=\"fldlong\">#%i target %s<input name=\"st_%i_t%i\" type=\"text\" value=\"%s\"></div></div><div class=\"fldshort\">Target:<input class=\"inpnum\" name=\"t_%i_t%i\" type=\"text\" value=\"%s\"></div></div>", target_idx + 1 ,s.ch[channel_idx].target[target_idx].target_active?"* ACTIVE *":"",channel_idx, target_idx, states.c_str(), channel_idx, target_idx, float_buffer);
  return;
  
}

void get_meter_config_fields(char *out)
{
  char buff[150];
  strcpy(out, "<div class='secbr'><h3>Energy meter</h3></div><div class=\"fld\"><select name=\"emt\" id=\"emt\" onchange=\"emtChanged(this.value)\">");

  for (int energym_idx = 0; energym_idx <= ENERGYM_MAX; energym_idx++)
  {
    snprintf(buff, 150, "<option value=\"%d\" %s>%s</>", energym_idx, (s.energy_meter_type == energym_idx) ? "selected" : "", energym_strings[energym_idx]);
    strcat(out, buff);
  }
  strcat(out, "</select></div>");
  snprintf(buff, 150, "<div id='emhd' class=\"fld\"><div class=\"fldh\">host:<input name=\"emh\" id=\"emh\" type=\"text\" value=\"%s\"></div>", s.energy_meter_host);
  strcat(out, buff);
  snprintf(buff, 150, "<div id='empd' class=\"fldtiny\">port:<input name=\"emp\" id=\"emp\" type=\"text\" value=\"%d\"></div>", s.energy_meter_port);
  strcat(out, buff);
  snprintf(buff, 150, "<div id='emidd' class=\"fldtiny\">unit:<input name=\"emid\" id=\"emid\" type=\"text\" value=\"%d\"></div></div>", s.energy_meter_id);
  strcat(out, buff);
  // Serial.println(out);
  return;
}

void get_metered_values(char *out)
{
  char buff[150];
  time_t current_time;
  time(&current_time);
  localtime_r(&current_time, &tm_struct);

  time_t now_suntime = current_time + (s.lon * 240);
  tm tm_sun;
  char time1[9];
  char time2[9];
  char eupdate[20];


  
#ifdef SENSOR_DS18B20_ENABLED
  snprintf(buff, 150, "<div class=\"fld\"><div>Temperature: %s</div></div>",String(ds18B20_temp_c,2).c_str());
  strcat(out, buff);
#else
            return F("not in use");
#endif

  gmtime_r(&now_suntime, &tm_sun);
  snprintf(buff, 150, "<div class=\"fld\"><div>Local time: %02d:%02d:%02d, solar time: %02d:%02d:%02d</div></div>"
  ,tm_struct.tm_hour,tm_struct.tm_min,tm_struct.tm_sec
  ,tm_sun.tm_hour,tm_sun.tm_min,tm_sun.tm_sec);
  strcat(out, buff);


  localtime_r(&recording_period_start, &tm_struct);
  sprintf(time1, "%02d:%02d:%02d",tm_struct.tm_hour,tm_struct.tm_min,tm_struct.tm_sec);
  localtime_r(&energym_read_last, &tm_struct);
  
  if (energym_read_last==0) {
    strcpy(time2,"");
    strcpy(eupdate,", not updated");
  }
  else {
     sprintf(time2, "%02d:%02d:%02d",tm_struct.tm_hour,tm_struct.tm_min,tm_struct.tm_sec);
      strcpy(eupdate,"");
  }

  if (s.energy_meter_type == ENERGYM_SHELLY3EM)
  {
#ifdef METER_SHELLY3EM_ENABLED
  float netEnergyInPeriod;
  float netPowerInPeriod;
  get_values_shelly3m(netEnergyInPeriod, netPowerInPeriod);
  snprintf(buff, 150, "<div class=\"fld\"><div>Period %s-%s: net energy in %d Wh, power in  %d W %s</div></div>"
  ,time1,time2,(int)netEnergyInPeriod, (int)netPowerInPeriod,eupdate);
  strcat(out, buff);
#endif
  }
  else if (s.energy_meter_type == ENERGYM_FRONIUS_SOLAR or (s.energy_meter_type == ENERGYM_SMA_MODBUS_TCP ))
  {
#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
  snprintf(buff, 150, "<div class=\"fld\"><div>Period %s-%s: produced %d Wh, power  %d W %s</div></div>",time1,time2,(int)energy_produced_period, (int)power_produced_period_avg,eupdate);
  strcat(out, buff);
#endif
  }


  return;
}


String setup_form_processor(const String &var)
{
  if (var == "sta_mode")
    return s.sta_mode ? "checked" : "";
  if (var == "wifi_ssid")
    return s.wifi_ssid;
  if (var == "wifi_password")
    return s.wifi_password;
  if (var == "http_username")
    return F("powerguru");
  if (var == "http_password")
    return s.http_password;

  if (var == "emt")
    return String(s.energy_meter_type);

  if (var == "energy_meter_fields")
  {
    char out[600];
    get_meter_config_fields(out);
    return out;
  }

  if (var == "base_load_W")
#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
    return String(s.base_load_W);
#else
    return F("(disabled)")
#endif


  if (var.startsWith("target_ch_"))
  {
    // e.g target_ch_0_1
    char out[400];
    int channel_idx = var.substring(10, 11).toInt();
    int target_idx = var.substring(12, 13).toInt();
    get_channel_target_fields(out, channel_idx, target_idx);
    return out;
  }
  if (var == "states")
  {
    return state_array_string(active_states);
  }

  if (var == "metered_values") {
    char out[400];
    get_metered_values(out);
    return out;
  }

  if (var == "lat") {
    return String(s.lat,2);
  }
  if (var == "lon") {
    return String(s.lon,2);
  }

 

#ifdef QUERY_POWERGURU_ENABLED
  /*if (var == "pg_url")
    return s.pg_url;*/
  if (var == "pg_host")
    return s.pg_host;
  if (var == "pg_port")
    return String(s.pg_port);
  if (var == "pg_cache_age")
    return String(s.pg_cache_age);

#endif

  // Serial.println("Channel processor");

  for (int i = 0; i < CHANNELS; i++)
  {
    if (var.equals(String("gpio_ch") + i))
    {
      return String(s.ch[i].gpio);
    }

    if (var.equals(String("up_ch") + i))
    {
      Serial.print("###");
      Serial.println(var);
      return String(s.ch[i].is_up ? "up" : "down");
    }
  }
  return String();
}

// ...

void read_energy_meter()
{
  if (s.energy_meter_type == ENERGYM_SHELLY3EM)
  {
#ifdef METER_SHELLY3EM_ENABLED
    bool readOk = read_meter_shelly3em();
    if (readOk)
      time(&energym_read_last);
#endif
  }
  else if (s.energy_meter_type == ENERGYM_FRONIUS_SOLAR or (s.energy_meter_type == ENERGYM_SMA_MODBUS_TCP ))
  {
#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
    read_inverter();
#endif
  }
}

void set_relays()
{
  int active_state_count = 0;
  bool target_state_match_found;
  bool channel_should_be_up;

  // how many current active states we do have
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    if (active_states[i] > 0)
      active_state_count++;
    else
      break;
  }

  // loop channels and check whether channel should be up
  for (int channel_idx = 0; channel_idx < CHANNELS; channel_idx++)
  { //reset target_active variable
    for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++) {
      s.ch[channel_idx].target[target_idx].target_active = false; 
    }
    target_state_match_found = false;

    channel_should_be_up = false;
    // loop channel targets until there is match (or no more targets)
    for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++)
    {
      // check matching states, i.e. if any of target states matches current active states
      for (int act_state_idx = 0; act_state_idx < active_state_count; act_state_idx++)
      {
        for (int ch_state_idx = 0; ch_state_idx < CHANNEL_STATES_MAX; ch_state_idx++)
        {
          if (active_states[act_state_idx] == s.ch[channel_idx].target[target_idx].upstates[ch_state_idx])
          { 
            
#ifdef SENSOR_DS18B20_ENABLED
                // TODO: check that sensor value is valid
                //  states are matching, check if the sensor value is below given target (channel should be up) or reached (should be down)
                if ((ds18B20_temp_c < s.ch[channel_idx].target[target_idx].target))
            {
              channel_should_be_up = true;
               s.ch[channel_idx].target[target_idx].target_active = true;    
            }
#else
                channel_should_be_up = target_state_match_found;
                s.ch[channel_idx].target[target_idx].target_active = target_state_match_found;  
#endif
            if (target_state_match_found)
              break;
          }
        }
        if (target_state_match_found)
          break;
      }  
      if (target_state_match_found)
        break;
    } //target loop


    if (s.ch[channel_idx].is_up != channel_should_be_up)
    {
      Serial.println();
      Serial.print("channel:");
      Serial.println(channel_idx);
      Serial.print("new relay value:");
      s.ch[channel_idx].is_up = channel_should_be_up;
      Serial.print(channel_should_be_up);
      Serial.print("Setting gpio :");
      Serial.print(s.ch[channel_idx].gpio);
      Serial.print(" ");
      Serial.println((s.ch[channel_idx].is_up ? "HIGH" : "LOW"));
      digitalWrite(s.ch[channel_idx].gpio, (s.ch[channel_idx].is_up ? HIGH : LOW));
    }
    /*
    else
    {
      Serial.print("channel ");
      Serial.print(channel_idx);
      Serial.print(": ");
      Serial.println(s.ch[channel_idx].is_up);
    } */
  } // channel loop
}
// Web response functions
void onWebRootGet(AsyncWebServerRequest *request)
{
  if (!request->authenticate(s.http_username, s.http_password))
    return request->requestAuthentication();
  String message;
  // large char array, tested with 14k
  request->send_P(200, "text/html", setup_form_html, setup_form_processor);
}

void onWebResetGet(AsyncWebServerRequest *request)
{
  Serial.println("Resetting");
  request->send(200, "text/plain", "Resetting... Reload after a few seconds.");
  delay(1000);
  // write a char(255) / hex(FF) from startByte until endByte into the EEPROM

  for (unsigned int i = eepromaddr; i < eepromaddr + sizeof(s); ++i)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  strcpy(s.http_username, "");
  s.sta_mode = false;
  s.ch[0].gpio = 255; // not the best way
  s.check_value = 12345;

  writeToEEPROM();
  delay(1000);
  ESP.restart();
}

void onWebRootPost(AsyncWebServerRequest *request)
{
  String message;
  
  int paramsNr = request->params();
  Serial.println(paramsNr);

  s.sta_mode = request->hasParam("sta_mode", true);
  Serial.print("s.sta_mode:");
  Serial.print(s.sta_mode);

  strcpy(s.wifi_ssid, request->getParam("wifi_ssid", true)->value().c_str());
  strcpy(s.wifi_password, request->getParam("wifi_password", true)->value().c_str());
  strcpy(s.http_username, request->getParam("http_username", true)->value().c_str());
  strcpy(s.http_password, request->getParam("http_password", true)->value().c_str());

  if (s.energy_meter_type != request->getParam("emt", true)->value().toInt()) {
    restart_required= true;
    s.energy_meter_type = request->getParam("emt", true)->value().toInt();
  }
    

  Serial.println(request->getParam("emh", true)->value().c_str());
  strcpy(s.energy_meter_host, request->getParam("emh", true)->value().c_str());
  Serial.println(s.energy_meter_host);
  s.energy_meter_port = request->getParam("emp", true)->value().toInt();
  s.energy_meter_id = request->getParam("emid", true)->value().toInt();

  s.lat = request->getParam("lat", true)->value().toFloat();
  s.lon = request->getParam("lon", true)->value().toFloat();

#ifdef INVERTER_FRONIUS_SOLARAPI_ENABLED
  // strcpy(s.fronius_address, request->getParam("fronius_address", true)->value().c_str());
  s.base_load_W = request->getParam("base_load_W", true)->value().toInt();

#endif

#ifdef QUERY_POWERGURU_ENABLED
  // strcpy(s.pg_url, request->getParam("pg_url", true)->value().c_str());
  strcpy(s.pg_host, request->getParam("pg_host", true)->value().c_str());
  s.pg_port = request->getParam("pg_port", true)->value().toInt();
  s.pg_cache_age = request->getParam("pg_cache_age", true)->value().toInt();
#endif

  // channel/target fields
  char gpio_fld[9];
  char state_fld[8];
  char target_fld[7];
  for (int channel_idx = 0; channel_idx < CHANNELS; channel_idx++)
  {
    snprintf(gpio_fld, 9, "gpio_ch%i", channel_idx);
    s.ch[channel_idx].gpio = request->getParam(gpio_fld, true)->value().toInt();

    for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++)
    {
      snprintf(state_fld, 8, "st_%i_t%i", channel_idx, target_idx);
      snprintf(target_fld, 7, "t_%i_t%i", channel_idx, target_idx);
      if (request->hasParam(state_fld, true))
      {
        str_to_uint_array(request->getParam(state_fld, true)->value().c_str(), s.ch[channel_idx].target[target_idx].upstates, ",");
        s.ch[channel_idx].target[target_idx].target = request->getParam(target_fld, true)->value().toFloat();
      }
    }
  }
 
  
  if (request->hasParam("timesync", true)) {
    time_t ts = request->getParam("ts", true)->value().toInt();
    Serial.print("TIMESTAMP:");
    Serial.print(ts);
    setInternalTime(ts);

    if (rtc_found)
      setRTC();
  }

#ifdef OTA_UPDATE_ENABLED
  if (request->hasParam("ota_next", true))
  {
    s.next_boot_ota_update = true;
    writeToEEPROM();
    //request->redirect("/update");
    request->send(200, "text/html", "<html><head><meta http-equiv='refresh' content='5; url=./update' /></head><body>wait...</body></html>"); 
  }

#endif
  // save to non-volatile memory
  writeToEEPROM();

  // delete cache file
  LittleFS.remove(pg_state_cache_filename);
  request->redirect("/");


}

void onWebStatusGet(AsyncWebServerRequest *request)
{

  if (!request->authenticate(s.http_username, s.http_password))
  {
    return request->requestAuthentication();
  }
  StaticJsonDocument<250> doc; // oli 128, lisätty heapille ja invertterille
  String output;
  JsonObject variables = doc.createNestedObject("variables");

#ifdef METER_SHELLY3EM_ENABLED
  float netEnergyInPeriod;
  float netPowerInPeriod;
  get_values_shelly3m(netEnergyInPeriod, netPowerInPeriod);
  variables["netEnergyInPeriod"] = netEnergyInPeriod;
  variables["netPowerInPeriod"] = netPowerInPeriod;
#endif

#ifdef INVERTER_FRONIUS_SOLARAPI_ENABLED
  variables["energyProducedPeriod"] = energy_produced_period;
  variables["powerProducedPeriodAvg"] = power_produced_period_avg;
#endif

  variables["updated"] = meter_read_ts;
  variables["freeHeap"] = ESP.getFreeHeap();
  variables["uptime"] = (unsigned long)(millis() / 1000);
  // TODO: näistä puuttu nyt sisäiset, pitäisikö lisätä vai poistaa kokonaan, onko tarvetta debugille
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    // doc["states"][i] = active_states[i];
    if (active_states[i] > 0)
      doc["states"][i] = active_states[i];
    else
      break;
  }

  serializeJson(doc, output);
  request->send(200, "application/json", output);
}


void setup()
{
  Serial.begin(115200);


#ifdef SENSOR_DS18B20_ENABLED

// voltage to 1-wire bus
// voltage from data pin so we can reset the bus (voltage low) if needed
  pinMode(ONEWIRE_VOLTAGE_GPIO, OUTPUT);
  digitalWrite(ONEWIRE_VOLTAGE_GPIO, HIGH);

  sensors.begin();

#endif
#ifdef QUERY_POWERGURU_ENABLED
  // TODO: pitäisikö olla jo kevyempi
  while (!LittleFS.begin())
  {
    Serial.println(F("Failed to initialize LittleFS library"));
    delay(1000);
  }
  Serial.println(F("LittleFS initialized"));
#endif
  Serial.println(F("setup() starting"));

  EEPROM.begin(sizeof(s));
  readFromEEPROM();
  Serial.print("AFTER READ EEPROM:");
  Serial.println(s.energy_meter_host);

  if (s.check_value != 12345) 
  {
    Serial.println(F("Initiating eeprom"));
    s.check_value = 12345; // this is indication that eeprom is initiated
    strcpy(s.wifi_ssid, "");
    strcpy(s.wifi_password, "powerguru");
    strcpy(s.http_username, "powerguru");
    strcpy(s.http_password, "powerguru");
 

  s.ch[0].gpio = 5;s.ch[1].gpio = 4;
  s.lat = 64.96;
  s.lon = 27.59;
  for (int channel_idx = 0; channel_idx < CHANNELS; channel_idx++)
    for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++)
    {
      s.ch[channel_idx].target[target_idx] = {{}, 0};
    }

#ifdef QUERY_POWERGURU_ENABLED
  strcpy(s.pg_host, "");
  s.pg_port = 80;
  s.pg_cache_age = 7200;
#endif
#if defined(INVERTER_FRONIUS_SOLARAPI_ENABLED) || defined(INVERTER_SMA_MODBUS_ENABLED)
s.base_load_W =  0; 
#endif
#ifdef OTA_UPDATE_ENABLED
  s.next_boot_ota_update = false;
#endif
  s.energy_meter_type = 0;
  strcpy(s.energy_meter_host, "");
  s.energy_meter_port = 80;
  s.energy_meter_id = 3;

  writeToEEPROM();
  }

  for (int i = 0; i < CHANNELS; i++)
  {
    Serial.print(F("Setting gpio "));
    Serial.print(s.ch[i].gpio);
    pinMode(s.ch[i].gpio, OUTPUT);
    // if up/down up-to-date stored to non-volatile:
    // digitalWrite(s.ch[i].gpio, (s.ch[i].is_up ? HIGH : LOW));
    //  use defaults
    digitalWrite(s.ch[i].gpio, (s.ch[i].is_up ? HIGH : LOW));
    Serial.println((s.ch[i].is_up ? "HIGH" : "LOW"));
  }
  /*
    if (1 == 2) //Softap should be created if cannot connect to wifi (like in init), redirect
    { // check also https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/CaptivePortal/CaptivePortal.ino
      if (WiFi.softAP("powerguru-lite", "powerguru", 1, false, 1) == true)
      {
        Serial.println(F("WiFi AP created!"));
      }
    }*/
  bool create_ap = !s.sta_mode;

  if (s.sta_mode)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(s.wifi_ssid, s.wifi_password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      Serial.println(F("WiFi Failed!"));
      create_ap = true; // try to create AP instead
    }
    else
    {
      Serial.print(F("IP Address: "));
      Serial.println(WiFi.localIP());
      WiFi.setAutoReconnect(true);
      WiFi.persistent(true);
    }
  }


  if (create_ap) // Softap should be created if so defined, cannot connect to wifi , redirect
  {              // check also https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/CaptivePortal/CaptivePortal.ino
    String mac = WiFi.macAddress();
    for (int i = 14; i > 0; i -= 3)
    {
      mac.remove(i, 1);
    }
    String APSSID = String("powerguru-") + mac;

    if (WiFi.softAP(APSSID.c_str(), "powerguru", (int)random(1, 14), false, 1) == true)
    {
      Serial.println(F("WiFi AP created with ip"));
      Serial.println(WiFi.softAPIP().toString());
      // dnsServer.start(53, "*", WiFi.softAPIP());

      // server_web.on(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER,)
    }
    else
    {
      delay(5000); // cannot create AP, restart
      ESP.restart();
    }
  }
#ifdef RTC_DS3231_ENABLED
  Serial.println("Starting RTC!");
  Wire.begin(I2CSDA_GPIO, I2CSCL_GPIO);
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
  }
    else {
      rtc_found = true;
      Serial.println("RTC found");
      Serial.flush();
      settimeofday_cb(time_is_set);                  // register callback if time was sent
      if (time(nullptr) < 1600000000) getRTC();      // Fallback to RTC on startup if we are before 2020-09-13
    }

#endif

#ifdef OTA_UPDATE_ENABLED
  // wait for update
  if (s.next_boot_ota_update)
  {
    // TODO: password protection
    s.next_boot_ota_update = false; // next boot is normal
    writeToEEPROM();


    server_OTA.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(200, "text/html", "<html><body><h2>Update mode</h2><a href=\"/update\">update</a> | <a href=\"/restart\">restart</a></body></html>"); });

    server_OTA.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
                  {  request->send(200, "text/html", "<html><head><meta http-equiv='refresh' content='10; url=./' /></head><body></body></html>"); 
                    ESP.restart(); });

    AsyncElegantOTA.begin(&server_OTA,s.http_username,s.http_password); // Start ElegantOTA
    server_ota_started = millis();
    server_OTA.begin(); // tähän joku timeout
    while (true)
    {
      delay(1000); // just wait here, until uploaded or restarted manually
    }
  }
#endif

  // TODO: prepare for no internet connection? -> channel defaults probably, RTC?
  // https://werner.rothschopf.net/202011_arduino_esp8266_ntp_en.htm
  RTC_DS3231_ENABLED(MY_TZ, MY_NTP_SERVER); // --> Here is the IMPORTANT ONE LINER needed in your sketch!

  server_web.on("/reset", HTTP_GET, onWebResetGet);
  server_web.on("/status", HTTP_GET, onWebStatusGet);
  server_web.on("/", HTTP_GET, onWebRootGet);
  server_web.on("/", HTTP_POST, onWebRootPost);
  server_web.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
                  {  request->redirect("/"); });//actually called from 
                    

 

  /* if (create_ap) {
     server_web.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
   }
   */

  server_web.onNotFound(notFound);

  server_web.begin();

  Serial.print(F("setup() finished:"));
  Serial.println(ESP.getFreeHeap());

#ifdef QUERY_POWERGURU_ENABLED
  // powerguru_last_refresh = -powerguru_interval_s * 1000;
  //  strcpy(s.pg_url, "http://192.168.66.8:8080/state_series?price_area=FI");
#endif

  // first period is not full, so start calculations from now

  // TODO: if not ntp server: could we get old ts from eeprom (rtcmem has more cycles), or how about using date from  powerguru/shelly query (if used)
  //  ehkä tästä saisi jotain, vaikkei olekaan viimeisen päälle, rtcmem olisi paras paikka kun ei kulum kai, https://stackoverflow.com/questions/54458116/how-can-i-set-the-esp32s-clock-without-access-to-the-internet
 /* do
  {
    time(&started);
    delay(1000);
    Serial.print("*");
  } while (started < 1600000000);*/
 

} // end of setup()

long get_period_start_time(long ts)
{
  return long(ts / (NETTING_PERIOD_MIN * 60UL)) * (NETTING_PERIOD_MIN * 60UL);
}

void loop()
{
  // Serial.print(F("Starting loop"));
  /*if (!s.sta_mode) {
    dnsServer.processNextRequest();
  }*/
#ifdef OTA_UPDATE_ENABLED
  // resetting and rebooting in update more
  if (s.next_boot_ota_update || restart_required) {
    delay(1000);
    ESP.restart();
  }
#endif

  time(&now);
  if (now < 1600000000) // we need clock set
    return;
  if (started <1600000000)
    started = now;

  current_period_start = get_period_start_time(now); // long(now / (NETTING_PERIOD_MIN * 60UL)) * (NETTING_PERIOD_MIN * 60UL);
  if (get_period_start_time(now) == get_period_start_time(started))
    recording_period_start = started;
  else
    recording_period_start = current_period_start;

  if (previous_period_start != current_period_start)
    period_changed = true; // more readable

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(1000);
    return;
  }

  // TODO: all sensor /meter reads could be here?, do we need diffrent frequencies?
  if (((millis() - sensor_last_refresh) > process_interval_s * 1000) || period_changed)
  {
    Serial.print(F("Reading sensor and meter data..."));
#ifdef SENSOR_DS18B20_ENABLED
    read_sensor_ds18B20();
#endif

    read_energy_meter();
    refresh_states(current_period_start);
    sensor_last_refresh = millis();
    set_relays(); // tässä voisi katsoa onko tarvetta mennä tähän eli onko tullut muutosta
  }
  if (period_changed)
  {
    previous_period_start = current_period_start;
    period_changed = false;
  }

#ifdef INVERTER_SMA_MODBUS_ENABLED
  mb.task(); // process modbuss event queue
#endif

  // delay(5000);
}
