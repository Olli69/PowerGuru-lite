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

#define QUERY_POWERGURU
#define METER_SHELLY3EM
#define SENSOR_DS18B20
#define INVERTER_FRONIUS_SOLARAPI
#define TARIFF_STATES_FI // add Finnish tariffs (yösähkö,kausisähkö) to active states

#define RTCMEMORYSTART 65
#define eepromaddr 0
#define WATT_EPSILON 50

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
tm tm;

#define MY_NTP_SERVER "europe.pool.ntp.org"
// for timezone https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
//#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"
// TODO: parametriksi jotenkin
#define MY_TZ "EET-2EEST,M3.5.0/3,M10.5.0/4"

// DNSServer dnsServer;
AsyncWebServer server(80);

// client
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
// # include <Arduino_JSON.h>
#include <ArduinoJson.h>

// Non-volatile memory https://github.com/CuriousTech/ESP-HVAC/blob/master/Arduino/eeMem.cpp

#ifdef SENSOR_DS18B20
// see: https://randomnerdtutorials.com/esp8266-ds18b20-temperature-sensor-web-server-with-arduino-ide/
#include <OneWire.h>
#include <DallasTemperature.h> // tätä ei ehkä välttämättä tarvita, jos käyttäisi onewire.h:n rutineeja
// lukurutiini, mieti gpio , milloin pull-up
// GPIO where the DS18B20 is connected to,
const int oneWireBus = 4; // oli 4 MiniD2, D3 GPI0 has internal up up 10kR
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

float ds18B20_temp_c;
#endif

#ifdef QUERY_POWERGURU
// char pg_url[] = "http://192.168.66.8:8080/state_series?price_area=FI";
const char *pg_state_cache_filename = "/pg_state_cache.json";

// unsigned powerguru_interval_s = 30;
// unsigned long powerguru_last_refresh = -powerguru_interval_s * 1000; //
#endif

#ifdef METER_SHELLY3EM
// const char *shelly_url = "http://192.168.66.40/status";
unsigned shelly3em_interval_s = 30;
// unsigned long shelly3em_last = -shelly3em_interval_s * 1000;
#endif

// all read operations at once
unsigned sensor_read_interval_s = 30;
unsigned long sensor_last_refresh = -sensor_read_interval_s * 1000; // start reading as soon as you get to first loop

//  the following variables are unsigned longs because the time, measured in
//  milliseconds, will quickly become a bigger number than can be stored in an int.
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)

const unsigned long netting_period_min = 60; // should be 60
unsigned long recording_period_start = 0;    // first period: boot time, later period starts
unsigned long current_period_start = 0;
unsigned long previous_period_start = 0;
time_t started = 0;
bool period_changed = false;

#define CHANNELS 2
#define CHANNEL_TARGETS_MAX 3
#define CHANNEL_STATES_MAX 10
#define ACTIVE_STATES_MAX 20

#define MAX_ID_STR_LENGTH 30
#define MAX_URL_STR_LENGTH 70
// nämä flashiin, voisiko olla array of unsigned int 0-65k

// uusi tapa, under construction
typedef struct
{
  uint16_t upstates[CHANNEL_STATES_MAX];
  float target;
} target_struct;

typedef struct
{
  uint16_t upstates_ch[CHANNEL_STATES_MAX]; // tämä pois
  float target_b_ch;                        // tämä pois
  float target_u_ch;                        // tämä pois
  bool is_up;
  bool default_up;
  uint8_t gpio;
  target_struct target[CHANNEL_TARGETS_MAX]; // new way
} channel_struct;

// TODO: add fixed ip, subnet?
typedef struct
{
  bool sta_mode;
  char wifi_ssid[MAX_ID_STR_LENGTH];
  char wifi_password[MAX_ID_STR_LENGTH];
  char http_username[MAX_ID_STR_LENGTH];
  char http_password[MAX_ID_STR_LENGTH];
  channel_struct ch[CHANNELS];
#ifdef QUERY_POWERGURU
  char pg_url[MAX_URL_STR_LENGTH];
  uint16_t pg_cache_age;
  // uint16_t pg_refresh_interval;
#endif
#ifdef METER_SHELLY3EM
  char shelly_url[MAX_URL_STR_LENGTH];
#endif
#ifdef INVERTER_FRONIUS_SOLARAPI
  char fronius_address[MAX_URL_STR_LENGTH];
  uint32_t base_load_W; // production above base load is "free" to use/store
#endif
} settings_struct;

settings_struct s;

// channel_struct ch[CHANNELS];
uint16_t active_states[ACTIVE_STATES_MAX];

void str_to_uint_array(const char *str_in, uint16_t array_out[CHANNEL_STATES_MAX])
{
  char *ptr = strtok((char *)str_in, ",");
  byte i = 0;

  for (int j = 0; j < CHANNEL_STATES_MAX; j++)
  {
    array_out[j] = 0;
  }

  while (ptr)
  {
    Serial.print(atol(ptr));
    Serial.print(",");
    array_out[i] = atol(ptr);
    ptr = strtok(NULL, ",");
    i++;
    if (i == CHANNEL_STATES_MAX)
    {
      break;
    }
  }
  return;
}

void readFromEEPROM()
{
  // channel
  EEPROM.get(eepromaddr, s);
  Serial.print(F("readFromEEPROM:"));
  // Serial.println(CHANNELS * sizeof(s.ch));
  // yield();
}

void writeToEEPROM()
{
  // channel
  EEPROM.put(eepromaddr, s); // write data to array in ram
  EEPROM.commit();
  Serial.print(F("writeToEEPROM:"));
  // Serial.println(CHANNELS * sizeof(s.ch));
  //  yield();
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

#ifdef SENSOR_DS18B20
void read_sensor_ds18B20()
{
  sensors.requestTemperatures();
  ds18B20_temp_c = sensors.getTempCByIndex(0);
}
#endif

#ifdef METER_SHELLY3EM
unsigned last_period = 0;
long last_period_last_ts = 0;
long now_ts = 0;
float energyin_prev = 0;
float energyout_prev = 0;
float energyin = 0;
float energyout = 0;

void read_meter_shelly3em()
{
  if (strlen(s.shelly_url) == 0)
    return;
  // Serial.print(F("Starting to read Shelly"));
  DynamicJsonDocument doc(2048); // oli 1536

  // sensorReadings = ;

  DeserializationError error = deserializeJson(doc, httpGETRequest(s.shelly_url, ""));
  if (error)
  {
    Serial.print(F("Shelly meter deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  now_ts = doc["unixtime"];
  unsigned now_period = int(now_ts / (netting_period_min * 60));

  if (last_period != now_period and last_period > 0)
  { // new period
    Serial.println(F("Shelly - new period"));
    last_period = now_period; // riittäiskö ..._ts muutt
    // from previous call
    last_period_last_ts = now_ts;
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
    last_period_last_ts = now_ts - shelly3em_interval_s; // estimate
    energyin_prev = energyin;
    energyout_prev = energyout;
  }
}
#endif

#ifdef INVERTER_FRONIUS_SOLARAPI

long unsigned int inverter_total_period_init = 0;
long unsigned int energy_produced_period = 0;
long unsigned int power_produced_period_avg = 0;

void read_inverter_fronius()
{
  if (strlen(s.fronius_address) == 0)
    return;

  time(&now);
  StaticJsonDocument<64> filter;

  JsonObject filter_Body_Data = filter["Body"].createNestedObject("Data");
  filter_Body_Data["DAY_ENERGY"] = true; // instead of TOTAL_ENERGY
  filter_Body_Data["PAC"] = true;

  StaticJsonDocument<256> doc;
  String inverter_url = String(s.fronius_address) + "/solar_api/v1/GetInverterRealtimeData.cgi?scope=Device&DeviceId=1&DataCollection=CumulationInverterData";
  // Serial.println(inverter_url);

  DeserializationError error = deserializeJson(doc, httpGETRequest(inverter_url.c_str(), ""), DeserializationOption::Filter(filter));

  if (error)
  {
    Serial.print(F("Fronius inverter deserializeJson() failed: "));
    Serial.println(error.f_str());
    energy_produced_period = 0;
    power_produced_period_avg = 0;
    return;
  }

  long int total_energy = 0;
  long int current_power = 0;
  for (JsonPair Body_Data_item : doc["Body"]["Data"].as<JsonObject>())
  {

    if (Body_Data_item.key() == "PAC")
    {
      Serial.print(", PAC:");
      Serial.print((long)Body_Data_item.value()["Value"]);
      current_power = Body_Data_item.value()["Value"];
    }
    // use DAY_ENERGY (more accurate) instead of TOTAL_ENERGY
    if (Body_Data_item.key() == "DAY_ENERGY")
    {
      Serial.print("DAY_ENERGY:");
      total_energy = Body_Data_item.value()["Value"];
      if (period_changed)
      {
        Serial.println("PERIOD CHANGED");
        inverter_total_period_init = total_energy;
      }
      Serial.print((long)Body_Data_item.value()["Value"]);
    }
  }
  Serial.println();

  energy_produced_period = total_energy - inverter_total_period_init;

  long int time_since_recording_period_start = now - recording_period_start;
  /*
    Serial.print("now:");
    Serial.println(now);
    Serial.print("recording_period_start:");
    Serial.println(recording_period_start);
    */

  if (time_since_recording_period_start > 60) // in the beginning of period use current power, 60 is  an estimate
    power_produced_period_avg = energy_produced_period * 3600 / time_since_recording_period_start;
  else
    power_produced_period_avg = current_power;

  Serial.printf("energy_produced_period: %ld , time_since_recording_period_start: %ld , power_produced_period_avg: %ld \n", energy_produced_period, time_since_recording_period_start, power_produced_period_avg);
}
#endif

#ifdef QUERY_POWERGURU
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
  // unsigned long age = timeClient.getEpochTime() - ts;
  unsigned long age = now - ts;
  /*
    Serial.print("ts:");
    Serial.print(ts);
    Serial.print(", age:");
    Serial.println(age);
  */
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
  // clean old
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    state_array[i] = 0;
  }
  // add internally generated states, see https://github.com/Olli69/PowerGuru/blob/main/docs/states.md
  byte idx = 0;
  state_array[idx++] = 1;                // 1 is always on
  state_array[idx++] = 100 + tm.tm_hour; // time/hour based

#ifdef METER_SHELLY3EM
  state_array[idx++] = (energyin - energyout - energyin_prev + energyout_prev) > 0 ? 1001 : 1002;
#endif

#ifdef TARIFF_STATES_FI
  /*
  130 RFU	päiväsähkö, daytime 07-22:, every day
  131 RFU	yösähkö, 22-07, every day
  140 RFU	kausisähkö talvipäivä, Nov 1- Mar 31 07-22, Mon-Sat
  141 RFU	kausisähkö, other time
  */

  // päiväsähkö/yösähkö (Finnish day/night tariff)
  if (6 < tm.tm_hour && tm.tm_hour < 22)
  { // day
    state_array[idx++] = 130;
  }
  else
  {
    state_array[idx++] = 131;
  }
  // Finnish seasonal tariff, talvipäivä/winter day
  if ((6 < tm.tm_hour && tm.tm_hour < 22) && (tm.tm_mon > 9 || tm.tm_mon < 3) && tm.tm_wday != 0)
  {
    state_array[idx++] = 140;
  }
  else
  {
    state_array[idx++] = 141;
  }
#endif

#ifdef INVERTER_FRONIUS_SOLARAPI
  if (power_produced_period_avg > (s.base_load_W + WATT_EPSILON)) //"extra" energy produced, more than estimated base load
    state_array[idx++] = 1003;
#endif
  return idx;
}

void refresh_states(unsigned long current_period_start)
{
  // get first internal states, then add  more from PG server
  byte idx = get_internal_states(active_states);

#ifndef QUERY_POWERGURU
  return; // fucntionality disabled
#endif
  if (strlen(s.pg_url) == 0)
    return;

  time(&now);
  localtime_r(&now, &tm);
  Serial.print(" refresh_states ");
  // Serial.print(timeClient.getFormattedTime());
  Serial.print(F("  current_period_start: "));
  Serial.println(current_period_start);

  StaticJsonDocument<16> filter;
  char start_str[11];
  itoa(current_period_start, start_str, CHANNEL_STATES_MAX);
  filter[(const char *)start_str] = true;

  StaticJsonDocument<200> doc;
  DeserializationError error;

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
    // Stream& input;
    // TODO: save to a file and use filter with the cached json

    // StaticJsonDocument<400> doc;
    String url_to_call = String(s.pg_url) + "&states=";

    for (int i = 0; i < CHANNELS; i++)
      for (int j = 0; j < CHANNEL_STATES_MAX; j++)
      {
        {
          if (s.ch[i].upstates_ch[j] == 0)
          {
            break;
          }
          url_to_call += String(s.ch[i].upstates_ch[j]) + ",";
        }
      }
    url_to_call.replace(" ", "");
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
      h3 {margin-block-start: 3em;}
      input {font-size:2em;}
      input[type=checkbox] {width:50px;height:50px;}
      input[type=submit] {margin-top:30px;}
      .fld {margin-top: 10px;clear:both;}
      .fldh { width:45%%;margin-right:3%%; }
      .fldshort {float:left; width:20%%;margin-right:3%%; }
      .fldlong {float:left; width:70%%;margin-right:3%%; }
    </style>
</head><body>
<form method="post">

<div class="fld">Wifi</div>
<div class="fld"><input type="checkbox" id="sta_mode" name="sta_mode" value="sta_mode" %sta_mode%><label for="sta_mode"> Connect to existing wifi network</label></div>

<div class="fld"><div>Wifi SSID</div><div><input name="wifi_ssid" type="text" value="%wifi_ssid%"></div></div>
<div class="fld"><div>Wifi password</div><div><input name="wifi_password"  type="text" value="%wifi_password%"></div></div>
<div class="fld"><div>Access uid</div><div><input  name="http_username"  type="text" value="%http_username%" readonly></div></div>

<div class="fld"><div>Access password</div><div><input name="http_password" type="text" value="%http_password%"></div></div>
<div class="fld"><div>Shelly energy meter url</div><div><input name="shelly_url" type="text" value="%shelly_url%"></div></div>
<div class="fldlong"><div>Fronius address</div><div><input name="fronius_address" type="text" value="%fronius_address%"></div></div>
<div class="fldshort">Base load (W): <input name="base_load_W" type="text" value="%base_load_W%"></div>



<div class="fldlong"><div>Powerguru server url</div><div><input name="pg_url" type="text" value="%pg_url%"></div></div>
<div class="fldshort">Max cache age (s): <input name="pg_cache_age" type="text" value="%pg_cache_age%"></div>
<br>
<h2>Status</h2>
<div class="fld"><div>Current sensor value: %sensorv0%</div></div>
<div class="fld"><div>Active states: %states%</div></div>
<p>Refresh to update.</p>
<h2>uusi</h2>
<h3>Channel 1</h3>
%target_ch_0_0%
%target_ch_0_1%
%target_ch_0_2%
<h3>Channel 2</h3>
%target_ch_1_0%
%target_ch_1_1%
%target_ch_1_2%
<h2>Channels</h2>
<h3>Channel 1</h3>
<div class="fld"><div>Current status: %up_ch0%</div></div>
<div class="fldlong"><div>Up on states</div><div><input name="states_ch0" type="text" value="%upstates_ch0%"></div></div>
<div class="fldshort">Target: <input name="t_u_ch0" type="text" value="%t_u_ch0%"></div>

<div class="fld">Channel options
<div class="fldshort">Base target: <input name="t_b_ch0" type="text" value="%t_b_ch0%"></div>
<div class="fldshort"><label for="du_ch0"> default up:</label><br><input type="checkbox"  name="du_ch0" value="du_ch0" %du_ch0%></div>
<div class="fldshort">gpio: <input name="gpio_ch0" type="text" value="%gpio_ch0%"></div>
</div>

<h3>Channel 2</h3>
<div class="fld"><div>Current status: %up_ch1%</div></div>
<div class="fldlong"><div>Up on states</div><div><input name="states_ch1" type="text" value="%upstates_ch1%"></div></div>
<div class="fldshort">Target: <input name="t_u_ch1" type="text" value="%t_u_ch1%"></div>


<div class="fld">Channel options
<div class="fldshort">Base target: <input name="t_b_ch1" type="text" value="%t_b_ch1%"></div>
<div class="fldshort"><label for="du_ch1"> default up:</label><br><input type="checkbox"  name="du_ch1" value="du_ch1" %du_ch1% ></div>
<div class="fldshort">gpio: <input name="gpio_ch1" type="text" value="%gpio_ch1%"></div>
</div>

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
  // char out[400];
  String states = state_array_string(s.ch[channel_idx].target[target_idx].upstates);
  snprintf(out, 400, "<div><div  class=\"fldlong\">#%i priority target<input name=\"st_%i_t%i\" type=\"text\" value=\"%s\"></div></div><div class=\"fldshort\">Target:<input name = \"t_%i_t%i\" type =\"text\" value = \"%f\"></div></div>", target_idx + 1, channel_idx, target_idx, states.c_str(), channel_idx, target_idx, s.ch[channel_idx].target[target_idx].target);

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
    // return s.http_username;
    return F("powerguru");
  if (var == "http_password")
    return s.http_password;

#ifdef METER_SHELLY3EM
  if (var == "shelly_url")
    return s.shelly_url;
#endif

  if (var == "fronius_address")
#ifdef INVERTER_FRONIUS_SOLARAPI
    return s.fronius_address;
#else
    return F("(disabled)")
#endif

  if (var == "base_load_W")
#ifdef INVERTER_FRONIUS_SOLARAPI
    return String(s.base_load_W);
#else
            return F("(disabled)")
#endif

  if (var == "sensorv0")
#ifdef SENSOR_DS18B20
    return String(ds18B20_temp_c);
#else
                    return F("not in use");
#endif
  if (var.startsWith("target_ch_"))
  {
    // e.g target_ch_0_1
    char out[400];
    int channel_idx = var.substring(10, 11).toInt();
    int target_idx = var.substring(12, 13).toInt();
    Serial.print("channel_idx olisko:");
    Serial.println(var.substring(10, 11));
    Serial.print("#");
    Serial.println(var);
    // var.s

    get_channel_target_fields(out, channel_idx, target_idx);
    return out;
  }
  if (var == "states")
  {
    // char out[400];
    // char[CHANNEL_STATES_MAX * 6 + 1];
    return state_array_string(active_states);
    /*
    String states = String();
    for (int i = 0; i < CHANNEL_STATES_MAX; i++)
    {
      if (active_states[i] > 0) {
        states += String(active_states[i]);
        if (i+1 <CHANNEL_STATES_MAX && (active_states[i+1] > 0) )
          states += String(",");
        }
      else
        break;
    }
    return states;*/
  }

#ifdef QUERY_POWERGURU
  if (var == "pg_url")
    return s.pg_url;
  if (var == "pg_cache_age")
    return String(s.pg_cache_age);

#endif

  // Serial.println("Channel processor");

  for (int i = 0; i < CHANNELS; i++)
  {

    if (var.equals(String("upstates_ch") + i))
    {
      String out;
      for (int j = 0; j < CHANNEL_STATES_MAX; j++)
      {
        if (s.ch[i].upstates_ch[j] == 0)
        {
          break;
        }
        if (j > 0)
        {
          out += ",";
        }
        out += String(s.ch[i].upstates_ch[j]);
      }
      return String(out);
    }
    if (var.equals(String("t_u_ch") + i))
    {
      // return String(target_u_ch[i]);
      return String(s.ch[i].target_u_ch);
    }
    if (var.equals(String("t_b_ch") + i))
    {
      // return String(target_b_ch[i]);
      return String(s.ch[i].target_b_ch);
    }
    if (var.equals(String("gpio_ch") + i))
    {
      return String(s.ch[i].gpio);
    }
    if (var.equals(String("up_ch") + i))
    {
      return String(s.ch[i].is_up ? "up" : "down");
    }
    if (var.equals(String("du_ch") + i))
    {
      return s.ch[i].default_up ? "checked" : "";
    }
  }
  return String();
}

// ...

void set_relays()
{
  int active_state_count = 0;
  // how many current active states we do have
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    if (active_states[i] > 0)
      active_state_count++;
    else
      break;
  }

  for (int i = 0; i < CHANNELS; i++)
  {
    bool any_state_active = false;
    bool new_up = false;
    for (int k = 0; k < active_state_count; k++)
    {
      for (int j = 0; j < CHANNEL_STATES_MAX; j++)
      {
        if (active_states[k] == s.ch[i].upstates_ch[j])
        {
          any_state_active = true;

          // Serial.print(" ,state matches:");
          // Serial.println(s.ch[i].upstates_ch[j]);
          //       goto states_clear;
          break;
        }
      }
    }
//  states_clear:
#ifdef SENSOR_DS18B20
    if ((any_state_active && ds18B20_temp_c < s.ch[i].target_u_ch) || (!any_state_active && ds18B20_temp_c < s.ch[i].target_b_ch))
    {
      new_up = true;
    }
#else
    new_up = any_state_active;
#endif
    if (s.ch[i].is_up != new_up)
    {
      Serial.println();
      Serial.print("channel:");
      Serial.println(i);
      Serial.print("new relay value:");
      s.ch[i].is_up = new_up;
      Serial.print(new_up);
      Serial.print("Setting gpio :");
      Serial.print(s.ch[i].gpio);
      Serial.print(" ");
      Serial.println((s.ch[i].is_up ? HIGH : LOW));
      digitalWrite(s.ch[i].gpio, (s.ch[i].is_up ? HIGH : LOW));
    }

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

  writeToEEPROM();
  delay(1000);
  ESP.restart();
}

void onWebRootPost(AsyncWebServerRequest *request)
{
  String message;
  int paramsNr = request->params();
  Serial.println(paramsNr);
  // s.sta_mode = request->getParam("sta_mode", true)->value().

  s.sta_mode = request->hasParam("sta_mode", true);
  Serial.print("s.sta_mode:");
  Serial.print(s.sta_mode);

  strcpy(s.wifi_ssid, request->getParam("wifi_ssid", true)->value().c_str());
  strcpy(s.wifi_password, request->getParam("wifi_password", true)->value().c_str());
  strcpy(s.http_username, request->getParam("http_username", true)->value().c_str());
  strcpy(s.http_password, request->getParam("http_password", true)->value().c_str());
  strcpy(s.shelly_url, request->getParam("shelly_url", true)->value().c_str());
#ifdef INVERTER_FRONIUS_SOLARAPI
  strcpy(s.fronius_address, request->getParam("fronius_address", true)->value().c_str());
  s.base_load_W = request->getParam("base_load_W", true)->value().toInt();

#endif

#ifdef QUERY_POWERGURU
  strcpy(s.pg_url, request->getParam("pg_url", true)->value().c_str());
  s.pg_cache_age = request->getParam("pg_cache_age", true)->value().toInt();
#endif

  if (request->hasParam("states_ch0", true))
  {

    // uint16_t upstates_ch[CHANNEL_STATES_MAX];
    Serial.print("processing:");
    Serial.println(request->getParam("states_ch0", true)->value().c_str());
    str_to_uint_array(request->getParam("states_ch0", true)->value().c_str(), s.ch[0].upstates_ch);
    Serial.print(s.ch[0].upstates_ch[0]);
    Serial.print("...");
    Serial.print(s.ch[0].upstates_ch[1]);
    Serial.println();
    s.ch[0].gpio = request->getParam("gpio_ch0", true)->value().toInt();
    s.ch[0].default_up = request->hasParam("du_ch0", true);
    s.ch[0].target_b_ch = request->getParam("t_b_ch0", true)->value().toFloat();
    s.ch[0].target_u_ch = request->getParam("t_u_ch0", true)->value().toFloat();
  }
  if (request->hasParam("states_ch1", true))
  {
    str_to_uint_array(request->getParam("states_ch1", true)->value().c_str(), s.ch[1].upstates_ch);
    s.ch[1].gpio = request->getParam("gpio_ch1", true)->value().toInt();
    s.ch[1].default_up = request->hasParam("du_ch1", true);
    s.ch[1].target_b_ch = request->getParam("t_b_ch1", true)->value().toFloat();
    s.ch[1].target_u_ch = request->getParam("t_u_ch1", true)->value().toFloat();
  }
  char state_fld[8];
  char target_fld[7];
  for (int channel_idx = 0; channel_idx < CHANNELS; channel_idx++)
  {
    for (int target_idx = 0; target_idx < CHANNEL_TARGETS_MAX; target_idx++)
    {
      snprintf(state_fld, 8, "st_%i_t%i", channel_idx, target_idx);
      snprintf(target_fld, 7, "t_%i_t%i", channel_idx, target_idx);
      if (request->hasParam(state_fld, true))
      { Serial.print("ON:");
        Serial.println(state_fld);
        str_to_uint_array(request->getParam(state_fld, true)->value().c_str(), s.ch[channel_idx].target[target_idx].upstates);
        s.ch[channel_idx].target[target_idx].target = request->getParam(target_fld, true)->value().toFloat();
      }
      else {
        Serial.print("EI OO:");
        Serial.println(state_fld);
      }
    }
  }
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

#ifdef METER_SHELLY3EM
  float netEnergyInPeriod = (energyin - energyout - energyin_prev + energyout_prev);
  float netPowerInPeriod;
  if ((now_ts - last_period_last_ts) != 0)
  {
    netPowerInPeriod = round(netEnergyInPeriod * 3600.0 / ((now_ts - last_period_last_ts)));
  }
  else
  {
    netPowerInPeriod = 0;
  }
  variables["netEnergyInPeriod"] = netEnergyInPeriod;
  variables["netPowerInPeriod"] = netPowerInPeriod;
#endif

#ifdef INVERTER_FRONIUS_SOLARAPI
  variables["energyProducedPeriod"] = energy_produced_period;
  variables["powerProducedPeriodAvg"] = power_produced_period_avg;
#endif

  variables["updated"] = now_ts;
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

#ifdef SENSOR_DS18B20
  sensors.begin();
#endif
#ifdef QUERY_POWERGURU
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
  if (s.ch[0].gpio == 255) // not the best way
  {
    Serial.println(F("Initiating eeprom"));

    // D2-GPIO 4, 1 wire
    strcpy(s.http_username, "powerguru");
    strcpy(s.http_password, "powerguru");

#ifdef QUERY_POWERGURU
    // strcpy(s.pg_url, "http://192.168.66.8:8080/state_series?price_area=FI");
#endif
#ifdef METER_SHELLY3EM
    // strcpy(s.shelly_url, "http://192.168.66.40/status");
#endif

    s.ch[0] = {{10101}, 20, 80, false, false, 14U}; // D5=GPIO14 , D1	GPIO5
    s.ch[1] = {{10102}, 21, 81, false, false, 12U}; // D6=GPIO12, D3	GPIO0
    writeToEEPROM();
  }

  for (int i = 0; i < CHANNELS; i++)
  {
    Serial.print(F("Setting gpio "));
    Serial.println(s.ch[i].gpio);
    pinMode(s.ch[i].gpio, OUTPUT);
    // if up/down up-to-date stored to non-volatile:
    // digitalWrite(s.ch[i].gpio, (s.ch[i].is_up ? HIGH : LOW));
    //  use defaults
    digitalWrite(s.ch[i].gpio, (s.ch[i].default_up ? HIGH : LOW));
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

      // server.on(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER,)
    }
    else
    {
      delay(5000); // cannot create AP, restart
      ESP.restart();
    }
  }

  // TODO: prepare for no internet connection? -> channel defaults probably, RTC?
  // https://werner.rothschopf.net/202011_arduino_esp8266_ntp_en.htm
  configTime(MY_TZ, MY_NTP_SERVER); // --> Here is the IMPORTANT ONE LINER needed in your sketch!

  server.on("/reset", HTTP_GET, onWebResetGet);
  server.on("/status", HTTP_GET, onWebStatusGet);
  server.on("/", HTTP_GET, onWebRootGet);
  server.on("/", HTTP_POST, onWebRootPost);

  /* if (create_ap) {
     server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
   }
   */

  server.onNotFound(notFound);

  server.begin();

  Serial.print(F("setup() finished:"));
  Serial.println(ESP.getFreeHeap());

#ifdef QUERY_POWERGURU
  // powerguru_last_refresh = -powerguru_interval_s * 1000;
  //  strcpy(s.pg_url, "http://192.168.66.8:8080/state_series?price_area=FI");
#endif

  // first period is not full, so start calculations from now

  // TODO: if not ntp server: could we get old ts from eeprom (rtcmem has more cycles), or how about using date from  powerguru/shelly query (if used)
  //  ehkä tästä saisi jotain, vaikkei olekaan viimeisen päälle, rtcmem olisi paras paikka kun ei kulum kai, https://stackoverflow.com/questions/54458116/how-can-i-set-the-esp32s-clock-without-access-to-the-internet
  do
  {
    time(&started);
    delay(500);
    Serial.print("*");
  } while (started < 1645106860);

  Serial.print("started:");
  Serial.println(started);

} // end of setup()

long get_period_start_time(long ts)
{
  return long(ts / (netting_period_min * 60UL)) * (netting_period_min * 60UL);
}

void loop()
{
  // Serial.print(F("Starting loop"));
  /*if (!s.sta_mode) {
    dnsServer.processNextRequest();
  }*/

  time(&now);
  // current_period_start = long(timeClient.getEpochTime() / (netting_period_min * 60UL)) * (netting_period_min * 60UL);
  current_period_start = get_period_start_time(now); // long(now / (netting_period_min * 60UL)) * (netting_period_min * 60UL);
  if (get_period_start_time(now) == get_period_start_time(started))
    recording_period_start = started;
  else
    recording_period_start = current_period_start;

  if (previous_period_start != current_period_start)
    period_changed = true; // more readable

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    return;
  }

  // TODO: all sensor /meter reads could be here?, do we need diffrent frequencies?
  if (((millis() - sensor_last_refresh) > sensor_read_interval_s * 1000) or period_changed)
  {
    Serial.print(F("Reading sensor and meter data..."));
#ifdef SENSOR_DS18B20
    read_sensor_ds18B20();
#endif
#ifdef METER_SHELLY3EM
    read_meter_shelly3em();
#endif
#ifdef INVERTER_FRONIUS_SOLARAPI
    read_inverter_fronius();
#endif
    refresh_states(current_period_start);
    sensor_last_refresh = millis();
    set_relays(); // tässä voisi katsoa onko tarvetta mennä tähän eli onko tullut muutosta
  }
  /*
  #ifdef QUERY_POWERGURU
    if (((millis() - powerguru_last_refresh) > powerguru_interval_s * 1000) or period_changed)
    {
      // Serial.println(F("Calling refresh_states"));
      refresh_states(current_period_start);
      powerguru_last_refresh = millis();
    }
  #endif
  */

  set_relays(); // tässä voisi katsoa onko tarvetta mennä tähän eli onko tullut muutosta

  if (period_changed)
  {
    previous_period_start = current_period_start;
    period_changed = false;
  }

  delay(5000);
}
