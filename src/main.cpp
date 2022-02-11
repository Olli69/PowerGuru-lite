
// https://registry.platformio.org/libraries/me-no-dev/ESP%20Async%20WebServer/installation

#include <Arduino.h>
#include <math.h> //round
#include <EEPROM.h>

#include <DNSServer.h> // for captive portal

#ifdef ESP32 //not fully implemented with ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266) // tested only with ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif

#define QUERY_POWERGURU
#define METER_SHELLY3EM
#define SENSOR_DS18B20
#define NTP_TIME

#define RTCMEMORYSTART 65
#define eepromaddr 0

#include <ESPAsyncWebServer.h>

// NTP time sync
#ifdef NTP_TIME
#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");
#endif

DNSServer dnsServer;
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
unsigned ds18B20_interval_s = 30;
unsigned long ds18B20_last = -ds18B20_interval_s * 1000;
float ds18B20_temp_c;
#endif

#ifdef QUERY_POWERGURU
// char powerguru_url[] = "http://192.168.66.8:8080/state_series?price_area=FI";

unsigned powerguru_interval_s = 3600;
unsigned long powerguru_last = -powerguru_interval_s * 1000; //
#endif

#ifdef METER_SHELLY3EM
// const char *shelly_url = "http://192.168.66.40/status";
unsigned shelly3em_interval_s = 30;
unsigned long shelly3em_last = -shelly3em_interval_s * 1000;
#endif

//  the following variables are unsigned longs because the time, measured in
//  milliseconds, will quickly become a bigger number than can be stored in an int.
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)

const int netting_period_min = 60; // should be 60
int current_period_start = 0;

#define CHANNELS 2
#define CHANNEL_STATES_MAX 10
#define MAX_ID_STR_LENGTH 30
#define MAX_URL_STR_LENGTH 70
// nämä flashiin, voisiko olla array of unsigned int 0-65k
typedef struct
{
  uint16_t upstates_ch[CHANNEL_STATES_MAX];
  float target_b_ch;
  float target_u_ch;
  bool is_up;
  bool default_up;
  uint8_t gpio;

} channel_struct;

//TODO: add fixed ip, subnet?
typedef struct
{
  bool sta_mode;
  char wifi_ssid[MAX_ID_STR_LENGTH];
  char wifi_password[MAX_ID_STR_LENGTH];  
  char http_username[MAX_ID_STR_LENGTH];
  char http_password[MAX_ID_STR_LENGTH];
  channel_struct ch[CHANNELS];
#ifdef QUERY_POWERGURU
  char powerguru_url[MAX_URL_STR_LENGTH];
#endif
#ifdef METER_SHELLY3EM
  char shelly_url[MAX_URL_STR_LENGTH];
#endif
} settings_struct;

settings_struct s;

// channel_struct ch[CHANNELS];
uint16_t active_states[CHANNEL_STATES_MAX];

void str_to_uint_array(const char *str_in, uint16_t array_out[CHANNEL_STATES_MAX])
{
  char *ptr = strtok((char *)str_in, ",");
  byte i = 0;

  for (int j = 0; j < CHANNEL_STATES_MAX; j++)
  {
    array_out[j] = 0;
  }

  Serial.print("str_to_uint_array->");
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
  Serial.println("<-str_to_uint_array");

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

String httpGETRequest(const char *url)
{
  WiFiClient client;
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(client, url);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0)
  {
    payload = http.getString();
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
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
  Serial.print(F("Starting to read Shelly"));
  DynamicJsonDocument doc(2048); // oli 1536

  // sensorReadings = ;

  DeserializationError error = deserializeJson(doc, httpGETRequest(s.shelly_url));
  if (error)
  {
    Serial.print(F("Meter deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  now_ts = doc["unixtime"];
  unsigned now_period = int(now_ts / (netting_period_min * 60));

  if (last_period != now_period and last_period > 0)
  { // new period
    Serial.println(F("VAIHTO"));
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

#ifdef QUERY_POWERGURU

void query_powerguru(int current_period_start)
{
  if (strlen(s.powerguru_url) == 0)
    return;
  // Stream& input;
  // TODO: save to a file and use filter with the cached json

  // StaticJsonDocument<400> doc;
  String url_to_call = String(s.powerguru_url) + "&states=";

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
  Serial.print(F("current_period_start:"));
  Serial.println(current_period_start);
  StaticJsonDocument<16> filter;
  char start_str[11];
  itoa(current_period_start, start_str, CHANNEL_STATES_MAX);
  filter[(const char *)start_str] = true;

  StaticJsonDocument<100> doc;

  Serial.println(httpGETRequest(url_to_call.c_str()));

  DeserializationError error = deserializeJson(doc, httpGETRequest(url_to_call.c_str()), DeserializationOption::Filter(filter));

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  JsonArray state_list = doc[start_str];
  Serial.println("state_list info:");
  Serial.println(start_str);
  Serial.println(state_list.size());

  // clean old
  for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    active_states[i] = 0;
  }
  // add internal
  byte idx = 0;
  active_states[idx++] = 1; // always on
#ifdef METER_SHELLY3EM
  active_states[idx++] = (energyin - energyout - energyin_prev + energyout_prev) > 0 ? 1001 : 1002;
#endif

  // for (unsigned int i = idx; i < state_list.size(); i++)
  for (unsigned int i = 0; i < state_list.size(); i++)
  {
    // for (String state : doc["states"].as<String>()) {
    //  const char* states_0 = states[0];
    Serial.print("state:");
    Serial.print((const char *)state_list[i]);
    Serial.print("#");
    Serial.println((int)state_list[i]);
    active_states[idx++] = (uint16_t)state_list[i];
    if (idx == CHANNEL_STATES_MAX)
      break;
  }
}
#endif

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
      input[type=checkbox] {width:50px;}
      input[type=submit] {margin-top:30px;}
      .fld, #fldh{margin-top: 10px;}
      .fldh {float:left; width:45%%;margin-right:3%%; }
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
<div class="fld"><div>Powerguru server url</div><div><input name="powerguru_url" type="text" value="%powerguru_url%"></div></div>
<h2>Channels</h2>
<div class="fld"><div>Current sensor value: %sensorv0%</div></div>
<h3>Channel 1</h3>
<div class="fld"><div>Current status: %up_ch0%</div></div>
<div class="fld"><div>Up on states</div><div><input name="states_ch0" type="text" value="%upstates_ch0%"></div></div>
<div class="fld"><div>Sensor targets</div></div>
<div class="fldh">Base: <input name="t_b_ch0" type="text" value="%t_b_ch0%"></div>
<div class="fldh">Upper: <input name="t_u_ch0" type="text" value="%t_u_ch0%"></div>

<div><div class="fld">Channel options</div><div class="fldh"><input type="checkbox"  name="du_ch0" value="du_ch0" %du_ch0%><label for="du_ch0"> default up</label></div>
<div class="fldh">gpio: <input name="gpio_ch0" type="text" value="%gpio_ch0%"></div>
</div>


<h3>Channel 2</h3>
<div class="fld"><div>Current status: %up_ch1%</div></div>
<div class="fld"><div>Up on states</div><div><input name="states_ch1" type="text" value="%upstates_ch1%"></div></div>
<div class="fld"><div>Sensor targets</div></div>
<div class="fldh">Base: <input name="t_b_ch1" type="text" value="%t_b_ch1%"></div>
<div class="fldh">Upper: <input name="t_u_ch1" type="text" value="%t_u_ch1%"></div>
<div><div class="fld">Channel options</div><div class="fldh"><input type="checkbox"  name="du_ch1" value="du_ch1" %du_ch1% ><label for="du_ch1"> default up</label></div>
<div class="fldh">gpio: <input name="gpio_ch1" type="text" value="%gpio_ch1%"></div>
</div>

<br><input type="submit" value="Save">  
</form>
</body></html>)===";

String setup_form_processor(const String &var)
{
  
  if (var == "sta_mode")
    return s.sta_mode?"checked":"";
  if (var == "wifi_ssid")
    return s.wifi_ssid;
  if (var == "wifi_password")
    return s.wifi_password;
  if (var == "http_username")
    //return s.http_username;
    return String("powerguru");
  if (var == "http_password")
    return s.http_password;

#ifdef METER_SHELLY3EM
  if (var == "shelly_url")
    return s.shelly_url;
#endif

  if (var == "sensorv0")
#ifdef SENSOR_DS18B20
    return String(ds18B20_temp_c);
#else
    return String("not in use");
#endif

#ifdef QUERY_POWERGURU
  if (var == "powerguru_url")
    return s.powerguru_url;
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
      return s.ch[i].default_up?"checked":"";
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

          Serial.print(" ,state matches:");
          Serial.println(s.ch[i].upstates_ch[j]);
          //      goto states_clear;
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

void setup()
{
  Serial.begin(115200);

#ifdef SENSOR_DS18B20
  sensors.begin();
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
    strcpy(s.powerguru_url, "http://192.168.66.8:8080/state_series?price_area=FI");
#endif
#ifdef METER_SHELLY3EM
    strcpy(s.shelly_url, "http://192.168.66.40/status");
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
    digitalWrite(s.ch[i].gpio, (s.ch[i].is_up ? HIGH : LOW));
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
  

  if (s.sta_mode){
    WiFi.mode(WIFI_STA);
    WiFi.begin(s.wifi_ssid, s.wifi_password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      Serial.println(F("WiFi Failed!"));
      create_ap = true; // try to create AP instead
    }
    else {
      Serial.print(F("IP Address: "));
      Serial.println(WiFi.localIP());
      WiFi.setAutoReconnect(true);
      WiFi.persistent(true);
    }
  }

  if (create_ap) //Softap should be created if so defined, cannot connect to wifi , redirect
  { // check also https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/CaptivePortal/CaptivePortal.ino
    String mac = WiFi.macAddress();
    for (int i = 14; i > 0;i-=3) {
      mac.remove(i,1);
    }
    String APSSID = String("powerguru-") + mac;
    
    if (WiFi.softAP(APSSID.c_str(), "powerguru", (int)random(1, 14), false, 1) == true)
    {
      Serial.println(F("WiFi AP created with ip"));
      Serial.println(WiFi.softAPIP().toString());
      // dnsServer.start(53, "*", WiFi.softAPIP());

      //server.on(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER,)
    }
    else {
      delay(5000); // cannot create AP, restart
      ESP.restart();
    }
  }






#ifdef NTP_TIME
  timeClient.begin();
  delay(1000);
  timeClient.update();
  Serial.println(timeClient.getFormattedTime());
  Serial.println(timeClient.getEpochTime());
#endif
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){ 
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
    
    });

#ifdef METER_SHELLY3EM
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
            { 
      if(!request->authenticate(s.http_username, s.http_password)) {
      return request->requestAuthentication();
      }
    StaticJsonDocument<150> doc; //oli 128, lisätty heapille vähän
    String output;
    float net_energyin_period = (energyin - energyout - energyin_prev + energyout_prev);
    float net_power_period;
    if ((now_ts - last_period_last_ts) != 0)
    {
      net_power_period = round(net_energyin_period*3600.0 / ((now_ts - last_period_last_ts) ));
      
    }
    else
    {
      net_power_period = 0;
    }

    JsonObject variables = doc.createNestedObject("variables");
    variables["net_energyin_period"] = net_energyin_period;
    variables["net_power_period"] = net_power_period;
    variables["updated"] = now_ts;
    variables["freeHeap"] = ESP.getFreeHeap();

    
    //doc["states"][0] = variables["net_energyin_period"]>0 ? "1001":"1002";

      for (int i = 0; i < CHANNEL_STATES_MAX; i++)
  {
    doc["states"][i] = active_states[i];
    if (active_states[i] > 0)
      doc["states"][i] = active_states[i];
    else
      break;
  }

    serializeJson(doc, output);
    request->send(200, "application/json", output); });
#endif

  // Send a GET request to <IP>/get?message=<message>
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (!request->authenticate(s.http_username, s.http_password))
                return request->requestAuthentication();
              String message;
              // large char array, tested with 14k
              request->send_P(200, "text/html", setup_form_html, setup_form_processor); });

  // Send a POST request to <IP>/post with a form field message set to <message>
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
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
              strcpy(s.powerguru_url, request->getParam("powerguru_url", true)->value().c_str());
              

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


              // save to non-volatile memory
              writeToEEPROM();

              // AsyncWebParameter* paramUN = request->getParam("USERNAME",true);
              for (int i = 0; i < paramsNr; i++)
              {
                AsyncWebParameter *p = request->getParam(i);

                Serial.print("Param name: ");
                Serial.println(p->name());

                Serial.print("Param value: ");
                Serial.println(p->value());

                Serial.println("------");
              }
              /*
              if (request->hasParam(PARAM_MESSAGE, true)) {
                  message = request->getParam(PARAM_MESSAGE, true)->value();
              } else {
                  message = "No message sent";
              }
    */

              request->redirect("/");
              // request->send(200, "text/plain", "Hello, POST: " + message);
            });

 /* if (create_ap) {
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
  }
  */

  server.onNotFound(notFound);

  server.begin();

  Serial.print(F("setup() finished:"));
  Serial.println(ESP.getFreeHeap());

  // String upstates_ch[CHANNELS] = {"10101,10120, 11022", "10102,10104,11022,12110,11141"};
  // float target_ch[CHANNELS] = {25, 99};

  // uint16_t ups_ch[CHANNEL_STATES_MAX];
  /*
  uint16_t* ups_ch = new uint16_t[CHANNEL_STATES_MAX];
  Serial.println("alku");
  String_to_uint_array(String("10101,10120, 11022"), ups_ch);
  Serial.println("jatkuu");
  for (int i = 0; i < CHANNEL_STATES_MAX;i++){
    Serial.print(i);Serial.print(": ");
    Serial.println(ups_ch[i]);

  }
  */
}

void loop()
{
  //Serial.print(F("Starting loop"));
  /*if (!s.sta_mode) {
    dnsServer.processNextRequest();
  }*/

  current_period_start = int(timeClient.getEpochTime() / (netting_period_min * 60)) * (netting_period_min * 60);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    return;
  }

#ifdef SENSOR_DS18B20
  if ((millis() - ds18B20_last) > ds18B20_interval_s * 1000)
  {
    Serial.print(F("Calling read_sensor_ds18B20"));
    read_sensor_ds18B20();
    ds18B20_last = millis();
  }
#endif

#ifdef METER_SHELLY3EM
  if ((millis() - shelly3em_last) > shelly3em_interval_s * 1000)
  {
    Serial.print(F("Calling read_meter_shelly3em"));
    read_meter_shelly3em();
    shelly3em_last = millis();
  }
#endif

#ifdef QUERY_POWERGURU
  if ((millis() - powerguru_last) > powerguru_interval_s * 1000)
  {
    Serial.print(F("Calling query_powerguru"));
    query_powerguru(current_period_start);
    powerguru_last = millis();
  }
#endif

  set_relays(); // tässä voisi katsoa onko tarvetta mennä tähän eli onko tullut muutosta

  delay(10000);
}
