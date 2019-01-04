#include <FS.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <JC_Button.h> // https://github.com/JChristensen/JC_Button
#include <fauxmoESP.h>
#include <RemoteDebug.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include "secrets.h"

#define RELAY_PIN_SW 4
#define WIFI_LED_PIN 12

const byte
    BUTTON_PIN(5),
    LED_PIN(13);
const unsigned long
    LONG_PRESS(2000),
    BLINK_INTERVAL(100);

enum btn_states_t
{
  ONOFF,
  TO_BLINK,
  BLINK,
  TO_ONOFF
};

bool ledState;            // current LED status
unsigned long btn_ms;     // current time from millis()
unsigned long btn_msLast; // last time the LED was switched

char plugname[32]; // nicht vergessen platformio.ini zu ändern!
volatile int desiredRelayState = 0;
volatile int relayState = 0;
bool relay_status = false;
bool shouldSaveConfig = false;

Button MyButton(BUTTON_PIN);
struct RemoteDebug Debug;
struct fauxmoESP fauxmo;
struct WiFiManager wifiManager;

void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setRelay(bool state)
{
  relay_status = state;

  if (relay_status)
  {
    digitalWrite(RELAY_PIN_SW, HIGH);
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN_SW, LOW);
    digitalWrite(LED_PIN, LOW);
  }

  Serial.print("relay ");
  Serial.println(relay_status ? "on" : "off");
}

void otaSetup()
{
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(plugname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

[[noreturn]] void factoryReset() {
  SPIFFS.remove("/config.json");
  WiFi.disconnect(true);
  SPIFFS.format();
  wifiManager.resetSettings();
  while (1)
    ; // deliberate crash (restart doesn't properly clear config!)
}

void longPress()
{
  Serial.println("Long Press");
  rdebugIln("Long Press");
  wifiManager.resetSettings();
  delay(1000);
  SPIFFS.format();
  delay(1000);
  ESP.restart();
}

void shortPress()
{
  Serial.println("Short Press");
  rdebugIln("Short Press");
  desiredRelayState = !desiredRelayState;
  setRelay(desiredRelayState);
}

void configModeCallback(WiFiManager *wifiManager)
{
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(WIFI_LED_PIN, HIGH);
  rdebugIln("Back to config mode");
}

// reverse the current LED state. if it's on, turn it off. if it's off, turn it on.
void switchLED()
{
  btn_msLast = btn_ms; // record the last switch time
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
}

// switch the LED on and off every BLINK_INTERVAL milliseconds.
void fastBlink()
{
  if (btn_ms - btn_msLast >= BLINK_INTERVAL)
    switchLED();
}

void setup()
{
  Serial.begin(115200);
  pinMode(RELAY_PIN_SW, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  MyButton.begin();

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
          Serial.println("\nparsed json");
          strcpy(plugname, json["plugname"]);
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter custom_plugname("plugname", "Geraete Name", plugname, 32);
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_plugname);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect("LenAlexa");

  digitalWrite(WIFI_LED_PIN, HIGH);

  otaSetup();
  Debug.begin(plugname);
  Debug.setResetCmdEnabled(true);

  fauxmo.addDevice(plugname);
  fauxmo.setPort(80); // required for gen3 devices
  fauxmo.enable(true);
  fauxmo.onSetState([](unsigned char device_id, const char *device_name, bool state, unsigned char value) {
    Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
    rdebugIln("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
    setRelay(state);
  });

  strcpy(plugname, custom_plugname.getValue());

  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["plugname"] = plugname;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
}

void loop()
{
  fauxmo.handle();
  yield();
  ArduinoOTA.handle();
  Debug.handle();

  static btn_states_t STATE;
  btn_ms = millis();
  MyButton.read();

  switch (STATE)
  {
  // this state watches for short and long presses, switches the LED for
  // short presses, and moves to the TO_BLINK state for long presses.
  case ONOFF:
    if (MyButton.wasReleased())
    {
      switchLED();
      shortPress();
    }
    else if (MyButton.pressedFor(LONG_PRESS))
      STATE = TO_BLINK;
    break;

  // this is a transition state where we start the fast blink as feedback to the user,
  // but we also need to wait for the user to release the button, i.e. end the
  // long press, before moving to the BLINK state.
  case TO_BLINK:
    if (MyButton.wasReleased())
    {
      STATE = BLINK;
      longPress();
    }
    else
      fastBlink();
    break;

  // the fast-blink state. Watch for another long press which will cause us to
  // turn the LED off (as feedback to the user) and move to the TO_ONOFF state.
  case BLINK:
    if (MyButton.pressedFor(LONG_PRESS))
    {
      STATE = TO_ONOFF;
      digitalWrite(LED_PIN, LOW);
      ledState = false;
    }
    else
      fastBlink();
    break;

  // this is a transition state where we just wait for the user to release the button
  // before moving back to the ONOFF state.
  case TO_ONOFF:
    if (MyButton.wasReleased())
      STATE = ONOFF;
    break;
  }
}
