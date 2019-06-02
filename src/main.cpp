#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <TimeUtils.h>
#include <StatusLED.h>
#include <Valves.h>
#include <FlowMeter.h>
#include <time.h>
#include <LongTicker.h>
#include <PushButton.h>
#include <LiquidCrystal_I2C.h>

/*------------------------------------------------------------------------------------*/
/* Constant Definitions                                                               */
/*------------------------------------------------------------------------------------*/
// Access point to configure Wi-Fi
const char* ACCESS_POINT_NAME = "ESP8266";
const char* ACCESS_POINT_PASS = "esp8266";

// Status LED redefinitions
const StatusLED::Status IRRIGATING = StatusLED::Status::custom_1;
const StatusLED::Status ANY_ERROR = StatusLED::Status::custom_2;

// MQTT Constants
const char * MQTT_CLIENT_PREFIX = "DripCtrl-";
const char * MQTT_BROKER_ADDRESS = "192.168.1.215";
const char * MQTT_IN_TOPIC = "/home-assistant/drip/request";

// MQTT Commands
const char MQTT_CMD_CONFIG_DRIP = 'c';   // Configure dripping parameters
const char MQTT_CMD_START_DRIP = 's';    // Start manual dripping
const char MQTT_CMD_STOP_DRIP = 't';     // Stop manual dripping
const char MQTT_CMD_RAIN_DELAY = 'r';    // Set rain delay
const char MQTT_CMD_RESET = 'x';         // Restart system
const char MQTT_CMD_RESET_METER = 'a';   //TODO: restart the flow meter

// MQTT Events
const char * MQTT_REPORT_FLOW = "/home-assistant/drip/flow";
const char * MQTT_DRIP_STARTED = "/home-assistant/drip/started";
const char * MQTT_DRIP_STOPPED = "/home-assistant/drip/stopped";

// Default Drip Values
const char *START_IRRIGATION_TIME = "07:00:00"; // HH:MM:SS
const int IRRIGATION_PERIOD_HOURS = 12;         // Minimum 12 hours
const int IRRIGATION_LONG_MINUTES = 45;         // Maximum 120 minutes
const int RAIN_DELAY_HOURS = 24;                // Minimum 24 hours

// Other Constants
const int LCD_DISPLAY_INTERVAL_SECONDS = 60;    // Update the LCD display

/*------------------------------------------------------------------------------------*/
/* GPIO Definitions                                                                   */
/*------------------------------------------------------------------------------------*/
const int GPIO_VALVE_ENABLE = 0;       // ESP8266 NodeMCU D3 (OUTPUT)
const int GPIO_UNUSED_01 = 2;          // ESP8266 NodeMUC D4 (UART)
const int GPIO_UNUSED_02 = 2;          // ESP8266 NodeMUC D4 (Boot mode. Do not user for INPUT)
const int GPIO_UNUSED_03 = 3;          // ESP8266 NodeMCU D9 (UART)
const int GPIO_DISPLAY_SDA = 4;        // ESP8266 NodeMCU D2 (SDA) 
const int GPIO_DISPLAY_SCL = 5;        // ESP8266 NodeMCU D1 (SCL)
const int GPIO_UNUSED_06 = 6;          // ESP8266 NodeMCU -+ F M
const int GPIO_UNUSED_07 = 7;          // ESP8266 NodeMCU  + L E
const int GPIO_UNUSED_08 = 8;          // ESP8266 NodeMCU  + A M
const int GPIO_UNUSED_09 = 9;          // ESP8266 NodeMCU  + S O
const int GPIO_UNUSED_10 = 10;         // ESP8266 NodeMCU  + H R
const int GPIO_UNUSED_11 = 11;         // ESP8266 NodeMCU -+   Y
const int GPIO_VALVE_SIGNAL = 12;      // ESP8266 NodeMCU D6 (OUTPUT)
const int GPIO_FLOW_METER_SIGNAL = 13; // ESP8266 NodeMUC D7 (INPUT
const int GPIO_STATUS_LED = 14;        // ESP8266 NodeMCU D5 (OUTPUT)
const int GPIO_UNUSED_15 = 15;         // ESP8266 NodeMCU D8 (Boot from SD Card)
const int GPIO_PUSH_BUTTON = 16;       // ESP8266 NodeMCU D0 (INPUT)
/*------------------------------------------------------------------------------------*/
/* Helper Classes                                                                     */
/*------------------------------------------------------------------------------------*/
class DripParams {
  public:
    DripParams(const char *startDripTime, int dripPeriod, int dripTime):
    _period(dripPeriod),
    _duration(dripTime) {
      strcpy(_startTime, startDripTime);
      strptime(startDripTime, "%T", &_start);
      _rainDelayHours = 0;
      _rainDelayResumeTime = TimeUtils::getCurrentTimeRaw() - 1; // No rain delay 
    }
    ~DripParams() {};

    struct tm getStartTime(void) {
      return _start;
    };

    time_t getTodayStartTime(void) {
      struct tm *currentTime = TimeUtils::getCurrentTime();
      currentTime->tm_hour = _start.tm_hour; 
      currentTime->tm_min = _start.tm_min; 
      currentTime->tm_sec = _start.tm_sec;
      return mktime(currentTime); 
    }

    int getDripPeriodSeconds(void) {
        return _period * 3600;
    }

    int getDripTimeSeconds(void) {
      return _duration * 60;
    }

    int getDripTimeMinutes(void) {
      return _duration;
    }

    int getDripPeriodHours(void) {
      return _period;
    }
    
    void setStartDripTime(char *startTime) {
        //Start time should be in the format HH:MM:SS
        strcpy(_startTime, startTime);
        strptime(startTime, "%T", &_start);
    }

    void setDripPeriodHours(int hours) {
      _period = hours;
    }

    void setDripTimeMinutes(int minutes) {
      _duration = minutes;
    }

    void setRainDelay(int hours) {
      _rainDelayHours = hours;
      _rainDelayResumeTime = TimeUtils::getCurrentTimeRaw() + hours * 3600; 
    }

    void resetRainDelay() {
      _rainDelayHours = 0;
      _rainDelayResumeTime = TimeUtils::getCurrentTimeRaw() - 1; // No rain delay 
    }

    bool isRainDelaySet() {
      return _rainDelayHours > 0;  
    }

    time_t getRainDelayResumeTime(void) {
      return _rainDelayResumeTime;
    }

    const char * toString() {
      sprintf(_auxBuffer, "Start Time: %s, Duration: %d minutes, period: %d hours, Rain Delay: %d hours", 
        _startTime, _duration, _period, _rainDelayHours);
      return _auxBuffer;
    }

    void saveToEEPROM() {
      Serial.println("[DRIPCTR]: Saving Scheduling Data to EEPROM");
      // EPROM Struct
      // Byte 0: Valid Data 00, Invalid Data FF
      // Byte 1: Start time hour 0-23
      // Byte 2: Start time minute 0-59
      // Byte 3: Start time second 0-59
      // Byte 4: Drip Period 0,6,12,24
      // Byte 5: Dripping Duration Minutes 0-255
      // Rain delay will not be save and will not survive reboot.
      int addr = 0;
      EEPROM.write(addr, 0x00); addr++;
      EEPROM.write(addr, _start.tm_hour); addr++;
      EEPROM.write(addr, _start.tm_min); addr++;
      EEPROM.write(addr, _start.tm_sec); addr++;
      EEPROM.write(addr, _period); addr++;
      EEPROM.write(addr, _duration); addr++;
      EEPROM.commit();
      Serial.println("[DRIPCTR]: Finished Saving Scheduling Data to EEPROM");
    }
    void restoreFromEEPROM() {
      Serial.println("[DRIPCTR]: Restoring Scheduling Data from EEPROM");
      int addr = 0;
      byte value = EEPROM.read(addr); addr++;
      if (value == 0x00) {
        // Valid data on EEPROM
        int hour = EEPROM.read(addr); addr++;
        int min = EEPROM.read(addr); addr++;
        int sec = EEPROM.read(addr); addr++;
        int period = EEPROM.read(addr); addr++;
        int duration = EEPROM.read(addr);
        // Validate values
        if (hour >= 0 && hour <= 23 && min >= 0 && min <= 59 && sec >= 0 && sec <= 59) {
          // Valid start time
          sprintf(_startTime, "%02d:%02d:%02d", hour, min, sec);
          _start.tm_hour = hour;
          _start.tm_min = min;
          _start.tm_sec = sec;

          if (period % 6 == 0 && period <= 24) {
            // Valid period time
            _period = period; 
            _duration = duration;
            Serial.println("[DRIPCTR]: Scheduling data restored from EEPROM");
            Serial.printf("[DRIPCTR]: Schedule: %s\n", toString());
          } else {
            Serial.printf("[DRIPCTR]: EEPROM contains invalid period %02d\n", period);
          }
        } else {
          Serial.printf("[DRIPCTR]: EEPROM contains invalid start time %02d:%02d:%02d\n", hour, min, sec);
        }
      } else {
        Serial.println("[DRIPCTR]: Scheduling Data on EEPROM is not valid. May be has never been saved");
      }
    }
  private:
    char _auxBuffer[200];
    char _startTime[100];
    struct tm _start;
    int _period;
    int _duration;
    int _rainDelayHours;
    time_t _rainDelayResumeTime;
};


/*------------------------------------------------------------------------------------*/
/* Global Variables                                                                   */
/*------------------------------------------------------------------------------------*/
// WiFi Manager
WiFiManager wifiManager;

// Intervals
LongTicker dripTicker("DRIPTICK");   // Dripping Scheduling
Ticker lcdDisplayUpdate; // Update LCD text

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Drip Valve and Flow Meter
SolenoidValve solenoidValve(GPIO_VALVE_ENABLE, GPIO_VALVE_SIGNAL);
FlowMeter flowMeter(GPIO_FLOW_METER_SIGNAL);

// Status LED
StatusLED statusLed(GPIO_STATUS_LED); 

// Default Drip Parameters.
DripParams dripParams(START_IRRIGATION_TIME, IRRIGATION_PERIOD_HOURS, IRRIGATION_LONG_MINUTES);

// Push Button
PushButton pushButton(GPIO_PUSH_BUTTON, 2, 8);

// Liquid Crystal Display
LiquidCrystal_I2C lcd(0x27,16,2);

// First Line LCD
char lcdLine[17] = "\0";

// Next Drip, dripping remaining, rain delay ramaining
time_t toDisplay;

/*------------------------------------------------------------------------------------*/
/* WiFi Manager Global Functions                                                      */
/*------------------------------------------------------------------------------------*/
// WiFiManager Configuration CallBack
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("[WIFI]: Entered config mode");
  Serial.print("[WIFI]:"); Serial.println(WiFi.softAPIP());
  Serial.printf("[WIFI]: %s", (myWiFiManager->getConfigPortalSSID()).c_str());
}

/*------------------------------------------------------------------------------------*/
/* Other Global Functions                                                             */
/*------------------------------------------------------------------------------------*/
void scheduleDrip(void);

void reportFlow() {
  char payload[20];
  sprintf(payload, "%d", flowMeter.getCountedLiters(true));
  Serial.printf("[DRIPCTRL]: Reporting flow. Liters: %s\n", payload);
  mqttClient.publish(MQTT_REPORT_FLOW, payload);
}

void updateLcd(bool noTimeDisplay) {
  char aux[50];
  time_t now = TimeUtils::getCurrentTimeRaw();
  Serial.printf("[DEBUG]: toDisplay: %d\n", toDisplay);
  int remaining = toDisplay - now;
  int minutes = remaining / 60;
  int hours = 0;
  Serial.printf("[DEBUG]: remaining: %d\n", remaining);
  if (minutes > 59) {
     hours = minutes / 60;
     minutes = minutes % 60;
  }
  Serial.printf("[DEBUG]: hours: %02d, minutes: %02d\n", hours, minutes);
  sprintf(aux, "%s %02d:%02d", lcdLine, hours, minutes);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(noTimeDisplay ? lcdLine : aux);
  lcd.setCursor(0,1);
  lcd.print(TimeUtils::getTimeStr(now).c_str());
}

void stopScheduledDrip(void) {
  solenoidValve.closeValve();
  mqttClient.publish(MQTT_DRIP_STOPPED, "");
  reportFlow();
  statusLed.setStatus(StatusLED::Status::stable);
  scheduleDrip();
}

void startScheduledDrip(void) {
  solenoidValve.openValve();
  mqttClient.publish(MQTT_DRIP_STARTED, "");
  statusLed.setStatus(IRRIGATING);
  sprintf(lcdLine,"Dripping");
  // Current Time
  time_t nowRaw = TimeUtils::getCurrentTimeRaw();
  // Dripping duration
  int duration = dripParams.getDripTimeSeconds();
  toDisplay = nowRaw + duration;
  dripTicker.once(duration / 60, stopScheduledDrip);
  updateLcd(false);
}

void scheduleDrip() {
  // Scheduler: 
  //           Depending on the parameters, the result of the scheduling could be:
  //           1. Start dripping. Schedule to stop
  //              The current time is in between dripping scheduled periods
  //           2. Schedule to start dripping
  //              The current time is outside dripping scheduled periods but there is a dripping schedule for today
  //           3. Schedule to reschedule
  //              The current time is later than any schedule period for today, or there is a rain delay

  Serial.printf("[DRIPCTRL] Scheduling: %s\n", dripParams.toString());
  
  // Current Time
  time_t nowRaw = TimeUtils::getCurrentTimeRaw();
  // First schedule dripping for today
  time_t dripStart = dripParams.getTodayStartTime();
  // Dripping duration
  int duration = dripParams.getDripTimeSeconds();
  // Dripping time in seconds. If set to greatet than 0, then the result was #1. See above.
  int dripTimeSeconds = 0;
  // Time to next dripping in seconds. If set to greater than 0, then the result was #2. See above.
  int timeToNextDripSeconds = 0;
  // Time to next scheduling in minutes. If set to greater that 0, the the result was #3. See above.
  int rescheduleTimeMinutes = 0;
  // Second dripping of the day
  time_t secondTime = dripStart + dripParams.getDripPeriodSeconds();
  // Rain delay: resume time
  time_t rainDelayResumeTime = dripParams.getRainDelayResumeTime();
  
  // Is there a rain delay in effect?
  if (nowRaw < rainDelayResumeTime - 60) {
    // Reschedule at rainDelayResumeTime
    rescheduleTimeMinutes = (rainDelayResumeTime - nowRaw) / 60;
    toDisplay = rainDelayResumeTime;
    Serial.printf("[DRIPCTRL]: Within rain delay. Reschedule in %d minutes\n", rescheduleTimeMinutes);
  } else if (nowRaw < dripStart) {
    // Too early to start dripping today
    timeToNextDripSeconds = dripStart - nowRaw;
    Serial.println("[DRIPCTRL]: Too early for first dripping");
    toDisplay = dripStart;
  } else if (nowRaw >= dripStart && nowRaw < (dripStart + duration)) {
    // Within first dripping of the day
    dripTimeSeconds = dripStart + duration - nowRaw + 1;  
    toDisplay = dripStart + duration;
  } else if ( nowRaw >= secondTime && nowRaw < (secondTime + duration)) {
    // Within second dripping of the day
    dripTimeSeconds = secondTime + duration - nowRaw + 1;
    toDisplay = secondTime + duration;
  } else if ( nowRaw < secondTime) {
    // Too late for first dripping. Too early for next
    Serial.println("[DRIPCTRL]: Too late for first, too early for second");
    timeToNextDripSeconds = secondTime - nowRaw + 1;
    toDisplay = secondTime;
  } else {
    // Too late for dripping today
    Serial.println("[DRIPCTRL]: Not more dripping today. Reschedule at midnight");
    rescheduleTimeMinutes = TimeUtils::minutesTillMidnight();
    toDisplay = dripStart + 24 * 3600;
  }
  if (dripTimeSeconds > 0) {
    // #1 Start dripping. Schedule to stop
    dripTimeSeconds = dripTimeSeconds < 60 ? 60 : dripTimeSeconds;
    Serial.printf("[DRIPCTRL]: Start Drip for %d seconds\n", dripTimeSeconds);
    statusLed.setStatus(IRRIGATING);
    solenoidValve.openValve();
    mqttClient.publish(MQTT_DRIP_STARTED, "");
    dripTicker.once((dripTimeSeconds / 60) + 1, stopScheduledDrip);
    sprintf(lcdLine, "Dripping...");
  } else if (rescheduleTimeMinutes > 0) {
    // #3 Too late for dripping today, or rain delay. Schedule to re-schedule
    dripTicker.once(rescheduleTimeMinutes + 1, scheduleDrip);
    sprintf(lcdLine, "Done today");
  } else {
    // #2 Schedule to start dripping
    Serial.printf("[DRIPCTRL]: Not time for dripping. %d seconds to next dripping.\n", timeToNextDripSeconds);
    timeToNextDripSeconds = timeToNextDripSeconds < 60 ? 60 : timeToNextDripSeconds;
    dripTicker.once((timeToNextDripSeconds / 60) + 1, startScheduledDrip);
    sprintf(lcdLine, "Scheduled");
  }
  updateLcd(false);
}

/*------------------------------------------------------------------------------------*/
/* MQTT Global Functions                                                              */
/*------------------------------------------------------------------------------------*/
// MQTT Subscribe Callback
void callback(char* topic, byte* payload, unsigned int length) {
  bool noTimeDisplay = false;
  Serial.printf("[MQTT]: Message arrived [%s]\n", topic);
  Serial.print("[MQTT]: Payload (");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println(")");
  char aux[40];
  int rainDelay;
  switch((char) payload[0]) {
    case MQTT_CMD_CONFIG_DRIP: // Configutation in the format of HH:MM:SSMMHH where HH:MM:SS is start time, MM duration, and HH period
      sprintf(aux, "%c%c\0", payload[9], payload[10]);
      dripParams.setDripTimeMinutes(atoi(aux));
      sprintf(aux, "%c%c\0", payload[11], payload[12]);
      dripParams.setDripPeriodHours(atoi(aux));
      sprintf(aux,"%c%c:%c%c:%c%c\0", payload[1], payload[2], payload[4], payload[5], payload[7], payload[8]);
      dripParams.setStartDripTime(aux);
      Serial.printf("[DRIPCTRL]: New Drip Configuration: StartTime(%s), Duration(%d minutes), Period(%d hours)\n", 
        aux, dripParams.getDripTimeMinutes(), dripParams.getDripPeriodHours());

      if (!solenoidValve.isValveOpen()) {
          dripTicker.detach();
          scheduleDrip();
      }
      dripParams.saveToEEPROM();
      break;
    case MQTT_CMD_RAIN_DELAY: // Rain delay in the format of HH which is hours to not drip
      sprintf(aux, "%c%c\0", payload[1], (length <= 2 ? 0 : payload[2]));
      rainDelay = atoi(aux);
      if (rainDelay > 0) {
        dripParams.setRainDelay(atoi(aux));
        Serial.printf("[DRIPCTRL]: Rain delay set for %s hours\n", aux);
        if (!solenoidValve.isValveOpen()) {
            dripTicker.detach();
            scheduleDrip();
        }
        sprintf(lcdLine, "Rain Delay");
      } else {
        Serial.println("[DRIPCTRL]: Cancel rain delay");
        dripTicker.detach();
        dripParams.setRainDelay(0);
        scheduleDrip();
      }
      break;
    case MQTT_CMD_START_DRIP: // Start dripping in the format of MM which is the drip time in minutes
      sprintf(aux, "%c%c\0", payload[1], payload[2]);
      Serial.printf("[DRIPCTRL]: Start manual dripping for %s minutes\n", aux);
      if (solenoidValve.isValveOpen()) {
        Serial.println("[DRIPCTRL]: Already dripping. Ignore Command");
        return;
      }
      dripTicker.detach();
      solenoidValve.openValve();
      mqttClient.publish(MQTT_DRIP_STARTED, "");
      statusLed.setStatus(IRRIGATING);
      dripTicker.once(atoi(aux), stopScheduledDrip);
      toDisplay = TimeUtils::getCurrentTimeRaw() + atoi(aux) * 60;
      sprintf(lcdLine, "Driping");
      break;
    case MQTT_CMD_STOP_DRIP: // Stop dripping
      Serial.println("[DRIPCTRL]: Stop manual dripping");
      if (!solenoidValve.isValveOpen()) {
        Serial.println("[DRIPCTRL]: Not dripping now. Ignore Command");
        return;
      }
      dripTicker.detach();
      solenoidValve.closeValve();
      mqttClient.publish(MQTT_DRIP_STOPPED, "");
      reportFlow();
      statusLed.setStatus(StatusLED::Status::stable);
      scheduleDrip();
      break;
    case MQTT_CMD_RESET: // Reset system
      sprintf(lcdLine, "Reseting");
      updateLcd(true);
      Serial.println("[DRIPCTRL]: Reseting system...");
      delay(5);
      ESP.reset();
      break;
    defaul: 
      Serial.printf("[MQTT]: Unknown MQTT Command: %c\n", (char) payload[0]);
      break;
  }
  updateLcd(noTimeDisplay);
}

// MQTT Client reconnection
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println("[MQTT]: Attempting MQTT connection...");
    // Create a random client ID
    String clientId = MQTT_CLIENT_PREFIX;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("[MQTT]: Connected");
      // ... and resubscribe
      mqttClient.subscribe(MQTT_IN_TOPIC);
    } else {
      Serial.printf("[MQTT]: Failed, rc= %s, try again in 5 seconds\n", mqttClient.state());
      // Visual Indication
      sprintf(lcdLine, "MQTT Error: %d",mqttClient.state());
      updateLcd(true);
      statusLed.setStatus(ANY_ERROR);
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*------------------------------------------------------------------------------------*/
/* Other Helpers                                                                      */
/*------------------------------------------------------------------------------------*/
void onPushButtonPressedOnStart() {
  Serial.println("[DRIPCTRL]: Button Pressed on Start. Delete Wi-Fi credentials and reset");
  wifiManager.resetSettings();
  delay(5);
  ESP.reset();
}
void onPushButtonVeryShortlyPressed() {
  bool noTimeDisplay = true;
  Serial.println("[DRIPCTRL]: Button Pressed very shortly. Switch Solenoid Valve");
  if (solenoidValve.isValveOpen()) {
    statusLed.setStatus(StatusLED::Status::stable);
    solenoidValve.closeValve();
    mqttClient.publish(MQTT_DRIP_STOPPED, "");
    reportFlow();
    scheduleDrip();
    noTimeDisplay = false;
  } else {
    statusLed.setStatus(IRRIGATING);
    solenoidValve.openValve();
    mqttClient.publish(MQTT_DRIP_STARTED, "");
    sprintf(lcdLine,"Dripping");
  }
  updateLcd(noTimeDisplay);
}
void onPushButtonShortlyPressed() {
  Serial.println("[DRIPCTRL]: Button Pressed shortly");
  if (dripParams.isRainDelaySet()) {
    Serial.println("[DRIPCTRL]: Rain delay was set. Reset it.");
    dripParams.resetRainDelay();
    scheduleDrip();
  } else {
    Serial.println("[DRIPCTRL]: Rain was not set. Set rain delay for 24hs");
    dripParams.setRainDelay(24);
    sprintf(lcdLine,"Rain Delay");
  }
  updateLcd(false);
}

void onPushButtonLongPressed() {
  Serial.println("[DRIPCTRL]: Button Pressed on Start. Reseting...");
  sprintf(lcdLine, "Resetting");
  updateLcd(true);
  delay(5);
  ESP.reset();
}

/*------------------------------------------------------------------------------------*/
/* Setup                                                                              */
/*------------------------------------------------------------------------------------*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);
  lcd.init();
  lcd.backlight();
  lcd.print("  Initializing");
  lcd.setCursor(0,1);
  lcd.print("  Drip Control");

  // Push button setup
  pushButton.setup(onPushButtonPressedOnStart, onPushButtonVeryShortlyPressed, onPushButtonShortlyPressed, onPushButtonLongPressed);
  
  // Instantiate and setup WiFiManager
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect(ACCESS_POINT_NAME, ACCESS_POINT_PASS)) {
    Serial.println("Failed to connect and hit timeout");
    ESP.reset();
    delay(1000);  
  }

  // Config time
  setenv("TZ", "EST5EDT,M3.2.0/02:00:00,M11.1.0/02:00:00", 1);
  configTime(0, 0, "pool.ntp.org");

  // Initialize OTA (Over the air) update
  ArduinoOTA.setHostname(ACCESS_POINT_NAME);
  ArduinoOTA.setPassword(ACCESS_POINT_PASS);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA]: Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA]: End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA]: Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA]: Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("[OTA]: Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("[OTA]: Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("[OTA]: Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("[OTA]: Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("[OTA]: End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("[OTA]: Ready");

  mqttClient.setServer(MQTT_BROKER_ADDRESS, 1883);
  mqttClient.setCallback(callback);

  // Start flow metering
  flowMeter.start();

  // Allow valve to know whether fluid is flowing
  solenoidValve.setFlowMeter(&flowMeter);
  
  statusLed.setStatus(StatusLED::Status::stable);
  lcd.clear();
  updateLcd(true);
  lcdDisplayUpdate.attach(LCD_DISPLAY_INTERVAL_SECONDS, updateLcd, false); 
  dripParams.restoreFromEEPROM();
  scheduleDrip();
}

/*------------------------------------------------------------------------------------*/
/* Loop                                                                               */
/*------------------------------------------------------------------------------------*/
void loop() {
  // OTA
  ArduinoOTA.handle();
  
  // Flow Meter
  flowMeter.run();

  // MQTT
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Push Button
  pushButton.run();

  // Solenoid Valve
  solenoidValve.run();

}
