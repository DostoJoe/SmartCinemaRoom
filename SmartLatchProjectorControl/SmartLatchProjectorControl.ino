  /***

  ESP RainMaker IoT connected door latch and living room automation
  using MG90S servo
  using IR LED control (Projector)
  using 315MHz RF transmitter (Projector Screen) 
  
  ***/
  
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ESP32Servo.h>
#include <IRremote.hpp>
#include <RCSwitch.h>
#include <AceButton.h>

using namespace ace_button;

const char *service_name = "RainMaker_BLE";
const char *pop = "12345678";

// define the Chip Id
uint32_t espChipId = 0;

// define the Node Name
char nodeName[] = "ESP32_Living_Room_Automation";

// define the Device Names
char latchServo[] = "Latch";
char projector[] = "Projector";
char projectorScreen[] = "Projector Screen";

// Servo definitions
Servo myServo;                 // create new servo object
static uint8_t servoPin = 23;  // D23

// IR definitions
static uint8_t IRLED = 21; // D21
#define IRPowerA 0xDF00
#define IRPowerC 0x1C
#define IRPowerR 1

// RF definitions
RCSwitch rfSwitch = RCSwitch();
static uint8_t RFTransmitPin = 22;
int screenDownDelay = 27000;

// define the GPIO connected with switches
static uint8_t SwitchPin1 = 13;  //D13

static uint8_t wifiLed = 2;     // D2
static uint8_t gpio_reset = 0;  // Press BOOT for reset WiFi

/* Variable for reading pin status*/
bool latchToggle = LOW;  //Define bool to remember the toggle state of servo arm
bool projectorToggle = LOW;  //Define bool to remember the toggle state of projector
bool projectorScreenToggle = LOW;  //Define bool to remember the toggle state of projector screen

ButtonConfig config1;
AceButton doorSwitch(&config1);

void handleEvent1(AceButton *, uint8_t, uint8_t);

// The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static Switch latchServoSwitch(latchServo, &latchToggle);
static Switch projectorSwitch(projector, &projectorToggle);
static Switch projectorScreenSwitch(projectorScreen, &projectorScreenToggle);

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      #if CONFIG_IDF_TARGET_ESP32
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
            printQR(service_name, pop, "ble");
      #else
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
            printQR(service_name, pop, "softap");
      #endif
            break;
          case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.printf("\nConnected to Wi-Fi!\n");
            digitalWrite(wifiLed, true);
            break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(device_name, latchServo) == 0) 
  {
    Serial.printf("Latch = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) 
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      latchToggle = val.val.b;
      servoPositionSet();
      param->updateAndReport(val);
    }
  
  }
  else if(strcmp(device_name, projector) == 0)
  {
    Serial.printf("Projector = %s\n", val.val.b? "true" : "false");

    if(strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b? "true" : "false", device_name, param_name);
      projectorToggle = val.val.b;
      IRSend();
      param->updateAndReport(val);
    }
  }
  else if(strcmp(device_name, projectorScreen) == 0) {
    Serial.printf("Projector Screen = %s\n", val.val.b? "true" : "false");

    if(strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b? "true" : "false", device_name, param_name);
      projectorScreenToggle = val.val.b;
      if(projectorScreenToggle == false)
      {
        RFSend(1);
      }
      if(projectorScreenToggle == true)
      {
        RFSend(0);
      }

      param->updateAndReport(val);
    }
  }
}

void IRSend()
{
  IrSender.begin(IRLED); // Start with IR_SEND_PIN as send pin and if NO_LED_FEEDBACK_CODE is NOT defined, enable feedback LED at default feedback LED pin
  IrSender.sendNEC(IRPowerA, IRPowerC, IRPowerR);
  delay(1500);
  IrSender.sendNEC(IRPowerA, IRPowerC, IRPowerR); // Send twice following delay so the same function can be used for turning on and off
}

void RFSend(bool direction)
{
  if(direction == 0){
    // lower screen
    rfSwitch.send(2280580, 24);  // Screen down
    Serial.println("Screen Down");
    delay(screenDownDelay);
    // stop screen
    rfSwitch.send(2280584, 24);  // Screen stop
    Serial.println("Screen Stop");
  }

  if(direction == 1){
    // raise screen
    rfSwitch.send(2280578, 24);  // Screen up
    Serial.println("Screen Up");
  }
}

void servoPositionSet()
{
  (latchToggle == false) ? myServo.write(0) : myServo.write(90);  // set servo position
}

void setup()
{
  Serial.begin(115200);

  // Set the GPIOs as output mode
  pinMode(wifiLed, OUTPUT);

  // Configure the input GPIOs
  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(gpio_reset, INPUT);

  // Write to the GPIOs the default state on booting
  digitalWrite(wifiLed, LOW);

  config1.setEventHandler(doorSwitchHandler);

  doorSwitch.init(SwitchPin1);

  Node my_node;
  my_node = RMaker.initNode(nodeName);

  //Standard switch device
  latchServoSwitch.addCb(write_callback);
  projectorSwitch.addCb(write_callback);
  projectorScreenSwitch.addCb(write_callback);

  //Add switch device to the node
  my_node.addDevice(latchServoSwitch);
  my_node.addDevice(projectorSwitch);
  my_node.addDevice(projectorScreenSwitch);

  //This is optional
  RMaker.enableOTA(OTA_USING_PARAMS);
  //If you want to enable scheduling, set time zone for your region using setTimeZone().
  //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
  // RMaker.setTimeZone("Asia/Shanghai");
  // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
  RMaker.enableTZService();
  RMaker.enableSchedule();

  //Service Name
  for (int i = 0; i < 17; i = i + 8) {
    espChipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("\nChip ID:  %d Service Name: %s\n", espChipId, service_name);

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  WiFi.onEvent(sysProvEvent);

  myServo.attach(servoPin);  // attach the servo to output pin
  
  rfSwitch.enableTransmit(RFTransmitPin); // RF TX pin assignment

  #if CONFIG_IDF_TARGET_ESP32
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
  #else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
  #endif

  latchServoSwitch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  projectorSwitch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  projectorScreenSwitch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
}

void loop()
{
  // Read GPIO0 (external button to reset device
  if (digitalRead(gpio_reset) == LOW)  //Push button pressed
  {
    Serial.printf("Reset Button Pressed!\n");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000)
    {
      // If key pressed for more than 10secs, reset all
      Serial.printf("Reset to factory.\n");
      RMakerFactoryReset(2);
    }
    else if ((endTime - startTime) > 3000)
    {
      Serial.printf("Reset Wi-Fi.\n");
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);

  if (WiFi.status() != WL_CONNECTED)
  {
    //Serial.println("WiFi Not Connected");
    digitalWrite(wifiLed, false);
  }
  else
  {
    //Serial.println("WiFi Connected");
    digitalWrite(wifiLed, true);
  }

  doorSwitch.check();
}

void doorSwitchHandler(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
  switch (eventType)
  {
    case AceButton::kEventReleased:  // on button release event
      latchToggle != latchToggle;
      latchServoSwitch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, latchToggle);
      servoPositionSet();
      Serial.println("Door switch pressed");
      Serial.println(latchToggle);
  }
}