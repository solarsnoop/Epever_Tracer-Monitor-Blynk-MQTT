#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <BlynkSimpleEsp8266.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#define WIFI_SSID             "namewifi"
#define WIFI_PASS             "passwordwifi"
#define AUTH                  "yourBLYNKTOKE"   
#define output1 D0
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
/*
   Virtual Pins - Base. (For Blynk)
*/
#define vPIN_restart                    V15
#define vPIN_PV_POWER                   V1
#define vPIN_PV_VOLTAGE                 V3
#define vPIN_BATT_TEMP                  V6
#define vPIN_BATTERY_CHARGE_POWER       V11
const char* mqtt_server = "IPADRESSYOURMQQTBROKER";
const char* mqttUser = "";
const char* mqttPassword = "";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
const int defaultBaudRate = 115200;
int timerTask1, timerTask2, timerTask3;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;
int restartvar = 1;
int restartesp = 0;
#define MAX485_DE D1
#define MAX485_RE_NEG D2

ModbusMaster node;
BlynkTimer timer;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

// A list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_3106,
  AddressRegistry_311A,
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber() {
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries)) {
    currentRegistryNumber = 0;
  }
}
BLYNK_WRITE(V20)
{
  int pinValue = param.asInt();
  if (pinValue==1){
     restartesp = 1;
     }
  if (pinValue==0){
     restartesp = 0;
     }
}
// ****************************************************************************

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("epevertracer/tracer1/outTopic/", "Tracer1");
      // ... and resubscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  // configure GPIO 0 as outputs:
  pinMode(output1, OUTPUT);
  digitalWrite(output1, HIGH);
  Serial.begin(defaultBaudRate);

  // Modbus slave ID 1
  node.begin(1, Serial);

  // callbacks to toggle DE + RE on MAX485
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("Connecting to Wifi...");
  
  WiFi.mode(WIFI_STA);
  
  Blynk.begin(AUTH, WIFI_SSID, WIFI_PASS, IPAddress(192,168,178,2), 8080);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(60000);
    ESP.restart();
  }
  
  Serial.println("Connected.");
  Serial.print("Connecting to Blynk...");
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("Tracer1", mqttUser, mqttPassword )) {
           Serial.println("connected");  
           } else {
           delay(2000);
           }
  }
  client.subscribe("epevertracer/tracer");
  while (!Blynk.connect()) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();
  Serial.println("Connected to Blynk.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting timed actions...");
  timerTask1 = timer.setInterval(5000L, executeCurrentRegistryFunction);
  timerTask2 = timer.setInterval(5000L, nextRegistryNumber);
  timerTask3 = timer.setInterval(5000L, uploadToBlynk);
  Serial.println("Setup OK!");
  Serial.println("----------------------------");
  Serial.println();
}

// --------------------------------------------------------------------------------
  
  // upload values
  void uploadToBlynk() {
    Blynk.virtualWrite(vPIN_PV_POWER,                   pvpower);
    Blynk.virtualWrite(vPIN_PV_VOLTAGE,                 pvvoltage);
    Blynk.virtualWrite(vPIN_BATT_TEMP,                  btemp);
    Blynk.virtualWrite(vPIN_BATTERY_CHARGE_POWER,       battChargePower);
    Blynk.virtualWrite(vPIN_restart,                    restartvar);
    client.publish("epevertracer/tracer1/pvpower", String(pvpower).c_str());
    client.publish("epevertracer/tracer1/pvvolt", String(pvvoltage).c_str());
    client.publish("epevertracer/tracer1/batpower", String(battChargePower).c_str());
    client.publish("epevertracer/tracer1/battemp", String(btemp).c_str());
    client.publish("epevertracer/tracer1/outTopic", "Tracer1");
  }
  
  // exec a function of registry read (cycles between different addresses)
  void executeCurrentRegistryFunction() {
    Registries[currentRegistryNumber]();
  }
  
  // -----------------------------------------------------------------

  void AddressRegistry_3100() {
    result = node.readInputRegisters(0x3100, 6);
  
    if (result == node.ku8MBSuccess) {
      
      pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("PV Voltage: ");
      Serial.println(pvvoltage);
  
      pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
      Serial.print("PV Power: ");
      Serial.println(pvpower);
      
    }
  }

  void AddressRegistry_3106()
  {
    result = node.readInputRegisters(0x3106, 2);

    if (result == node.ku8MBSuccess) {
      battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
      Serial.print("Battery Charge Power: ");
      Serial.println(battChargePower);
    }
  }

  void AddressRegistry_311A() 
  {
    result = node.readInputRegisters(0x311A, 2);
    
    if (result == node.ku8MBSuccess) {
      btemp = node.getResponseBuffer(0x01) / 100.0f;
      Serial.print("Battery Temperature: ");
      Serial.println(btemp);
    } else {
      rs485DataReceived = false;
      Serial.println("Read register 0x311A failed!");
    }
  }


void loop()
{
  if (Blynk.connected()) {
      Blynk.run();
    } 
  if (restartvar >0)    {
  Blynk.virtualWrite(vPIN_restart, restartvar);
  restartvar = 0;
  delay(5000);
  Serial.println("Restart finish");
  Blynk.virtualWrite(vPIN_restart, restartvar);
  }
  if (!Blynk.connected()) {
      Serial.println("Restart blynk disconnected");
      ESP.restart();
  }
  if (restartesp==1){
      Serial.println("Restart blynk disconnected");
      ESP.restart();
     }
  timer.run();
}
