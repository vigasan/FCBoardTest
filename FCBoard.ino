/**************************************************************************************************************************************************
* File name     : FCBoardTest.c
* Compiler      : 
* Autor         : VIGASAN   
* Created       : 27/03/2023
* Modified      : 
* Last modified :
*
*
* Description   : 
*
* Other info    : 
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <OneWire.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // You need to change the value of costant MQTT_MAX_PACKET_SIZE to 600 in the file PubSubClient.h 
                          // because the MQTT payload could exceed standard value of 256

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------  Constants  ----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#define UNPRESSED                       0
#define DEBOUNCE                        1
#define PRESSED                         2
#define RELAY_OFF                       0
#define RELAY_ON                        1

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const int IN_DIN1 = 12;
const int IN_DIN2 = 13;
const int RELAY1 = 33;
const int RELAY2 = 27;
const int LED_R = 32;
const int LED_G = 35;
const int LED_B = 34;
const int LED_1 = 21;
const int LED_2 = 19;
const int LED_3 = 18;
const int DAC_1 = 26;
const int DAC_2 = 25;
const int DS18S20_PIN = 23;

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT DISCOVERY PARAMETERS----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*     g_ssid = "your Wifi Name";                              // Wifi Name
const char*     g_password = "Wifi Password";                           // Wifi Password
const char*     g_mqtt_server = "192.168.1.25";                         // MQTT Server IP, same of Home Assistant
const char*     g_mqttUser = "mqttUser";                                // MQTT Server User Name
const char*     g_mqttPsw = "password";                                 // MQTT Server password
int             g_mqttPort = 1883;                                      // MQTT Server Port
const char*     g_mqtt_DeviceName = "FCTest";                           // MQTT Device Name

// Variable used for MQTT Discovery
const char*	    g_deviceModel = "FCBoard";                            	// Hardware Model
const char*     g_swVersion = "1.0";                                    // Firmware Version
const char*		g_manufacturer = "Vigasan";                             // Manufacturer Name
String			g_deviceName = "FCTest";                          		// Device Name

String      	g_TopicRelay1 = "FCTest/relay1";       			// Topic for entity relay1
String      	g_TopicRelay2 = "FCTest/relay2";       			// Topic for entity relay2
String      	g_TopicAOut1 = "FCTest/aout1";       			// Topic for entity analog out1
String      	g_TopicAOut2 = "FCTest/aout2";       			// Topic for entity analog out1
String      	g_TopicProbeTemperature = "FCTest/temp";		// Topic for entity temperature

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Global variables-------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
OneWire 			g_tempSensor(DS18S20_PIN);

unsigned long       g_TimeInputs = 0;
unsigned long       g_TimeLed = 0;
unsigned long       g_TimeTemperature = 0;
int                 g_mqttCounterConn = 0;
String              g_UniqueId;
bool                g_InitSystem = true;
int                 g_canPublish = 0;

byte                g_RelayStatus1 = RELAY_OFF;
byte                g_RelayStatus2 = RELAY_OFF;
byte                g_Input1 = 0;
byte                g_Input2 = 0;
byte				g_AnOut1 = 0;
byte				g_AnOut2 = 0;
float               g_ProbeTemperature = 0.0;

byte                g_st_input1 = UNPRESSED;
byte                g_st_input2 = UNPRESSED;


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // I/O Configuration
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pinMode(IN_DIN1, INPUT);
    pinMode(IN_DIN2, INPUT);
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);


    dacWrite(DAC_1, g_AnOut1);  // Set DAC output to 0
    dacWrite(DAC_2, g_AnOut2);  // Set DAC output to 0


    Serial.begin(115200);
    delay(500); 
	
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(g_deviceModel);
    Serial.print("DEVICE: ");
    Serial.println(g_deviceName);
    Serial.print("SW Rev: ");
    Serial.println(g_swVersion);
    Serial.println("----------------------------------------------");
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Wifi Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Configuration
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Discovery Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(g_InitSystem)
    {
        delay(1000);
        g_InitSystem = false;
		    Serial.println("INIT SYSTEM...");
        MqttHomeAssistantDiscovery();    
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Inputs Monitor
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeInputs > 200)
    {
        g_TimeInputs = millis();

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Inputs 1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch(g_st_input1)
        {
            case UNPRESSED:
            {
                if(digitalRead(IN_DIN1) == 1)            
                {
                    g_st_input1 = DEBOUNCE;
                }
            } break;
    
            case DEBOUNCE:
            {
                if(digitalRead(IN_DIN1) == 1)           
                {
                    g_st_input1 = PRESSED;
                    g_Input1 = 1;   

                    if(g_RelayStatus2 == RELAY_ON)
                    {
                      g_RelayStatus2 = RELAY_OFF;
                      digitalWrite(RELAY2, g_RelayStatus2);
                    } else
                    {
                      g_RelayStatus2 = RELAY_ON;
                      digitalWrite(RELAY2, g_RelayStatus2);
                    }
                    MqttPublishStatus_Relay2();
                    
                } else                                  
                {
                    g_st_input1 = UNPRESSED;
                    g_Input1 = 0;                       
                }
            } break;
    
            case PRESSED:
            {
                if(digitalRead(IN_DIN1) == 0)
                {
                    g_st_input1 = DEBOUNCE;
                }
            } break;
        }
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Inputs 2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch(g_st_input2)
        {
            case UNPRESSED:
            {
                if(digitalRead(IN_DIN2) == 1)            
                {
                    g_st_input2 = DEBOUNCE;
                }
            } break;
    
            case DEBOUNCE:
            {
                if(digitalRead(IN_DIN2) == 1)           
                {
                    g_st_input2 = PRESSED;
                    g_Input2 = 1;   
                    
                    if(g_RelayStatus1 == RELAY_ON)
                    {
                      g_RelayStatus1 = RELAY_OFF;
                      digitalWrite(RELAY1, g_RelayStatus1);
                    } else
                    {
                      g_RelayStatus1 = RELAY_ON;
                      digitalWrite(RELAY1, g_RelayStatus1);
                    }
                    MqttPublishStatus_Relay1();
                } else                                  
                {
                    g_st_input2 = UNPRESSED;
                    g_Input2 = 0;                       
                }
            } break;
    
            case PRESSED:
            {
                if(digitalRead(IN_DIN2) == 0)
                {
                    g_st_input2 = DEBOUNCE;
                }
            } break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Status Led
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeLed > 500)
    {
        g_TimeLed = millis();
        digitalWrite(LED_R, !digitalRead(LED_R));
        digitalWrite(LED_1, !digitalRead(LED_1));
        digitalWrite(LED_2, !digitalRead(LED_2));
        digitalWrite(LED_3, !digitalRead(LED_3));
    }
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Reading Probe Temperature
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeTemperature > 5000)
    {
        g_TimeTemperature = millis();
        g_ProbeTemperature = GetTemperature();
        MqttPublishTemperature();
    }
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);


    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) + String(mac[1],HEX) + String(mac[2],HEX) + String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX); 
    
    while(WiFi.status() != WL_CONNECTED && counter++ < 5) 
    {
        delay(500);
    }

}

void MqttReconnect() 
{
    // Loop until we're MqttReconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        if (g_mqttPubSub.connect(g_mqtt_DeviceName, g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");
            //ESP32 Subscribe following topics
            // Home assistant status
            g_mqttPubSub.subscribe("homeassistant/status");
            g_mqttPubSub.subscribe((g_TopicRelay1 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicRelay2 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicAOut1 + "/set").c_str());
            g_mqttPubSub.subscribe((g_TopicAOut2 + "/set").c_str());
            delay(500);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 3 seconds");
            delay(3000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;
    int uniqueId_increment = 0;
    if(g_mqttPubSub.connected())
    {
        StaticJsonDocument<600> payload;
        JsonArray modes;
        JsonObject device;
        JsonArray identifiers;

        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay 1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////     
		discoveryTopic = "homeassistant/switch/" + g_deviceName + "_relay1/config"; // Discovery Topic for entity relay1

        payload["name"] = g_deviceName + ".relay1";            
        payload["uniq_id"] = g_UniqueId + "_relay1";                                                          
        payload["stat_t"] = g_TopicRelay1 + "/state";    
        payload["cmd_t"] = g_TopicRelay1 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Relay 2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/switch/" + g_deviceName + "_relay2/config"; // Discovery Topic for entity relay2
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = g_deviceName + ".relay2";                 
        payload["uniq_id"] = g_UniqueId + "_relay2";                                                          
        payload["stat_t"] = g_TopicRelay2 + "/state";    
        payload["cmd_t"] = g_TopicRelay2 + "/set"; 
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Out 1
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
		discoveryTopic = "homeassistant/number/" + g_deviceName + "_aout1/config"; // Discovery Topic for analog out 1
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = g_deviceName + ".aout1";                
        payload["uniq_id"] = g_UniqueId + "_aout1";                                                          
        payload["stat_t"] = g_TopicAOut1 + "/state";    
        payload["cmd_t"] = g_TopicAOut1 + "/set"; 
		payload["min"] = "0";
		payload["max"] = "255";
		payload["step"] = "1";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Analog Out 2
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/number/" + g_deviceName + "_aout2/config"; // Discovery Topic for analog out 2
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        payload["name"] = g_deviceName + ".aout2";                 
        payload["uniq_id"] = g_UniqueId + "_aout2";                                                          
        payload["stat_t"] = g_TopicAOut2 + "/state";    
        payload["cmd_t"] = g_TopicAOut2 + "/set"; 
		payload["min"] = "0";
		payload["max"] = "255";
		payload["step"] = "1";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(100);
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Probe Temperature
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/" + g_deviceName + "_temp/config"; // Discovery Topic for Probe Temperature
        payload.clear();
        modes.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();
        
        uniqueId_increment++;
        payload["name"] = g_deviceName + ".temp";      
        payload["uniq_id"] = g_UniqueId + "_" + String(uniqueId_increment);
        payload["stat_t"] = g_TopicProbeTemperature;
        payload["dev_cla"] = "temperature";
        payload["unit_of_meas"] = "°C";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);
        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(500);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Send Current status of Relays and Inputs 
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        MqttPublishStatus_Relay1();
        delay(100);
        MqttPublishStatus_Relay2();
        delay(100);
		MqttPublishStatus_AOut1();
        delay(100);
		MqttPublishStatus_AOut2();
        delay(100);
    }
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    byte state = 0;
    String payload;
    String topicMsg;
    StaticJsonDocument<256> doc;
    
    for (int i = 0; i < length; i++) 
    {
        payload += (char)inFrame[i];
    }


    if(String(topic) == String("homeassistant/status")) 
    {
        if(payload == "online")           // When home Assistant restart send "online" message
        {
            MqttHomeAssistantDiscovery(); // So device send Discovery Data
        }
    }
	
	if(String(topic) == String(g_TopicRelay1 + "/set")) // MQTT message to set Relay1 
    {
        if(payload == "ON") 
            g_RelayStatus1 = RELAY_ON;
        else if(payload == "OFF") 
            g_RelayStatus1 = RELAY_OFF;
        digitalWrite(RELAY1, g_RelayStatus1);
        MqttPublishStatus_Relay1();
    }

    if(String(topic) == String(g_TopicRelay2 + "/set")) // MQTT message to set Relay2 
    {
        if(payload == "ON") 
            g_RelayStatus2 = RELAY_ON;
        else if(payload == "OFF") 
            g_RelayStatus2 = RELAY_OFF;
        digitalWrite(RELAY2, g_RelayStatus2);
        MqttPublishStatus_Relay2();
    }
	
	if(String(topic) == String(g_TopicAOut1 + "/set")) // MQTT message to set Analog Out 1 
    {
        g_AnOut1 = payload.toInt();
        dacWrite(DAC_1, g_AnOut1);
        MqttPublishStatus_AOut1();
    }
	
	if(String(topic) == String(g_TopicAOut2 + "/set")) // MQTT message to set Analog Out 2 
    {
        g_AnOut2 = payload.toInt();
        dacWrite(DAC_2, g_AnOut2);
        MqttPublishStatus_AOut2();
    }
}

void MqttPublishStatus_Relay1() // Sens MQTT status for Relay 1
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_RelayStatus1 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay1 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_Relay2() // Sens MQTT status for Relay 2
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        if(g_RelayStatus2 == 0)
            strPayload = "OFF";
        else
            strPayload = "ON";
        
        g_mqttPubSub.publish((g_TopicRelay2 + "/state").c_str(), strPayload.c_str());
    }
}

void MqttPublishStatus_AOut1()  // Sens MQTT status for Analog Out 1
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        g_mqttPubSub.publish((g_TopicAOut1 + "/state").c_str(), String(g_AnOut1).c_str());
    }
}

void MqttPublishStatus_AOut2()  // Sens MQTT status for Analog Out 2
{
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        g_mqttPubSub.publish((g_TopicAOut2 + "/state").c_str(), String(g_AnOut2).c_str());
    }
}

void MqttPublishTemperature() // Sens MQTT status for Probe Temperature
{
    String topicMsg;
    String payload;
    if(g_mqttPubSub.connected())
    {
        payload = g_ProbeTemperature;
        topicMsg = g_TopicProbeTemperature;
        g_mqttPubSub.publish(topicMsg.c_str(), payload.c_str());
    }
}

float GetTemperature()  // Get Temperature from DS18S20 probe
{
    byte data[12];
    byte addr[8];
  
    if ( !g_tempSensor.search(addr)) {
        //no more sensors on chain, reset search
        g_tempSensor.reset_search();
        return -1000;
    }
  
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        return -1000;
    }
  
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
        return -1000;
    }
  
    g_tempSensor.reset();
    g_tempSensor.select(addr);
    g_tempSensor.write(0x44,1); // start conversion, with parasite power on at the end
  
    byte present = g_tempSensor.reset();
    g_tempSensor.select(addr);
    g_tempSensor.write(0xBE); // Read Scratchpad
  
  
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = g_tempSensor.read();
    }
  
    g_tempSensor.reset_search();
  
    byte MSB = data[1];
    byte LSB = data[0];
  
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    float TemperatureSum = tempRead / 16;
  

    return TemperatureSum;
}
