/*
 * Gestion des entrée sortie  : Arduino
 */
#include <SoftwareSerial.h> //Included SoftwareSerial Library
//Started SoftwareSerial at RX and TX pin of ESP8266/NodeMCU
#include <Arduino.h>

int pinGX = PC0; //A0
int pinGY = PC1; //A1
int pinDX = PC2; //A2
int pinDY = PC3; //A3

int pinGbtn1 = PB3; // D3
int pinGbtn2 = PD7; // D7
int pinDbtn1 = PD2; // D2
int pinDbtn2 = PB4; // D4

int pinSoftRX = PD6; // D6
int pinSoftTX = PD5; // D5

int stepNb = 10;
unsigned long delayStart = 0; // the time the delay started
bool delayRunning = false;    // true if still waiting for delay to finish

void pinSetup();
int readDigitalValue(int pin, int &rawValue);
int readAnalogValue(int pin, int &rawValue);
void stringifySensors(bool withRaw, String &key, String &jsonData);

String lastSensorsKey = "";
String newSensorsKey = "";
String jsonData;

// enable or disable debug mode
// #define DEBUG ;

SoftwareSerial espSerial(pinSoftRX, pinSoftTX);

void setup()
{
#ifdef DEBUG
    //Serial for debug
    Serial.begin(115200);
    Serial.println("Démarrage de l'arduino");
#endif

    // Setup pins
    pinSetup();

    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    //Serial Begin at 9600 Baud : for exchange with ESP
    espSerial.begin(9600);

#ifdef DEBUG
    Serial.print("Get initial values : '");
#endif

    stringifySensors(false, lastSensorsKey, jsonData);

#ifdef DEBUG
    Serial.println(lastSensorsKey + "'");
    Serial.println("fin de setup");
#endif

    delayStart = millis(); // start delay
    delayRunning = true;   // not finished yet

    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    // check if delay has timed out after 10sec == 10000mS
    if (delayRunning && ((millis() - delayStart) >= 10000))
    {
        delayRunning = false; // prevent this code being run more then once

#ifdef DEBUG
        stringifySensors(true, newSensorsKey, jsonData);
#else
        stringifySensors(false, newSensorsKey, jsonData);
#endif

        if (newSensorsKey != lastSensorsKey)
        {
            lastSensorsKey = newSensorsKey;
            espSerial.write(jsonData.c_str());
#ifdef DEBUG
            Serial.println(jsonData);
#endif
        }

        delayRunning = true; // prevent this code being run more then once
    }
}

String createKey(int data[], int dataSize)
{
    String returnData;

    for (int i = 0; i < dataSize; i++)
    {
        if (i != 0)
            returnData.concat("|");
        returnData.concat(data[i]);
    }

    return "{" + returnData + "}";
}

String createJson(int data[], int dataSize)
{
    String lib[] = {"x1", "y1", "btnA1", "btnA2", "x2", "y2", "btnB1", "btnB2", "raw"};
    String returnData;

    for (int i = 0; i < dataSize; i++)
    {
        if (i != 0)
            returnData.concat(",");
        returnData.concat("\"" + lib[i] + "\":");
        returnData.concat(data[i]);
    }

    return "{" + returnData + "}";
}

void stringifySensors(bool withRaw, String &key, String &jsonData)
{

    int sensors[9], _sensors[8];

    sensors[0] = readAnalogValue(pinGX, _sensors[0]);
    sensors[1] = readAnalogValue(pinGY, _sensors[1]);
    sensors[2] = readDigitalValue(pinGbtn1, _sensors[2]);
    sensors[3] = readDigitalValue(pinGbtn2, _sensors[3]);
    sensors[4] = readAnalogValue(pinDX, _sensors[4]);
    sensors[5] = readAnalogValue(pinDY, _sensors[5]);
    sensors[6] = readDigitalValue(pinDbtn1, _sensors[6]);
    sensors[7] = readDigitalValue(pinDbtn2, _sensors[7]);

    //Create comparaison key
    key = createKey(sensors, 8);
    if (withRaw)
    {
        String rawJson = createJson(_sensors, 8);
        sensors[8] = 999;
        jsonData = createJson(sensors, 9);
        jsonData.replace("999", rawJson);
    }
    else
    {
        jsonData = createJson(sensors, 8);
    }
    jsonData = jsonData + "#" ;
}

int readDigitalValue(int pin, int &rawValue)
{
    rawValue = digitalRead(pin);
    return rawValue;
}

int readAnalogValue(int pin, int &rawValue)
{
    rawValue = analogRead(pin);
    return round(map(constrain(rawValue, 20, 900), 20, 900, 0, stepNb) * (180 / stepNb));
}

void pinSetup()
{
#ifdef DEBUG
    Serial.print("Setup des pins : ");
#endif

    pinMode(pinGX, INPUT);
    pinMode(pinGY, INPUT);
    pinMode(pinDX, INPUT);
    pinMode(pinDY, INPUT);

    pinMode(pinGbtn1, INPUT);
    pinMode(pinGbtn2, INPUT);
    pinMode(pinDbtn1, INPUT);
    pinMode(pinDbtn2, INPUT);

#ifdef DEBUG
    Serial.println("Ok");
#endif
}
