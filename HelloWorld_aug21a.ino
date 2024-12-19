#include "arduino_secrets.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "thingProperties.h"
#include <Arduino_MKRIoTCarrier.h>

MKRIoTCarrier carrier;

const int analogInPin = A0;     // pH sensor
const int relay1Pin = 1;        // D1
const int relay2Pin = 2;        // D2
const int relay3Pin = 3;        // D3
const int relay4Pin = 4;        // D4
const int oneWireBus = 5;       // D5 Temperature sensor
const int TdsSensorPin = A1;    // TDS sensor 
const int TurbiditySensorPin = A2; // Turbidity sensor

// ตั้งค่าค่าคงที่ TDS sensor
const float VREF = 3.3; 
const int SCOUNT = 30;
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

// ตั้งค่าตัวแปร pH
int buf[10];
int tempBuffer; 

// ตั้งค่า Temperature sensor
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


void setup() {
  Serial.begin(9600); 
  carrier.begin();

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  pinMode(TurbiditySensorPin, INPUT);

  digitalWrite(relay1Pin, LOW); 
  digitalWrite(relay2Pin, LOW); 
  digitalWrite(relay3Pin, LOW); 
  digitalWrite(relay4Pin, LOW); 

  Serial.println("Start Project Crab");
  sensors.begin();
  
  initProperties();

  // เชื่อมต่อ Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  delay(5000);
  // pH Sensor
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(analogInPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        tempBuffer = buf[i];
        buf[i] = buf[j];
        buf[j] = tempBuffer;
      }
    }
  }
  unsigned long int avgValue = 0;
  for (int i = 2; i < 8; i++)
    avgValue += buf[i];
  float pHVol = (float)avgValue * 3.3 / 1024 / 6;
  ph = -5.70 * pHVol + 21.34;
  Serial.print("pH Value: ");
  Serial.println(ph);

  // ควบคุมการเปิดปิด relay ปั๊มน้ำจากค่า PH
  if (ph < 7.0) {
    digitalWrite(relay1Pin, HIGH);
    notification = ("น้ำมีความเป็นกรดมากเกินไป กำลังใส่ปูนขาว");
  } else if (ph >= 7.8) {
    digitalWrite(relay1Pin, LOW);
    notification = ("ค่ากรด-เบสของน้ำกลับมาเป็นปกติแล้ว");

  }

  // Temperature Sensor
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.println(temp);

  // ควบคุมการเปิดปิด relay ทำความร้อนจากค่าอุณหภูมิ
  if (temp <= 28.0) {
    digitalWrite(relay2Pin, HIGH);
    notification = ("อุณหภูมิต่ำกว่าจุดที่กำหนด เปิดใช้เครื่องทำความร้อน(Heater)");
  } else if (temp >= 29.0) {
    digitalWrite(relay2Pin, LOW);
  }

  // ควบคุมการเปิดปิด relay ทำความเย็นจากค่าอุณหภูมิ
  if (temp >= 30.0) {
    digitalWrite(relay3Pin, HIGH); 
    digitalWrite(relay4Pin, HIGH);
    notification = ("อุณหภูมิเกินกว่าจุดที่กำหนด เปิดใช้เครื่องทำความเย็น(Cooler)");

  } else if (temp <= 29.0) {
    digitalWrite(relay3Pin, LOW);
    digitalWrite(relay4Pin, LOW);
  }

  // TDS Sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  // คำนวณค่า TDS
  averageVoltage = 0;
  for (int i = 0; i < SCOUNT; i++) {
    averageVoltage += analogBuffer[i];
  }
  averageVoltage = averageVoltage / SCOUNT * VREF / 1024.0;

  float compensationCoefficient = 1.0 + 0.02 * (25.0 - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
  Serial.print("TDS Value: ");
  Serial.print(tds);
  Serial.println(" ppm");

  if (tds < 1000) {
    notification = "ค่าTDSต่ำกว่าจุดที่กำหนดไว้ คุณควรที่จะเปลี่ยนน้ำทันที";
  } else if (tds > 3000) {
    notification = "ค่าTDSเกินกว่าจุดที่กำหนดไว้ คุณควรที่จะเปลี่ยนน้ำทันที";
  } else if ((tds >= 1000 && tds <= 1300) || (tds >= 2700 && tds <= 3000)) {
    notification = "ค่าTDSกำลังอยู่ในจุดเสี่ยง คุณควรที่จะเปลี่ยนน้ำโดยเร็ว";
  }

  // Turbidity Sensor
  int turbiditySensorValue = analogRead(TurbiditySensorPin);
  float ntu = turbiditySensorValue;
  turbidity = ntu;
  Serial.print("Turbidity Sensor : ");
  Serial.print(turbidity);
  Serial.println(" NTU");
  if (turbidity < 10) {
    notification = "ค่าความขุ่นต่ำกว่าจุดที่กำหนดไว้ คุณควรที่จะเปลี่ยนน้ำทันที";
  } else if (turbidity > 30) {
    notification = "ค่าความขุ่นเกินกว่าเกินกว่าจุดที่กำหนดไว้ คุณควรที่จะเปลี่ยนน้ำทันที";
  } else if ((turbidity >= 10 && turbidity <= 13) || (turbidity >= 27 && turbidity <= 30)) {
    notification = "ค่าความขุ่นกำลังอยู่ในจุดเสี่ยง คุณควรที่จะเปลี่ยนน้ำโดยเร็ว";
  }

  delay(5000);
}

void onPhChange() {
  Serial.print("pH value updated: ");
  Serial.println(ph);
}

void onTempChange() {
  Serial.print("Temperature updated: ");
  Serial.println(temp);
}

void onTdsChange() {
  Serial.print("TDS value updated: ");
  Serial.println(tds);
}

void onTurbidityChange() {
  Serial.print("Turbidity value updated: ");
  Serial.println(turbidity);
}
void onNotificationChange() {
  Serial.print("Notification value updated: ");
  Serial.println(notification);
}
