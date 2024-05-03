//
// CombinedBLESerialStep.ino
//
// Stepper motor control via BLE on ESP32
//
// This work may be reproduced, modified, distributed,
// performed, and displayed for any purpose, but must
// acknowledge Neil Gershenfeld, Quentin Bolsee and this project.
// Copyright is retained and must be preserved.
// The work is provided as is; no warranty is provided, and users accept all liability.
//

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define stepper motor control pins
#define DIR 2
#define STEP 3
#define MS1 0
#define MS2 7
#define MS3 6

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Variable to store the received BLE command
char bleCommand = 0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);
        Serial.println();
        Serial.println("*********");

        // Store the first character of the received value as command
        bleCommand = value[0];
      }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  digitalWrite(STEP, LOW);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, LOW);

  // 1/16 step
  pinMode(MS1, OUTPUT);
  digitalWrite(MS1, HIGH);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS2, HIGH);
  pinMode(MS3, OUTPUT);
  digitalWrite(MS3, HIGH);

  BLEDevice::init("MyESP32_1");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Hello World");
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {
  // Check if a command was received via BLE
  if (bleCommand != 0) {
    if (bleCommand == 'f') {
      digitalWrite(DIR, HIGH);
    } else if (bleCommand == 'r') {
      digitalWrite(DIR, LOW);
    }
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP, LOW);
    bleCommand = 0; // Reset command to avoid repeating the last command
  }
  delay(10); // Short delay to prevent overheating
}
