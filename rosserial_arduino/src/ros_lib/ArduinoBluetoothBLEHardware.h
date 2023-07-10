/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2023, Kei Okada
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of copyright holder nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Copied from esp32/hardware/esp32/2.0.9/libraries/BLE/examples/BLE_uart/BLE_uart.ino

#ifndef ROS_ARDUINO_BLUETOOTH_BLE_HARDWARE_H_
#define ROS_ARDUINO_BLUETOOTH_BLE_HARDWARE_H_

#if not defined(ESP32)
  #error ARDUINO BLUETOOTH HARDWARE CURRENTLY SUPPORT ONLY ESP32
#endif

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

#include <RingBuf.h>  // Need RingBuffer by Jean-Luc Locoduino

RingBuf<int, 128> rx_buffer;
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    // Serial.print("onWrite  : ");
    // Serial.println(rxValue.length());
    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        rx_buffer.push(rxValue[i]);
      }
    }
  }
};

class ArduinoHardware {
public:
  ArduinoHardware()
  {
  }

  void init()
  {
    init("rosserial BLE UART Service");
  }

  void init(char *name)
  {
    // Create the BLE Device
    BLEDevice::init(name);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                                       CHARACTERISTIC_UUID_TX,
                                                       BLECharacteristic::PROPERTY_NOTIFY |
                                                       BLECharacteristic::PROPERTY_READ
                                                       );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                                                           CHARACTERISTIC_UUID_RX,
                                                                           BLECharacteristic::PROPERTY_WRITE
                                                                           );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    //Serial.println("Waiting a client connection to notify...");
  }

  int read(){
    int data;
    // Serial.print("read  : ");
    // Serial.println(rx_buffer.size());
    if (rx_buffer.lockedPop(data)) {
      return data;
    }
    return -1;
  }

  void write(uint8_t* data, int length)
  {
    // Serial.print("write : ");
    // Serial.println(length);
    // Serial.println(ESP_GATT_MAX_ATTR_LEN);
    size_t max_length = ESP_GATT_MAX_ATTR_LEN-100;
    if (deviceConnected) {
      for(size_t i = 0; i < length; i += max_length) {
        // Serial.print("i = ");
        // Serial.print(i);
        // Serial.print(" / ");
        // Serial.println(min(length-i, max_length));
        pTxCharacteristic->setValue(data+i, (size_t)(min(length-i, max_length)));
        pTxCharacteristic->notify();
        delay(10); // bluetooth stack will go into congestion, if too many packets are sent
      }
      delay(20);
    }
    loop();
  }

  void loop() {
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
  }

  unsigned long time()
  {
    return millis();
  }


protected:
  int rxvalue_length = 0;

  BLEServer *pServer = NULL;
  BLECharacteristic * pTxCharacteristic;
  bool oldDeviceConnected = false;
  uint8_t txValue = 0;
};

#endif
