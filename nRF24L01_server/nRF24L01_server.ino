#include <RF24/RF24.h>  //standard arduino library, you can install it on library manager
#include "Nrf24DcServer.h"

RF24 driver(D2, D8);
Nrf24DcServer server(driver);

#define NETWORK_ADDR 0xC7C7C7LL

int sessionTimeout = 3000;  //in milli seconds
int readingTimeout = 2;     //in milli seconds
void setup() {
    // put your setup code here, to run once:

    Serial.begin(9600);
    Serial.print("Init nrf24 driver - ");
    Serial.println(server.init());

    driver.setAddressWidth(5);
    driver.setAutoAck(false);
    driver.setRetries(1, 15);
    driver.setCRCLength(RF24_CRC_8);
    driver.enableDynamicPayloads();
    driver.setDataRate(RF24_1MBPS);

    server.setWorkChannel(10);
    server.setBroadcastChannel(120);
    server.setNetworkAddr(NETWORK_ADDR); 

    // client ID that will be handled by server
    server.addDevice(0xc7);   

    server.putSendedData(0, "123", 4); // this data will be sended during communication session
    //server.addDeviceByRange(1, 300);
}

void loop() {
    yield();
    Serial.println("Sending to broadcast request for data");
    yield();
    uint64_t  startSingleSessionTime = millis();
    int numberOfHandledDevices = server.startSession();
    yield();
    uint32_t finishTime = millis() - startSingleSessionTime;
    Serial.print("Session duration ");
    Serial.print(finishTime);
    Serial.println("mSec");
    Serial.print("Number of handled devices = ");
    Serial.println(numberOfHandledDevices);

    if (numberOfHandledDevices)
    {
        for (int i = 0; i < server.handledDeviceCount(); ++i)
        {
            Serial.print("Device id - 0x");
            Serial.println(server.deviceIdAt(0), HEX);
            Serial.println("Received data:");

            for (int j = 0; j < server.lenfgthOfReceivedBufferByIndex(i); ++j)
                Serial.print((char)*(server.receivedDataByIndex(i)+j));

            Serial.println("\n--------------------------------------");
        }
    }

    delay(5000);
    yield();
}
