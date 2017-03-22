#include <RF24/RF24.h>   //standard arduino library, you can install it on library manager
#include "Nrf24DcClient.h"
//#include <printf.h>
#include "initNrf24DcClient.h"

#define NETWORK_ADDR 0xC7C7C7LL


RF24 driver(D2, D8);
Nrf24DcClient client(driver);

uint8_t buf[33] = { 0 };
uint8_t rLen;


void setup() {

    Serial.begin(9600);
    initDcClient(client);   // initalize Nrf24DcClient
}

void loop() {

    uint8_t res = client.listenBroadcast();  // returns not 0 if any command was received from server

    // 
    if (res != 0x00)
    {
        Serial.print("Command code : ");
        Serial.println(res);

        if (client.getReceivedData(nullptr, 32))   // if received data size not 0 then
        {
            uint8_t buf[32];
            int8_t len = client.getReceivedData(buf, 32);  // copy received data to <buff>, len - size of received data
            Serial.println("Received data:");
            Serial.print("Size - ");
            Serial.print(len);
            Serial.println(" bytes");
            for (int i = 0; i < len; ++i)
            {
                Serial.print(buf[i], HEX);
                Serial.print("(");
                Serial.print((char)buf[i]);
                Serial.print("), ");
            }
        }
    }
}
