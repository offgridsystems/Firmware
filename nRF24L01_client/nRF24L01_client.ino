#include <RF24/RF24.h>   //standard arduino library, you can install it on library manager
#include "Nrf24DcClient.h"
//#include <printf.h>
#include "initNrf24DcClient.h"

#define NETWORK_ADDR 0xC7C7C7LL


RF24 driver(5, 6);
Nrf24DcClient client(driver);
StartSessionCmd ssCmd(&client);

uint8_t buf[33] = { 0 };
uint8_t rLen;

//int sessionTimeout = 3000;  //in milli seconds

int freeRam() {
//    extern int __heap_start, *__brkval;
//    int v;
//    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup() {
    // put your setup code here, to run once:

    Serial.begin(9600);
    //printf_begin();
    //delay(1000);
    initDcClient(client);

    Serial.print(F("Starts setup section "));
    Serial.println(freeRam());

}

void loop() {
    uint8_t res = client.listenBroadcast();
    if (res != 0x00)
    {
        Serial.print("Command code : ");
        Serial.println(res);
        if (client.getReceivedData(nullptr, 32)) 
        {
            uint8_t buf[32];
            int8_t len = client.getReceivedData(buf, 32);
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
