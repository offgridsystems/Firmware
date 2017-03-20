#include <RF24/RF24.h>
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
/*
bool startSession()
{
    if (!client.startSession())
    {
        Serial.println("Start session error");
        return false;
    }

    driver.openReadingPipe(1, client.clientAddress_);
    driver.startListening();
    //driver.printRegisters();

    auto recvStartTime = millis();
    bool isRecvData = false;
    rLen = 32;

    //driver.printDetails();
    while ((millis() - recvStartTime) <= sessionTimeout)
    {
        //driver.printRegisters();
        if (driver.available())
        {
            rLen = driver.getDynamicPayloadSize();
            driver.read(buf, rLen);
            //Serial.print("Received msg ");
            //Serial.println(rLen);
            //Serial.println((char*) buf);

            if (rLen == 2 && strncmp((char*)buf, "S", 1) == 0)
            {
                isRecvData = true;
                break;
            }
        }
    }

    if (!isRecvData)
    {
        Serial.println("Start not received");
        return false;
    }
    driver.stopListening();

    driver.flush_tx();
    driver.openWritingPipe(client.clientAddress_);
    //driver.printDetails();
    if (!driver.write("12345678909876543212345678909871", 32))
    {
        Serial.println("error send packet");
        return false;
    }

    driver.txStandBy();
    isRecvData = false;

    rLen = 32;
    driver.openReadingPipe(1, client.clientAddress_);
    driver.startListening();
    while ((millis() - recvStartTime) <= sessionTimeout)
    {
        if (driver.available())
        {
            isRecvData = true;
            driver.read(buf, 32);
            rLen = driver.getDynamicPayloadSize();
            break;
        }
    }
    driver.stopListening();

    return isRecvData;

}
*/
void loop() {
    // put your main code here, to run repeatedly:
    //driver.setModeRx();

    client.listenBroadcast();
    //delay(10);
    //Serial.println(freeRam());
}
