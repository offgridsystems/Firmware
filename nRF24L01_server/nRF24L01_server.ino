/*
How it works.

1. Server sends command for satrting communication session
for all clients on broadcast channel.

2. Clients stops to listen bradcast channel and switch to listening workChannel
3. Server gets address from internal list of handled clients
and sends request for starting session to the certain client
4. if request is received by client server receives data from client and sends data to client
else server marks this client as unhadled

5. When last client from list will handle it will start to handle unhandled clients
until session timeout

*/


#include <RF24.h>  //standard arduino library, you can install it on library manager
//#include <RF24/RF24.h>  //standard arduino library, you can install it on library manager
#include "Nrf24DcServer.h"

RF24 driver(9, 10);           // init driver D2 - CE pin, D8 CS pin
Nrf24DcServer server(driver);

#define NETWORK_ADDR 0xC7C7C7LL

int sessionTimeout = 3000;  //in milli seconds
int readingTimeout = 10;     //in milli seconds
uint64_t t;

uint8_t key[] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };

void setup() {
    // put your setup code here, to run once:
    //delay(4000);
    Serial.begin(115200);
    Serial.print("Init nrf24 driver - ");
    Serial.println(server.init());       // initialize server and driver
    
    //server.init();
    //server.setRFDataRate(RF24_1MBPS);      // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)
    server.setRFDataRate(RF24_250KBPS);      // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)
    server.setRF_PA_Level(RF24_PA_HIGH);
    server.setWorkChannel(40);           // number of frequency channel for dirrect communicatin between server and client
    //server.setBroadcastChannel(120);     // number of frequency channel for receiving commands from server
    server.setNetworkAddr(NETWORK_ADDR); // Set network address, 3 bytes

    // manual adding clients ID that will be handled by server
    //server.addClient(100);
    //server.addClient(0xc8);

    //server.addDeviceByRange(1, 300);


    // putSendedData(int16_t idx, const void *data, uint8_t len)
    // copy data from <data> to internal buffer 
    // this data will be sended during communication session
    // @parametrs:
    //    idx - index of internal buffer
    //   data - pointer to the buffer
    //    len - size of buffer
    // @note: you can get index for client by calling 
    //         deviceIndexById(id) - where id is id of client
    server.putSendedData(-1, "123", 4); 

    t = millis();

    server.setEcryptKeyPointer(key, sizeof(key) / sizeof(uint8_t));
    server.setEncryption(true);
}

void startSes();
void lookup();

void loop() {

    server.serverLoop();

    if (Serial.available())
    {
        String msg;
        msg = Serial.readString();
        msg.trim();

        if (msg == "l")
            lookup();
    }

    if ((millis() - t) > 5000)
    {
        t = millis();
        Serial.println("Enter <l> in terminal for looking up clients.");
       
        startSes();
    }
}

void startSes() {
    Serial.println("Sending to broadcast request for data ");
    uint64_t  startSingleSessionTime = millis();

    // for starting communication sessions between server and client call server.startSession()
    // this function returns number of handled clients 
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
        for (int i = 0; i < server.handledClientsCount(); ++i)
        {
            Serial.print("Device id - 0x");
            Serial.println(server.clientIdAt(i));
            Serial.println("Received data:");

            for (int j = 0; j < server.lenfgthOfReceivedBufferByIndex(i); ++j)
                Serial.print((char)*(server.receivedDataByIndex(i) + j));

            Serial.println("\n--------------------------------------");
        }
    }

}

void lookup()
{
    Serial.println("Start loking for clients");
    int n = server.lookForClient();
    Serial.print("Found ");
    Serial.print(n);
    Serial.println(" Clients");

    for (int i = 0; i < server.handledClientsCount(); ++i)
    {
        Serial.println(server.clientIdAt(i));
    }

}
