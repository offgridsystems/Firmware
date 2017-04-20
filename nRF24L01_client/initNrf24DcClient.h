#ifndef INITNRF24DCCLIENT_H
#define INITNRF24DCCLIENT_H

#define NETWORK_ADDR 0xC7C7C7LL

#include "Nrf24DcClient.h"
#include "StartSessionCmd.h"
#include "LookupCmd.h"
#include "KeepaliveCmd.h"

uint8_t key[] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };


bool initDcClient(Nrf24DcClient &client) 
{
    Serial.print(F("Init nrf24 driver - "));
    //Serial.flush();
    Serial.println(client.init());           // initialize client and driver

    //client.setRFDataRate(RF24_1MBPS);   // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)
    client.setRFDataRate(RF24_250KBPS);      // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)
    client.setRF_PA_Level(RF24_PA_HIGH);

    client.setWorkChannel(40);               // number of frequency channel for dirrect communicatin between server and client
    //client.setBroadcastChannel(120);         // number of frequency channel for receiving commands from server
    client.setNetworkAddr(NETWORK_ADDR);     // Set network address, 3 bytes
    client.setDeviceId(1024);                // unique in same network ID
    client.setSessionTimeout(3000);          // time while client can comms with server in one session, after this time client aborts session

    // Adds handler for communication command
    // You also can create and add own commands, for help - read commets in AbstarctClientCommand.h
    client.addCommand(new StartSessionCmd(&client));
    client.addCommand(new LookupCmd(&client));
    client.addCommand(new KeepaliveCmd(&client));

    // copy data for sending to the server during communication session
    // you can call this function anywhere
    client.putDataForSend((void*)"1234567890123456789012345678901234", 32);

    client.setEcryptKeyPointer(key, sizeof(key) / sizeof(uint8_t));
    client.setEncryption(true);

    return true;
}

#endif // ! INITNRF24DCCLIENT_H

