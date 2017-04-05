#ifndef INITNRF24DCCLIENT_H
#define INITNRF24DCCLIENT_H

#define NETWORK_ADDR 0xC7C7C7LL

#include "Nrf24DcClient.h"
#include "StartSessionCmd.h"
#include "LookupCmd.h"
#include "KeepaliveCmd.h"

bool initDcClient(Nrf24DcClient &client) 
{
    Serial.print(F("Init nrf24 driver - "));
    //Serial.flush();
    Serial.println(client.init());           // initialize client and driver
    client.driver.setAddressWidth(5);        // must be always 5
    client.driver.setRetries(1, 15);         // set 15 retries and 500uSec between retransmition in autoACK mode.
    client.driver.setCRCLength(RF24_CRC_8);  // number of bits of CRC, can be RF24_CRC_8 or RF24_CRC_16
    client.driver.enableDynamicPayloads();   // alway use this options
    client.driver.setDataRate(RF24_1MBPS);   // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)

    client.setWorkChannel(66);               // number of frequency channel for dirrect communicatin between server and client
    //client.setBroadcastChannel(120);         // number of frequency channel for receiving commands from server
    client.setNetworkAddr(NETWORK_ADDR);     // Set network address, 3 bytes
    client.setDeviceId(30);                // unique in same network ID
    client.setSessionTimeout(3000);          // time while client can comms with server in one session, after this time client aborts session

    // Adds handler for communication command
    // You also can create and add own commands, for help - read commets in AbstarctClientCommand.h
    client.addCommand(new StartSessionCmd(&client));
    client.addCommand(new LookupCmd(&client));
    client.addCommand(new KeepaliveCmd(&client));

    // copy data for sending to the server during communication session
    // you can call this function anywhere
    client.putDataForSend((void*)"gkjfdghdfjhgjdfhgkjdsfhaljdfgaljgg", 32);
    return true;
}

#endif // ! INITNRF24DCCLIENT_H

