#ifndef INITNRF24DCCLIENT_H
#define INITNRF24DCCLIENT_H

#define NETWORK_ADDR 0xC7C7C7LL

#include "Nrf24DcClient.h"
#include "StartSessionCmd.h"

extern StartSessionCmd ssCmd;

bool initDcClient(Nrf24DcClient &client) 
{
    Serial.print(F("Init nrf driver - "));
    //Serial.flush();
    Serial.println(client.init());
    client.driver.setAddressWidth(5);
    client.driver.setRetries(1, 15);
    client.driver.setCRCLength(RF24_CRC_8);
    client.driver.enableDynamicPayloads();
    client.driver.setDataRate(RF24_1MBPS);

    client.setWorkChannel(10);
    client.setBroadcastChannel(120);
    client.setNetworkAddr(NETWORK_ADDR);
    client.setDeviceId(0xc7);
    client.setSessionTimeout(3000);

    //client.addCommand(&ssCmd);
    Serial.print(F("size of command "));
    Serial.println(client.addCommand(&ssCmd));
}

#endif // ! INITNRF24DCCLIENT_H

