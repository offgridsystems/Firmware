#ifndef LOOKUPCMD_H
#define LOOKUPCMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class LookupCmd :
    public AbstractClientCommand
{
public:
    LookupCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "lookup", DC_LOOKUP)
    {
    }

    ~LookupCmd() = default;

    bool run(String commandParametr)
    {
        bool res = true;
        //Serial.println(F("Start command lookup"));

        if (commandParametr.length() > 0)
        {
            int16_t t = commandParametr.toInt() * 1.2;
            client_->setTimeoutBeforeStartSearchingNetwork(t);
        }

        if (!client_->startSession())
        {
            Serial.println(F("Start session error"));
            res = false;
        }

        if (res)
        {
            uint64_t clientAddr = client_->clientAddress();
            client_->driver.openReadingPipe(1, clientAddr);
            client_->driver.openWritingPipe(clientAddr);
            client_->driver.startListening();
        }

        if (!client_->receiveStartSessionTag(4000) && res)
        {
            //Serial.println(F("Start not received"));
            res = false;
        }
        //client_->driver.printDetails();

        return res;
    }
};

#endif