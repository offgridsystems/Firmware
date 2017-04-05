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
        Serial.println(F("Start command lookup"));
        if (commandParametr.length() > 0)
        {
            Serial.print("Command parametr ");
            Serial.println(commandParametr);
            int16_t ch = commandParametr.toInt();
            client_->setWorkChannel(ch);
        }

        if (!client_->startSession())
        {
            Serial.println(F("Start session error"));
            res = false;
        }

        if (res)
        {
            client_->driver.openReadingPipe(1, client_->clientAddress());
            client_->driver.openWritingPipe(client_->clientAddress());
            client_->driver.startListening();
        }

        if (!client_->receiveStartSessionTag(4000) && res)
        {
            //Serial.println(F("Start not received"));
            res = false;
        }

        return res;
    }
};

#endif