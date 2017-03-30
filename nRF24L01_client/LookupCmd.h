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
        Serial.println(F("Satrt command lookup"));
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
            return false;
        }

        client_->driver.openReadingPipe(1, client_->clientAddress());
        client_->driver.startListening();


        if (!client_->receiveStartSessionTag(4000))
        {
            Serial.println(F("Start not received"));
            return false;
        }
        else
        {
            return true;
        }
    }
};

#endif