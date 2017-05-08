#ifndef CHANGECHANNELCMD_H
#define CHANGECHANNELCMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class ChangeChannelCmd :
    public AbstractClientCommand
{
public:
    ChangeChannelCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "cc", DC_CHANGE_CHANNEL)
    {
    }

    ~ChangeChannelCmd() = default;

    bool run(String commandParametr)
    {
        //Serial.println(F("Satrt command CC"));
        //Serial.print(F("command parametr"));
        //Serial.println(commandParametr);
        client_->setTimeoutBeforeStartSearchingNetwork(DC_DEFAULT_OFFLINE_TIMEOUT);
        if (commandParametr.length() > 0)
        {
            int16_t t = commandParametr.toInt();
            client_->setWorkChannel(t);
        }

        client_->resetKeepAliveTimer();
    }
};

#endif