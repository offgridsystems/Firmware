#ifndef KEEPALIVECMD_H
#define KEEPALIVECMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class KeepaliveCmd :
    public AbstractClientCommand
{
public:
    KeepaliveCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "keepAlive", DC_KEEPALIVE)
    {
    }

    ~KeepaliveCmd() = default;

    bool run(String commandParametr)
    {
        if (commandParametr.length() > 0)
        {
            int16_t t = commandParametr.toInt() * 1.2;
            client_->setTimeoutBeforeStartSearchingNetwork(t);
        }

        //Serial.println(F("Satrt command keepAlive"));
        client_->resetKeepAliveTimer();
    }
};

#endif