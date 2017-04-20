#ifndef STARTSESSIONCMD_H
#define STARTSESSIONCMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class StartSessionCmd :
    public AbstractClientCommand
{
public:
    StartSessionCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "ssch", DC_START_SESSION)
    {
    }

    ~StartSessionCmd() = default;

    bool run(String commandParametr)
    {
        //Serial.println(F("Satrt command ssh"));
        client_->resetKeepAliveTimer();

        int32_t t = DC_DEFAULT_OFFLINE_TIMEOUT;
        bool isRecvData = false;
        if (commandParametr.length() > 0)
        {
            t = commandParametr.toInt() * 1.2;
            client_->setTimeoutBeforeStartSearchingNetwork(t);
        }

        auto startTime = millis();

        int16_t startId = client_->deviceId();
        int16_t currentId = client_->deviceId();

        while ((millis() - startTime) < t)
        {
            client_->setDeviceId(currentId);
            currentId += 3;

            if (currentId > 330)
                break;

            client_->driver.stopListening();

            if (!client_->startSession())
            {
                Serial.println(F("Start session error"));
                continue;
            }

            client_->driver.openReadingPipe(1, client_->clientAddress());
            client_->driver.startListening();


            if (!client_->receiveStartSessionTag(t - (millis() - startTime)))
            {
                Serial.println(F("Start not received"));
                continue;
            }

            client_->waitEndSessionTag();
            client_->driver.stopListening();

            client_->driver.openWritingPipe(client_->clientAddress());

            if (client_->sendDataToServer())
            {
                isRecvData = true;
                client_->sendEndSessionTag();
            }
            else
            {
                isRecvData = false;
            }

            client_->driver.txStandBy();

            //if (isRecvData)
            //    break;


        }

        client_->setDeviceId(startId);

        client_->resetKeepAliveTimer();
        return isRecvData;
    }
};

#endif