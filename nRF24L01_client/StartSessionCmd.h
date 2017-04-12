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

        int16_t t = 1000;
        bool isRecvData = false;
        if (commandParametr.length() > 0)
        {
            t = commandParametr.toInt() * 1.2;
            client_->setTimeoutBeforeStartSearchingNetwork(t);
        }

        auto startTime = millis();
        while ((millis() - startTime) < t)
        {
            client_->driver.stopListening();

            if (!client_->startSession())
            {
                Serial.println(F("Start session error"));
                continue;
            }

            client_->driver.openReadingPipe(1, client_->clientAddress());
            client_->driver.startListening();


            if (!client_->receiveStartSessionTag(t-(millis()-startTime)))
            {
                Serial.println(F("Start not received"));
                continue;
            }
            //client_->driver.printDetails();
            isRecvData = client_->receiveDataFromServer();

            if (!isRecvData)
            {
                continue;
            }
            delay(1);
            client_->driver.stopListening();

            client_->driver.openWritingPipe(client_->clientAddress());
            //driver.printDetails();

            if (!client_->sendDataToServer())
            {
                isRecvData = false;
                //Serial.println(F("error send packet"));
                //continue;
            }

            client_->driver.txStandBy();

            if ( isRecvData )
                break;
        }

        return isRecvData;
    }
};

#endif