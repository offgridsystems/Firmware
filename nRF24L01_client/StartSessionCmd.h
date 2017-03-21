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
        if (commandParametr.length() > 0)
        {
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


        if (!client_->receiveStartSessionTag())
        {
            Serial.println(F("Start not received"));
            return false;
        }
        client_->driver.stopListening();

        client_->driver.flush_tx();
        client_->driver.openWritingPipe(client_->clientAddress());
        //driver.printDetails();

        if (!client_->sendDataToServer())
        {
            Serial.println(F("error send packet"));
            return false;
        }

        client_->driver.txStandBy();

        client_->driver.openReadingPipe(1, client_->clientAddress());
        client_->driver.startListening();
        bool isRecvData = client_->receiveDataFromServer();
        client_->driver.stopListening();

        return isRecvData;
    }
};

#endif