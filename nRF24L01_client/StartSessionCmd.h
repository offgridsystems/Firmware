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
        auto r = micros();
        //Serial.println(F("Satrt command ssh"));
        client_->resetKeepAliveTimer();

        bool isRecvData = false;
        client_->setTimeoutBeforeStartSearchingNetwork(DC_DEFAULT_OFFLINE_TIMEOUT);

        if (commandParametr.length() > 0)
        {
            int sepIdx = commandParametr.indexOf(',');

            if (sepIdx == -1) {
                Serial.println(F("Separator <,> was not found."));
                return false;
            }

            int16_t id = commandParametr.substring(0, sepIdx).toInt();

            if (id != client_->deviceId()) {
                //Serial.print(F("Not my id "));
                //Serial.println(id);
                return false;
            }

            int dataLen = commandParametr.length() - sepIdx;
            client_->setReceivedData(commandParametr.substring(sepIdx + 1).c_str(), dataLen);
        } 
        else
        {
            Serial.println(F("Command parametr length = 0."));
            return false;
        }

        //Serial.println(F("Start session "));
        delayMicroseconds(800);
        if (!client_->startSession())
        {
            Serial.println(F("Start session error"));
            return false;
        }

        client_->driver.openWritingPipe(client_->clientAddress());
        //driver.printDetails();

        if (client_->sendDataToServer())
        {
            isRecvData = true;
            Serial.println(F("Sended "));

            //client_->sendEndSessionTag();
            //continue;
        }
        else
        {
            isRecvData = false;
            Serial.println(F("error send packet"));
            //return false;
        }
        Serial.println(micros() - r);

        client_->resetKeepAliveTimer();
        return isRecvData;
    }
};

#endif