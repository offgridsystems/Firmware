#ifndef TUNERXPACMD_H
#define TUNERXPACMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class TuneRxPaCmd :
    public AbstractClientCommand
{
public:
    TuneRxPaCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "tuneRxPa", DC_TUNE_RX_PA)
    {
    }

    ~TuneRxPaCmd() = default;

    bool run(String commandParametr)
    {
        //Serial.println(F("Satrt command CC"));
        //Serial.print(F("command parametr"));
        //Serial.println(commandParametr);

        Serial.print("Starting to tune Rx PA ");
        Serial.println(commandParametr);
        client_->setTimeoutBeforeStartSearchingNetwork(DC_DEFAULT_OFFLINE_TIMEOUT);
        delay(2);

        if (commandParametr.length() > 0)
        {
            int16_t id = commandParametr.toInt();

            if (client_->deviceId() != id)
                return false;
        }
        else
        {
            return false;
        }

        //Serial.println(F("Starting to tune Rx PA"));
        client_->startSession();
        client_->driver.startListening();
        uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET] = { 0 };

        for (int i = 0; i < 4; ++i) {
            auto startTime = millis();
            int32_t r = 0;
            //driver_.printDetails();

            while ((millis() - startTime) < DC_TUNNIG_PA_TIMEOUT) {
                if (client_->driver.available()) {
                    client_->driver.read(buf, DC_MAX_SIZE_OF_RF_PACKET);
                    ++r;
                }
                yield();
            }
            Serial.println(r);
            yield();

        }
        //Serial.println("Results: ");

        //for (int i = 0; i < 4; ++i) {
        //    Serial.println(results[i]);
        //}

        //client_->resetKeepAliveTimer();
    }
};

#endif