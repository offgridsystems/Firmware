#ifndef TUNETXPACMD_H
#define TUNETXPACMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

//#define DC_SSCMD_RECEIVING_TIMEOUT

class TuneTxPaCmd :
    public AbstractClientCommand
{
public:
    TuneTxPaCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "tuneTxPa", DC_TUNE_TX_PA)
    {
    }

    ~TuneTxPaCmd() = default;

    bool run(String commandParametr)
    {
        //Serial.println(F("Satrt command CC"));
        //Serial.print(F("command parametr"));
        //Serial.println(commandParametr);
        
        Serial.print(F("Starting to tune Tx PA 1 "));
        Serial.println(commandParametr);
        client_->setTimeoutBeforeStartSearchingNetwork(DC_DEFAULT_OFFLINE_TIMEOUT);

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

        int32_t results[4] = { 0 };
        int32_t resultsFalls[4] = { 0 };

        client_->startSession();
        for (int i = 0; i < 4; ++i) {
            client_->driver.stopListening();
            client_->setRF_PA_Level(i);
            auto startTime = millis();
            int res = 0;
            int fall = 0;
            //client_->driver.printDetails();
            while ( (millis() - startTime) < DC_TUNNIG_PA_TIMEOUT) {
                
                if (client_->driver.write("12345678909876543212345678909876", 32))
                    ++res;
                else
                    ++fall;
            }
            results[i] = res;
            resultsFalls[i] = fall;

            Serial.println(res);
            Serial.println(fall);
            client_->driver.txStandBy();
        }

        uint8_t pa = getBestPaLevelResult(results, resultsFalls, 4);
        Serial.print("New PA level: ");
        Serial.println(pa);

        client_->setRF_PA_Level(pa);
        client_->resetKeepAliveTimer();
    }
};

#endif