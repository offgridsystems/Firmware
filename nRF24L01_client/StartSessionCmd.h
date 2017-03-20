#ifndef STARTSESSIONCMD_H
#define STARTSESSIONCMD_H

#include "AbstractClientCommand.h"
#include "Nrf24DcClient.h"

class StartSessionCmd :
    public AbstractClientCommand
{
public:
    StartSessionCmd(Nrf24DcClient* client)
        : AbstractClientCommand(client, "ssch")
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

        uint8_t buf[33] = { 0 };
        uint8_t rLen;

        if (!client_->startSession())
        {
            Serial.println(F("Start session error"));
            return false;
        }

        client_->driver.openReadingPipe(1, client_->clientAddress());
        client_->driver.startListening();
        //driver.printRegisters();

        auto recvStartTime = millis();
        bool isRecvData = false;
        rLen = 32;

        //driver.printDetails();
        while ((millis() - recvStartTime) <= client_->sessionTimeout())
        {
            //driver.printRegisters();
            if (client_->driver.available())
            {
                rLen = client_->driver.getDynamicPayloadSize();
                client_->driver.read(buf, rLen);
                //Serial.print("Received msg ");
                //Serial.println(rLen);
                //Serial.println((char*) buf);

                if (rLen == 2 && strncmp((char*)buf, "S", 1) == 0)
                {
                    isRecvData = true;
                    break;
                }
            }
        }

        if (!isRecvData)
        {
            Serial.println(F("Start not received"));
            return false;
        }
        client_->driver.stopListening();

        client_->driver.flush_tx();
        client_->driver.openWritingPipe(client_->clientAddress());
        //driver.printDetails();
        if (!client_->driver.write("12345678909876543212345678909871", 32))
        {
            Serial.println(F("error send packet"));
            return false;
        }

        client_->driver.txStandBy();
        isRecvData = false;

        rLen = 32;
        client_->driver.openReadingPipe(1, client_->clientAddress());
        client_->driver.startListening();
        while ((millis() - recvStartTime) <= client_->sessionTimeout())
        {
            if (client_->driver.available())
            {
                isRecvData = true;
                client_->driver.read(buf, 32);
                rLen = client_->driver.getDynamicPayloadSize();
                break;
            }
        }
        client_->driver.stopListening();

        return isRecvData;
    }
};

#endif