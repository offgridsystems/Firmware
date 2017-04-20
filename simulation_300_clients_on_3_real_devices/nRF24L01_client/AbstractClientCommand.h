#ifndef ABSTRACTCLIENTCOMMAND_H
#define ABSTRACTCLIENTCOMMAND_H

//#include "Nrf24DcClient.h"

class Nrf24DcClient;
class AbstractClientCommand
{
public:
    AbstractClientCommand(Nrf24DcClient *client, String commandName, uint8_t returnCode)
        : name_(commandName)
        , client_(client)
        , returnCode_(returnCode)
    {
    }

    virtual ~AbstractClientCommand() = default;
    
    virtual bool run(String commandParametr) = 0;
    bool isCommand(String commandName)
    {
        //Serial.print(F("isCommand "));
        //Serial.println(commandName);
        //Serial.println(name_);
        //Serial.println(commandName == name_);
        return commandName == name_;
    }

    String name() 
    {
        return name_;
    }

    uint8_t returnCode()
    {
        return returnCode_;
    }

protected:
    String name_;
    Nrf24DcClient *client_;
    uint8_t returnCode_;
};

#endif