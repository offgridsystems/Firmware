#ifndef ABSTRACTCLIENTCOMMAND_H
#define ABSTRACTCLIENTCOMMAND_H

//#include "Nrf24DcClient.h"

class Nrf24DcClient;
class AbstractClientCommand
{
public:
    AbstractClientCommand(Nrf24DcClient *client, String commandName)
        : name_(commandName)
        , client_(client)
    {
    }

    virtual ~AbstractClientCommand() = default;
    
    virtual bool run(String commandParametr) = 0;
    virtual bool isCommand(String commandName)
    {
        //Serial.print(F("isCommand "));
        //Serial.println(commandName);
        //Serial.println(name_);
        //Serial.println(commandName == name_);
        return commandName == name_;
    }

protected:
    String name_;
    Nrf24DcClient *client_;
};

#endif