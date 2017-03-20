#ifndef NRF24_DC_CLIENT_H
#define NRF24_DC_CLIENT_H

#include <RF24/RF24.h>
//#include "StartSessionCmd.h"
#include "AbstractClientCommand.h"

#define DC_START_SESSION 0x01
#define DC_NUMBER_OF_COMMANDS 2

//class AbstractClientCommand;

typedef RF24 rfDriver;
class Nrf24DcClient
{
  public:
    Nrf24DcClient(rfDriver& drv);
    bool init();
    bool startSession();
    int listenBroadcast();
    void setWorkChannel(uint8_t ch);
    void setBroadcastChannel(uint8_t ch);
    uint8_t workChannel();
    uint8_t broadcastChannel();
    void setNetworkAddr(uint64_t nAddr);
	uint64_t networkAddr();
    void setDeviceId(int16_t id);
    int16_t deviceId();
    uint64_t clientAddress();

    rfDriver& driver;
    bool isBroadcastMode_;
    void setSessionTimeout(uint16_t timeout);
    uint16_t sessionTimeout();
    bool addCommand(AbstractClientCommand* cmd);
  
  private:
	uint64_t serverAddress_;
	uint64_t networkAddress_;
	uint64_t clientAddress_;
    uint16_t clientId;
    uint8_t broadcastChannel_;
    uint8_t workChannel_;
    uint8_t buffer_[32];
    int8_t bufferLen_;
    uint16_t sessionTimeout_;

    int8_t bytePos(uint8_t searchedByte, uint8_t* data, uint8_t len);

    AbstractClientCommand* cmdArray_[DC_NUMBER_OF_COMMANDS];
    uint8_t cmdCount_;

};



#endif
