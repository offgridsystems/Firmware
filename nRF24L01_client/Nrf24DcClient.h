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

    // Prepare to session between client and server
    bool startSession();

    // Putting buffer with data for sending to server
    // during communication session
    // @parametrs buffer - pointer to the source buffer
    //               len - size of source buffer
    void putDataForSend(void* buffer, int8_t len);

    // Getting data received during communication session
    // @parametrs buffer - pointer to the destination buffer
    //            maxLen - size of destination buffer
    // Return size of received data
    int8_t getReceivedData(void* buffer, int8_t maxLen);

    // Listen broadast chanel to commands from server
    // call this function in main loop
    uint8_t listenBroadcast();

    // Set number of work channel for direct 
    // connectin between server and client
    void setWorkChannel(uint8_t ch);

    // Set number of channel for broadcast channell
    void setBroadcastChannel(uint8_t ch);

    // Returns number of work channel
    uint8_t workChannel();

    // Returns number of broadcast channel
    uint8_t broadcastChannel();

    // Set network address
    // @note: address have to be 3 bytes (0xFFFFFF is maximmum) 
    void setNetworkAddr(uint64_t nAddr);

    //Returns network address.
    uint64_t networkAddr();

    // Set unique ID for device from 1 to 1024
    void setDeviceId(int16_t id);

    // Returns unique ID for device from 1 to 1024
    int16_t deviceId();

    // Returns client address
    // Infact it is Network address + id
    uint64_t clientAddress();

    // reference to nrf24 driver 
    rfDriver& driver;

    // Set session timeout 
    void setSessionTimeout(int16_t timeout);

    // Returns session timeout 
    int16_t sessionTimeout();

    // Add command object for handling commands from server
    bool addCommand(AbstractClientCommand* cmd);

    // Sends data to the server put to the internal buffer putDataForSend()
    // Returns true if server received data
    bool sendDataToServer();

    // Receives data from server to internal buffer
    // if data is received, you can get it by getReceivedData()
    // @parametr timeout - time int mili sec.
    // Return true if any data was received
    bool receiveDataFromServer(int16_t timeout = 500);

    // wait for start session command from server
    // during <timeout> time 
    // Return true if command was received
    // @parametr timeout - in milliseconds
    //           -1 means sessionTimeout()
    bool receiveStartSessionTag(int16_t timeout = -1);

private:
    bool isBroadcastMode_;
    uint64_t serverAddress_;
    uint64_t networkAddress_;
    uint64_t clientAddress_;
    uint16_t clientId;
    uint8_t broadcastChannel_;
    uint8_t workChannel_;
    uint8_t buffer_[32];
    int8_t bufferLen_;
    int16_t sessionTimeout_;

    AbstractClientCommand* cmdArray_[DC_NUMBER_OF_COMMANDS];
    uint8_t cmdCount_;

    uint8_t dataToSend_[32];
    int8_t dataToSendLength_;
    uint8_t receivedData_[32];
    int8_t receivedDataLength_;

    int8_t bytePos(uint8_t searchedByte, uint8_t* data, uint8_t len);
    void prepareSesionBuffers_();

};



#endif
