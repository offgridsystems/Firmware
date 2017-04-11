#ifndef NRF24_DC_CLIENT_H
#define NRF24_DC_CLIENT_H

#include <RF24/RF24.h>
//#include "StartSessionCmd.h"
#include "AbstractClientCommand.h"

#define DC_START_SESSION 0x01
#define DC_LOOKUP 0x02
#define DC_KEEPALIVE 0x03
#define DC_NUMBER_OF_COMMANDS 3
#define DC_CHANNEL_COUNT 126
#define DC_REPS_COUNT 50
#define DC_DEFAULT_KEEPALIVE_TIMEOUT 100
#define DC_DEFAULT_TIMEOUT_BEFORE_STARTING_SEARCH_SERVER 1000

#define DC_CRYPTO_KEY_SIZE 16

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
    // if buffer = nullptr then function returns only size of received data
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
    void setSessionTimeout(int16_t timeout_mSec);

    // Returns session timeout 
    int16_t sessionTimeout();

    void setKeepaliveTimeout(int16_t timeout_mSec);
    int16_t keepalveTimeout();

    void setTimeoutBeforeStartSearchingNetwork(int16_t timeout_mSec);
    int16_t timeoutBeforeStartSearchingNetwork();

    // Add command object for handling commands from server
    bool addCommand(AbstractClientCommand* cmd);

    // Sends data to the server put to the internal buffer putDataForSend()
    // Returns true if server received data
    bool sendDataToServer();

    // Receives data from server to internal buffer
    // if data is received, you can get it by getReceivedData()
    // @parametr timeout - time int mili sec.
    // Return true if any data was received
    bool receiveDataFromServer(int16_t timeout = 5);

    // wait for start session command from server
    // during <timeout> time 
    // Return true if command was received
    // @parametr timeout - in milliseconds
    //           -1 means sessionTimeout()
    bool receiveStartSessionTag(int16_t timeout = -1);

    void clientLoop();

    void keepServer();

    void resetKeepAliveTimer();

    void setEncryption(bool flag);
    bool encryption();
    void setEcryptKeyPointer(uint8_t *pointer, uint8_t len);
    void encryptMsg(uint8_t *msg, uint8_t size);
    void decryptMsg(uint8_t *msg, uint8_t size);

private:
    volatile bool isBroadcastMode_;
    volatile uint64_t serverAddress_;
    volatile uint64_t networkAddress_;
    volatile uint64_t clientAddress_;
    volatile uint16_t clientId;
    //uint8_t broadcastChannel_;
    volatile uint8_t workChannel_;
    uint8_t buffer_[32];
    volatile int8_t bufferLen_;
    volatile int16_t sessionTimeout_;
    volatile int16_t keepAliveTimeout_;
    volatile uint32_t lastKeepaliveTime_;
    volatile int16_t timeoutBeforeStartSearchingNetwork_;

    AbstractClientCommand* cmdArray_[DC_NUMBER_OF_COMMANDS];
    uint8_t channelsActivity_[DC_CHANNEL_COUNT];
    volatile uint8_t cmdCount_;

    uint8_t dataToSend_[32];
    volatile int8_t dataToSendLength_;
    uint8_t receivedData_[32];
    volatile int8_t receivedDataLength_;

    volatile bool isEncrypt_;
    volatile uint8_t *keyPtr_;
    volatile uint8_t keySize_;

    int8_t bytePos(uint8_t searchedByte, uint8_t* data, uint8_t len);
    void prepareSesionBuffers_();
    bool isChannelBussy(uint8_t channel);
    void scanChannels();
    bool waitForKeepaliveMsg(int16_t timeout);
    void read(void *buf, uint8_t len);
    bool write(const void *buf, const uint8_t len);

};



#endif
