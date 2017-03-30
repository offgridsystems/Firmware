
#ifndef NRF24_DC_SERVER_H
#define NRF24_DC_SERVER_H
#include <RF24/RF24.h>

#define DC_MAX_CLIENT_NUMBER 300
#define DC_MAX_SIZE_OF_RECEIVED_DATA 32
#define DC_MAX_SIZE_OF_DATA_FOR_SENDING 4
#define DC_NUMBER_OF_BROADCAST_REQUESTS 6

#define DC_DEFAULT_SESSION_TIMEOUT 3000
#define DC_DEFAULT_READING_TIMEOUT 3

typedef RF24 rfDriver;

class Nrf24DcServer
{
  public:
    Nrf24DcServer(rfDriver& drv);

    // Thsi function id used for seeking clients
    // returns number of found clients
    // @parameter timeout - timeout for sekking
    // @note:
    // this function clears list of handled device
    // and store ids of found clients
    uint16_t lookForClient(int timeout);

    // Initializes object and driver
    bool init();

    // Set number of channel for session communication
    void setWorkChannel(uint8_t ch);

    // Set number of channel for bradcast messages
    void setBroadcastChannel(uint8_t ch);

    // Returns number of work channel
    uint8_t workChannel();

    // Returns number of bradcast channel
    uint8_t broadcastChannel();

    // Set network address
    // @note: address have to be 3 bytes (0xFFFFFF is maximmum) 
    void setNetworkAddr(uint64_t nAddr);

    //Returns network address.
	uint64_t networkAddr();
    
    // Turn on broadcast mode.
    // This means that boradcast channel and NoACK mode will be setted
    bool setBroadcastMode();

    // Turn on single client mode on.
    // This means that work channel and AuroACK mode will be setted
    // @parameter clientId - id of client which will communicate with server
    bool setSingleClientMode(int16_t clientId);
    
    // Returns number of handled clients added by addClient() or addClientByRange()
    int handledClientsCount();

    // Adds client which server will handle
    // @parametr clientId - id of client
    bool addClient(int16_t clientId);

    // Adds clients which server will handle by range
    // @parametr from - id of first client
    //          count - number of clients
    // @Example : addClientByRange(5,4);
    // Adds 4 client with id 5, 6, 7, 8;
    bool addClientByRange(int16_t from, int16_t count);

    // Remove client from handled list,
    // and this client won't be handled by server
    // @parameter clientId - id of device
    // if clientId == -1 all clients will be deleted from handled list
    bool removeClient(int16_t clientId);

    // Remove clients from handled list,
    // and these clients won't be handled by server
    // @parameter from - first id of client
    //           count - number of clients
    // @Example : removeClientsByRange(5,4);
    // Removes 4 client with id 5, 6, 7, 8;
    bool removeClientsByRange(int16_t from, int16_t count);

    // Returns ID of client by index from internal list of handled clients
    int16_t clientIdAt(int16_t index);

    // Returns index of client by id from internal list of handled clients
    int16_t clientIndexById(int16_t id);

    /**
     * returns true if client in array for handling
     * @parameters <id> id of client
     */
    bool isDeviceHandled(int16_t id);


    void setSessionTimeout(uint16_t timeOut);
    uint16_t sessionTimeout();

    void setReadingTimeout(uint16_t timeout);
    uint16_t readingTimeout();

    bool sendRequestForSession();
    bool sendRequestForLookup();

    bool sendStartSesionTag(uint8_t times = 1);
    int16_t startSession();

    uint8_t* receivedDataById(uint16_t id);
    uint8_t* receivedDataByIndex(uint16_t idx);
    uint8_t  lenfgthOfReceivedBufferByIndex(uint16_t idx);
    uint8_t  lenfgthOfReceivedBufferById(uint16_t id);

    // copy data from <data> to internal buffer 
    // this data will be sended during communication session
    // @parametrs:
    //    idx - index of internal buffer
    //   data - pointer to the buffer
    //    len - size of buffer
    // @note: you can get index for client by calling 
    //         deviceIndexById(id) - where id is id of client
    void putSendedData(int16_t idx, const void *data, uint8_t len);


  private:
    rfDriver &driver_;
    uint64_t networkAddress_;
	uint64_t serverAddress_;
    uint8_t broadcastChannel_;
    uint8_t workChannel_;
    //int16_t handledDevicesCount_;
    
    int16_t handledDevicesId_[DC_MAX_CLIENT_NUMBER];
    int8_t commsStatus_[DC_MAX_CLIENT_NUMBER];
    uint8_t dataBufferFromClients_[DC_MAX_CLIENT_NUMBER][DC_MAX_SIZE_OF_RECEIVED_DATA];
    uint8_t dataBufferFromClientsSize_[DC_MAX_CLIENT_NUMBER];
    uint8_t dataBufferToClients_[DC_MAX_CLIENT_NUMBER][DC_MAX_SIZE_OF_DATA_FOR_SENDING];

    int16_t currnetDeviceCount_;
    
    uint8_t sendBuffer_[32];
    //uint8_t recvBuffer_[32];
    uint16_t sessionTimeout_;
    uint16_t readingTimeout_;

    void prepareArrays();
    bool sendBroadcastRequestCommand(String command);
  
};



#endif
