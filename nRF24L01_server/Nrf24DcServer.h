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

    // Turn on single device mode on.
    // This means that work channel and AuroACK mode will be setted
    // @parameter device_id - id of device which will communicate with server
    bool setSingleDeviceMode(int16_t device_id);
    
    // Returns number of devices which server will handle
    int handledDeviceCount();

    // Adds device which server will handle
    // @parametr deviceId - id of device 
    bool addDevice(int16_t deviceId);

    // Adds devices which server will handle by range
    // @parametr from - id of first device 
    //          count - number of devices
    // @Example : addDeviceByRange(5,4);
    // Adds 4 device with id 5, 6, 7, 8;
    bool addDeviceByRange(int16_t from, int16_t count);

    // Remove device from handled list,
    // and this device won't be handled by server
    // @parameter deviceId - id of device
    bool removeDevice(int16_t deviceId);

    // Remove devices from handled list,
    // and these device won't be handled by server
    // @parameter from - first id of device
    //           count - number of diveces
    // @Example : removeDeviceByRange(5,4);
    // Removes 4 device with id 5, 6, 7, 8;
    bool removeDeviceByRange(int16_t from, int16_t count);

    // Returns ID of device by index from internal list of handled devices
    int16_t deviceIdAt(int16_t index);

    // Returns index of device by id from internal list of handled devices
    int16_t deviceIndexById(int16_t id);

    /**
     * returns true if device in array for handling
     * @parameters <id> id of device
     */
    bool isDeviceHandled(int16_t id);


    void setSessionTimeout(uint16_t timeOut);
    uint16_t sessionTimeout();

    void setReadingTimeout(uint16_t timeout);
    uint16_t readingTimeout();

    bool sendRequestForSession();

    int16_t startSession();

    uint8_t* receivedDataById(uint16_t id);
    uint8_t* receivedDataByIndex(uint16_t idx);
    uint8_t  lenfgthOfReceivedBufferByIndex(uint16_t idx);
    uint8_t  lenfgthOfReceivedBufferById(uint16_t id);

    void putSendedData(uint16_t idx, const void *data, uint8_t len);


  private:
    rfDriver &driver_;
    uint64_t networkAddress_;
	uint64_t serverAddress_;
    uint8_t broadcastChannel_;
    uint8_t workChannel_;
    int16_t handledDevicesCount_;
    
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
    void sendBroadcastRequestCommand();
  
};



#endif
