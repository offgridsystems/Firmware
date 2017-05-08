#ifndef DCRF24DRIVERPROXY_H
#define DCRF24DRIVERPROXY_H

#include <RF24.h> //standard arduino library, you can install it on library manager
#include <FastCRC.h>  //standard arduino library, you can install it on library manager
#include "dc_global.h"


#define DC_AUTOACK_PAYLOAD "ack"
#define DC_AUTOACK_RECEIVED_PAYLOAD "ack+"



class DC_RF24DriverProxy : protected RF24
{
public:
    DC_RF24DriverProxy() = delete;
    DC_RF24DriverProxy(uint8_t cepin, uint8_t cspin);
    ~DC_RF24DriverProxy();
    bool begin();
    void read(void* buf, uint8_t len);
    bool write(const void* buf, uint8_t len);
    void setRetries(uint16_t retransmitionPause, uint16_t retries);
    void setAutoAck(bool flag);
    void startListening();
    void stopListening();
    void openReadingPipe(uint8_t number, uint64_t addr);
    void openWritingPipe(uint64_t addr);
    bool txStandBy();
    uint8_t flush_tx(void);
    void flush_rx();
    bool available();
    void setChannel(uint8_t ch);
    uint8_t getDynamicPayloadSize();
    bool setDataRate(rf24_datarate_e speed);
    void setPALevel(uint8_t level);
    bool testRPD(void);
    bool testCarrier(void);

    void setEncryption(bool flag);
    bool encryption();
    void setEcryptKeyPointer(uint8_t *pointer, uint8_t len);
    void encryptMsg(uint8_t *msg, uint8_t size);
    void decryptMsg(uint8_t *msg, uint8_t size);


private:
    //RF24 _driver;
    uint16_t _retries;
    uint16_t _retransmissionPause;
    bool _autoAckFlag;
    volatile bool isEncrypt_;
    volatile uint8_t *keyPtr_;
    volatile uint8_t keySize_;
    FastCRC32 _crc;

    void waitEndSessionTag(uint32_t crc);
    void readWithoutAck(void* buf, uint8_t len);
    bool writeWithoutAck(const void* buf, uint8_t len);
    void writeAckPacket();


};

#endif // !DCRF24DRIVERPROXY_H


