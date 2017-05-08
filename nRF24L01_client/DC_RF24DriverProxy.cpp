#include "DC_RF24DriverProxy.h"

DC_RF24DriverProxy::DC_RF24DriverProxy(uint8_t cepin, uint8_t cspin)
    : RF24(cepin, cspin)
    , _retries(0)
    , _retransmissionPause(0)
    , _autoAckFlag(false)
{
}


DC_RF24DriverProxy::~DC_RF24DriverProxy()
{
}

bool DC_RF24DriverProxy::begin()
{
    bool res = RF24::begin();
    RF24::setAutoAck(false);
    RF24::setAddressWidth(5);
    RF24::setCRCLength(RF24_CRC_16);
    RF24::enableDynamicPayloads();
    return res;
}

void DC_RF24DriverProxy::read(void * buf, uint8_t len)
{
    readWithoutAck(buf, len);
    //RF24::flush_rx();

    if (_autoAckFlag)
    {
        writeAckPacket();

        uint32_t crcCode = _crc.cksum((uint8_t*)buf, len);
        auto startTime = micros();
        auto receivingEndSessionMsgTimeout = _retries * _retransmissionPause;
        //Serial.println(receivingEndSessionMsgTimeout_);

        while ((micros() - startTime) < (uint32_t)receivingEndSessionMsgTimeout)
        {
            if (RF24::available())
            {
                uint8_t msgLen = RF24::getDynamicPayloadSize();
                uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET] = { 0 };
                readWithoutAck(buf, msgLen);
                uint32_t crc2 = _crc.cksum(buf, msgLen);

                //Serial.print(" waitEndSessionTag ");
                //Serial.println((char*)buf);

                if (crcCode == crc2 && len == msgLen)
                {
                    writeAckPacket();
                }
                else if (msgLen == strlen(DC_AUTOACK_RECEIVED_PAYLOAD)
                    && strcmp(DC_AUTOACK_RECEIVED_PAYLOAD, (char*)buf) == 0)
                {
                    break;
                }

            }
        }
    }
}

bool DC_RF24DriverProxy::write(const void * buf, uint8_t len)
{
    writeWithoutAck(buf, len);
    bool res = true;

    if (_autoAckFlag)
    {
        res = false;

        for (uint16_t i = 0; i < _retries; ++i)
        {
            RF24::startListening();
            auto startTime = micros();

            while ((micros() - startTime) > _retransmissionPause)
            {
                if (RF24::available()) {
                    uint8_t msgLen = RF24::getDynamicPayloadSize();
                    char buf[32] = { 0 };
                    readWithoutAck(buf, msgLen);

                    if (msgLen != strlen(DC_AUTOACK_PAYLOAD) && strcmp(DC_AUTOACK_PAYLOAD, buf) != 0) {
                        continue;
                    }

                    uint8_t l = strlen(DC_AUTOACK_RECEIVED_PAYLOAD);

                    RF24::stopListening();

                    for (int i = 0; i < 3; ++i)
                        writeWithoutAck(DC_AUTOACK_RECEIVED_PAYLOAD, l);

                    RF24::txStandBy();
                    res = true;
                    break;
                }
            }

            if (res)
            {
                break;
            }
            else
            {
                RF24::stopListening();
                writeWithoutAck(buf, len);
                RF24::txStandBy();
            }
        }
    }

    return res;
}


void DC_RF24DriverProxy::setRetries(uint16_t retransmitionPause, uint16_t retries)
{
    _retransmissionPause = retransmitionPause;
    _retries = retries;
}

void DC_RF24DriverProxy::setAutoAck(bool flag)
{
    _autoAckFlag = flag;
}

void DC_RF24DriverProxy::startListening()
{
    RF24::startListening();
}

void DC_RF24DriverProxy::stopListening()
{
    RF24::stopListening();
}

void DC_RF24DriverProxy::openReadingPipe(uint8_t number, uint64_t addr)
{
    RF24::openReadingPipe(number, addr);
}

void DC_RF24DriverProxy::openWritingPipe(uint64_t addr)
{
    RF24::openWritingPipe(addr);
}

bool DC_RF24DriverProxy::txStandBy()
{
    return RF24::txStandBy();
}

uint8_t DC_RF24DriverProxy::flush_tx(void)
{
    return RF24::flush_tx();
}

void DC_RF24DriverProxy::flush_rx()
{
    RF24::flush_rx();
}

bool DC_RF24DriverProxy::available()
{
    return RF24::available();
}

void DC_RF24DriverProxy::setChannel(uint8_t ch)
{
    RF24::setChannel(ch);
}

uint8_t DC_RF24DriverProxy::getDynamicPayloadSize()
{
    return RF24::getDynamicPayloadSize();
}

bool DC_RF24DriverProxy::setDataRate(rf24_datarate_e speed)
{
    return RF24::setDataRate(speed);
}

void DC_RF24DriverProxy::setPALevel(uint8_t level)
{
    RF24::setPALevel(level);
}

bool DC_RF24DriverProxy::testRPD(void)
{
    return RF24::testRPD();
}

bool DC_RF24DriverProxy::testCarrier(void)
{
    return RF24::testCarrier();
}

void DC_RF24DriverProxy::waitEndSessionTag(uint32_t crc)
{

}

void DC_RF24DriverProxy::readWithoutAck(void * buf, uint8_t len)
{
    RF24::read(buf, len);

    if (isEncrypt_)
    {
        uint8_t *msgPtr = (uint8_t*)buf;
        decryptMsg(msgPtr, len);
    }

}

bool DC_RF24DriverProxy::writeWithoutAck(const void * buf, uint8_t len)
{
    uint8_t *msgPtr = (uint8_t*)buf;
    uint8_t tmpBuf[32] = { 0 };

    if (isEncrypt_)
    {
        memcpy(tmpBuf, msgPtr, len);
        msgPtr = &tmpBuf[0];
        encryptMsg(msgPtr, len);
    }

    //RF24::printDetails();
    //Serial.println(len);
    //RF24::stopListening();

    auto d = micros();
    for (int i = 0; i < 4; ++i)
        RF24::write(msgPtr, len, 1);
        //Serial.println((char*)msgPtr);
    Serial.println(micros() - d);

    RF24::txStandBy();
}

void DC_RF24DriverProxy::writeAckPacket()
{
    RF24::stopListening();
    writeWithoutAck(DC_AUTOACK_PAYLOAD, strlen(DC_AUTOACK_PAYLOAD));
    RF24::flush_rx();
    RF24::startListening();
}

void DC_RF24DriverProxy::setEncryption(bool flag)
{
    isEncrypt_ = flag;
}

bool DC_RF24DriverProxy::encryption()
{
    return isEncrypt_;
}

void DC_RF24DriverProxy::setEcryptKeyPointer(uint8_t *pointer, uint8_t len)
{
    keyPtr_ = pointer;
    keySize_ = len;
}

void DC_RF24DriverProxy::encryptMsg(uint8_t *msg, uint8_t size)
{
    int keyPos = 0;
    for (int i = 0; i < size; ++i)
    {
        msg[i] = msg[i] ^ this->keyPtr_[keyPos];
        ++keyPos;

        if (keyPos == keySize_)
            keyPos = 0;
    }
}

void DC_RF24DriverProxy::decryptMsg(uint8_t * msg, uint8_t size)
{
    int keyPos = 0;
    for (int i = 0; i < size; ++i)
    {
        msg[i] = msg[i] ^ this->keyPtr_[keyPos];
        ++keyPos;

        if (keyPos == keySize_)
            keyPos = 0;
    }

}
