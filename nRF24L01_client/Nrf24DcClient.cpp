#include "Nrf24DcClient.h"
#include "RamMonitor.h"
//#include <RH_NRF24.h>

//int freeRam();

extern RamMonitor mon;

Nrf24DcClient::Nrf24DcClient(rfDriver& drv)
    : driver(drv)

{
    isBroadcastMode_ = false;
    cmdCount_ = 0;
    lastKeepaliveTime_ = 0;

    setKeepaliveTimeout(DC_DEFAULT_KEEPALIVE_TIMEOUT);
    setTimeoutBeforeStartSearchingNetwork(DC_DEFAULT_TIMEOUT_BEFORE_STARTING_SEARCH_SERVER);
}

bool Nrf24DcClient::init()
{
    bool res = this->driver.begin();
    this->driver.setAddressWidth(5);        // must be always 5
    this->driver.setCRCLength(RF24_CRC_16);  // number of bits of CRC, can be RF24_CRC_8 or RF24_CRC_16
    this->driver.enableDynamicPayloads();   // alway use this options
    this->driver.setAutoAck(false);
    return res;
}

void Nrf24DcClient::setWorkChannel(uint8_t ch)
{
    //Serial.print("Setted new channel ");
    //Serial.println(ch);

    workChannel_ = ch;
}

void Nrf24DcClient::setBroadcastChannel(uint8_t ch)
{
    //broadcastChannel_ = ch;
    workChannel_ = ch;
}

uint8_t Nrf24DcClient::workChannel()
{
    return workChannel_;
}

uint8_t Nrf24DcClient::broadcastChannel()
{
    //return broadcastChannel_;
    return workChannel_;
}

void Nrf24DcClient::setNetworkAddr(uint64_t nAddr)
{
    networkAddress_ = nAddr;

    while (networkAddress_ > 0xFFFFFFLL)
        networkAddress_ /= 0x100;

    while (networkAddress_ * 0x100LL < 0xFFFFFFLL)
        networkAddress_ *= 0x100LL;

    networkAddress_ *= 0x10000LL;
    serverAddress_ = networkAddress_ + 0xFFFFLL;
    clientAddress_ = networkAddress_ + clientId;
}

uint64_t  Nrf24DcClient::networkAddr()
{
    return networkAddress_;
}

void Nrf24DcClient::setRFDataRate(const rf24_datarate_e speed)
{
    driver.setDataRate(speed);
    uint8_t delay;
    uint8_t number;

    switch (speed)
    {
    case RF24_250KBPS:
        delay = 5;
        number = 15;
        break;

    case RF24_1MBPS:
        delay = 4;
        number = 15;
        break;

    case RF24_2MBPS:
        delay = 3;
        number = 15;
        break;

    default:
        delay = 4;
        number = 15;
        break;
    }

    receivingEndSessionMsgTimeout_ = delay * (number + 1) * 0.250;
    driver.setRetries(delay, number);
}

void Nrf24DcClient::setRF_PA_Level(uint8_t level)
{
    driver.setPALevel(level);
}

void Nrf24DcClient::setDeviceId(int16_t id)
{
    clientAddress_ = networkAddress_ + id;
    clientId = id;
}

int16_t Nrf24DcClient::deviceId()
{
    return clientId;
}

uint64_t Nrf24DcClient::clientAddress()
{
    return clientAddress_;
}

void Nrf24DcClient::setSessionTimeout(int32_t timeout)
{
    sessionTimeout_ = timeout;
}

int32_t Nrf24DcClient::sessionTimeout()
{
    return sessionTimeout_;
}

void Nrf24DcClient::setKeepaliveTimeout(int16_t timeout)
{
    keepAliveTimeout_ = timeout;
}

int16_t Nrf24DcClient::keepalveTimeout()
{
    return keepAliveTimeout_;
}

void Nrf24DcClient::setTimeoutBeforeStartSearchingNetwork(int32_t timeout_mSec)
{
    timeoutBeforeStartSearchingNetwork_ = timeout_mSec;
}

int32_t Nrf24DcClient::timeoutBeforeStartSearchingNetwork()
{
    return timeoutBeforeStartSearchingNetwork_;
}

bool Nrf24DcClient::addCommand(AbstractClientCommand * cmd)
{
    if (cmdCount_ >= DC_NUMBER_OF_COMMANDS)
        return false;

    cmdArray_[cmdCount_] = cmd;
    ++cmdCount_;

    return true;
}

bool Nrf24DcClient::sendDataToServer()
{
    if (!dataToSendLength_)
        return false;
    else
        return write(dataToSend_, dataToSendLength_);
}

bool Nrf24DcClient::receiveDataFromServer(int16_t timeout)
{
    volatile auto startTime = millis();
    while ((millis() - startTime) <= timeout)
    {
        if (driver.available())
        {
            receivedDataLength_ = driver.getDynamicPayloadSize();
            read(receivedData_, receivedDataLength_);
            //Serial.print("qqqq-");
            //Serial.println(receivedDataLength_);
            return true;
        }
        yield();
    }

    return false;
}

bool Nrf24DcClient::receiveStartSessionTag(int16_t timeout)
{
    if (timeout == -1)
        timeout = this->sessionTimeout_;

    auto recvStartTime = millis();

    while ((millis() - recvStartTime) <= timeout)
    {
        if (driver.available())
        {
            uint8_t ssTagLen = strlen(DC_START_SESSION_TAG_STR);
            uint8_t rLen = driver.getDynamicPayloadSize();
            read(buffer_, rLen);
            waitEndSessionTag();

            if (rLen >= ssTagLen && strncmp((char*)buffer_, DC_START_SESSION_TAG_STR, ssTagLen) == 0)
            {
                if (rLen > ssTagLen)
                {
                    //Serial.println(ssTagLen);
                    //Serial.print("Buffer_ = ");
                    //Serial.println((char*)buffer_);
                    memset(receivedData_, 0, DC_MAX_SIZE_OF_RF_PACKET);
                    receivedDataLength_ = rLen - ssTagLen;
                    memcpy(receivedData_, buffer_ + ssTagLen, receivedDataLength_);
                }
                return true;
            }
        }
        yield();
    }

    return false;
}

void Nrf24DcClient::clientLoop()
{
    keepServer();
}

void Nrf24DcClient::keepServer()
{
    if (lastKeepaliveTime_ == 0)
        lastKeepaliveTime_ = millis();

    uint32_t currentTime = millis();

    if ((currentTime - lastKeepaliveTime_) > (timeoutBeforeStartSearchingNetwork()*1.5))
    {
        Serial.println("Keepalive timer overflow. Starting to look for server.");

        //scanChannels();
        //for (int ch = 0; ch < DC_CHANNEL_COUNT; ++ch)
        //    Serial.print(channelsActivity_[ch]);

        for (int ch = 0; ch < DC_CHANNEL_COUNT; ++ch)
        {
            setWorkChannel(ch);
            isBroadcastMode_ = false;
            if (waitForKeepaliveMsg(keepAliveTimeout_*1.2))
            {
                //setWorkChannel(ch);
                Serial.print("Found new server at ");
                Serial.println(ch);
                break;
            }

            yield();
        }

    }

}

void Nrf24DcClient::resetKeepAliveTimer()
{
    lastKeepaliveTime_ = millis();
}

void Nrf24DcClient::setEncryption(bool flag)
{
    isEncrypt_ = flag;
}

bool Nrf24DcClient::encryption()
{
    return isEncrypt_;
}

void Nrf24DcClient::setEcryptKeyPointer(uint8_t *pointer, uint8_t len)
{
    keyPtr_ = pointer;
    keySize_ = len;
}

void Nrf24DcClient::encryptMsg(uint8_t *msg, uint8_t size)
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

void Nrf24DcClient::decryptMsg(uint8_t * msg, uint8_t size)
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

void Nrf24DcClient::sendEndSessionTag()
{
    driver.setAutoAck(false);
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    driver.setAutoAck(true);
}


int8_t Nrf24DcClient::bytePos(uint8_t searchedByte, uint8_t * data, uint8_t len)
{
    int8_t pos = -1;

    for (int i = 0; i < len; ++i)
        if (*(data + i) == searchedByte)
        {
            pos = i;
            break;
        }

    return pos;
}

void Nrf24DcClient::prepareSesionBuffers_()
{
    memset(receivedData_, 0, 32);
    receivedDataLength_ = 0;
}

bool Nrf24DcClient::isChannelBussy(uint8_t channel)
{
    driver.setChannel(channel);
    driver.startListening();
    delayMicroseconds(300);
    bool res = driver.testRPD();
    driver.stopListening();
    return res;
}

void Nrf24DcClient::scanChannels()
{
    memset(channelsActivity_, 0, DC_CHANNEL_COUNT);

    for (int rep = 0; rep < DC_REPS_COUNT; ++rep)
    {
        for (int ch = 0; ch < DC_CHANNEL_COUNT; ++ch)
        {
            if (isChannelBussy(ch))
                ++channelsActivity_[ch];

            yield();
        }
    }
}

bool Nrf24DcClient::waitForKeepaliveMsg(int16_t timeout)
{
    uint32_t startTime = millis();


    while ((millis() - startTime) <= timeout)
    {
        uint8_t res = listenBroadcast();

        if (res == DC_KEEPALIVE)
        {
            return true;
        }
    }

    //driver.printDetails();


    return false;
}

void Nrf24DcClient::waitEndSessionTag()
{
    auto startTime = millis();

    //Serial.println(receivingEndSessionMsgTimeout_);
    while ((millis() - startTime) < receivingEndSessionMsgTimeout_)
    {
        if (driver.available())
        {
            uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET] = { 0 };
            read(buf, DC_MAX_SIZE_OF_RF_PACKET);
            break;
        }
    }
}

void Nrf24DcClient::read(void * buf, uint8_t len)
{
    driver.read(buf, len);

    if (isEncrypt_)
    {
        uint8_t *msgPtr = (uint8_t*)buf;
        decryptMsg(msgPtr, len);
    }

}

bool Nrf24DcClient::write(const void * buf, const uint8_t len)
{
    uint8_t *msgPtr = (uint8_t*)buf;
    uint8_t tmpBuf[32] = { 0 };

    if (isEncrypt_)
    {
        memcpy(tmpBuf, msgPtr, len);
        msgPtr = &tmpBuf[0];
        encryptMsg(msgPtr, len);
    }

    return driver.write(msgPtr, len);
}

int8_t Nrf24DcClient::getReceivedData(void * buffer, int8_t maxLen)
{
    if (maxLen > receivedDataLength_)
        maxLen = receivedDataLength_;
    //Serial.print("len = ");
    //Serial.println(maxLen);

    if (buffer)
    {
        memcpy(buffer, receivedData_, maxLen);
    }

    return maxLen;
}

uint8_t Nrf24DcClient::listenBroadcast()
{
    if (!isBroadcastMode_)
    {
        //Serial.print(F("Set broadcast mode "));
        //Serial.println(mon.free());
        //Serial.println(broadcastChannel());

        driver.stopListening();

        driver.setChannel(broadcastChannel());
        driver.setAutoAck(false);
        driver.openReadingPipe(1, serverAddress_);
        driver.startListening();
        isBroadcastMode_ = true;
    }

    int8_t receivedPacketSize = 0;

    if (driver.available())
    {
        receivedPacketSize = driver.getDynamicPayloadSize();
        memset(buffer_, 0, 32);
        read(buffer_, 32);

        //Serial.print(F(" received packet "));
        //Serial.println((char*)buffer_);
    }

    if (receivedPacketSize >= 1)
    {
        //delay(1000);
        //Serial.print(F(" received packet "));
        //Serial.println(receivedPacketSize);
        //Serial.println((char*)&buffer_[0]);
        int8_t spliterPos = 0;
        spliterPos = bytePos((uint8_t)':', buffer_, receivedPacketSize);

        if (spliterPos != -1)
        {
            char command[32] = { 0 };
            char parametr[32] = { 0 };
            strncpy(command, (char*)buffer_, spliterPos);
            ++spliterPos;
            strncpy(parametr, (char*)buffer_ + spliterPos, receivedPacketSize - spliterPos);
            
            //Serial.print(F("command "));
            //Serial.println((char*)buffer_);
            //Serial.println(command);
            //Serial.println(parametr);

            for (int i = 0; i < cmdCount_; ++i)
                if (cmdArray_[i]->isCommand(String(command)))
                {
                    //Serial.println(F("Satrt command ssh"));
                    driver.stopListening();
                    isBroadcastMode_ = false;

                    cmdArray_[i]->run(String(parametr));
                    resetKeepAliveTimer();

                    return cmdArray_[i]->returnCode();
                }

            //Serial.print("Unknow command - ");
            //Serial.println(command);
        }
    }

    return 0x00;
}

bool Nrf24DcClient::startSession()
{
    isBroadcastMode_ = false;
    driver.setAutoAck(true);
    driver.setChannel(workChannel());
    prepareSesionBuffers_();
    return true;
}

void Nrf24DcClient::putDataForSend(void * buffer, int8_t len)
{
    dataToSendLength_ = len;

    if (dataToSendLength_ > 32)
        dataToSendLength_ = 32;

    memcpy(dataToSend_, buffer, dataToSendLength_);
}

