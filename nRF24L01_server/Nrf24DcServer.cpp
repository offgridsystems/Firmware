#include "Nrf24DcServer.h"


Nrf24DcServer::Nrf24DcServer(rfDriver& drv)
    : driver_(drv)
{
    currnetDeviceCount_ = 0;
    setSessionTimeout(DC_DEFAULT_SESSION_TIMEOUT);
    setReadingTimeout(DC_DEFAULT_READING_TIMEOUT);
    setKeepAliveTimeout(DC_DEFAULT_KEEPALIVE_TIMEOUT);
}

uint16_t Nrf24DcServer::lookForClient(int timeout)
{
    currnetDeviceCount_ = 0;
    sendRequestForLookup(timeout);
    driver_.txStandBy();
    delay(500);
    uint64_t startSessionTime = millis();

    for (int id = 1; id < 1024; ++id)
    {

        //Serial.println(id);
        if ((millis() - startSessionTime) > timeout)
        {
            Serial.println("Lookup session timeout.");
            //Serial.println(id);
            break;
        }

        uint64_t clientAddr = id + networkAddr();

        setSingleClientMode(clientAddr);
        driver_.openWritingPipe(clientAddr);

        if (sendStartSesionTag())
        {
            addClient(id);
            //driver_.printDetails();
            //Serial.println(id);


            if (handledClientsCount() >= DC_MAX_CLIENT_NUMBER)
                break;
        }
        driver_.flush_tx();

        yield();
    }

    return handledClientsCount();
}

bool Nrf24DcServer::init()
{
    bool res = driver_.begin();
    driver_.setAddressWidth(5);           // must be always 5
    driver_.setAutoAck(false);
    driver_.setCRCLength(RF24_CRC_16);     // number of bits of CRC, can be RF24_CRC_8 or RF24_CRC_16
    driver_.enableDynamicPayloads();      // alway use this options
    setRFDataRate(RF24_1MBPS);      // the communication speed (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS)

    return res;
}

void Nrf24DcServer::setWorkChannel(uint8_t ch)
{
    workChannel_ = ch;
}

void Nrf24DcServer::setBroadcastChannel(uint8_t ch)
{
    //broadcastChannel_ = ch;
    workChannel_ = ch;
}

uint8_t Nrf24DcServer::workChannel()
{
    return workChannel_;
}

uint8_t Nrf24DcServer::broadcastChannel()
{
    //return broadcastChannel_;
    return workChannel_;
}

void Nrf24DcServer::setNetworkAddr(uint64_t nAddr)
{
    networkAddress_ = nAddr;

    while (networkAddress_ > 0xFFFFFFLL)
        networkAddress_ /= 0x100;

    while (networkAddress_ * 0x100LL < 0xFFFFFFLL)
        networkAddress_ *= 0x100LL;

    networkAddress_ *= 0x10000LL;
    serverAddress_ = networkAddress_ + 0xFFFFLL;
}

uint64_t  Nrf24DcServer::networkAddr()
{
    return networkAddress_;
}

void Nrf24DcServer::setRFDataRate(const rf24_datarate_e speed)
{
    driver_.setDataRate(speed);
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
    driver_.setRetries(delay, number);
}

void Nrf24DcServer::setRF_PA_Level(uint8_t level)
{
    driver_.setPALevel(level);
}

int Nrf24DcServer::handledClientsCount()
{
    return currnetDeviceCount_;
}

bool Nrf24DcServer::isDeviceHandled(int16_t id)
{
    for (int i = 0; i < currnetDeviceCount_; ++i)
    {
        if (id == handledDevicesId_[i])
            return true;
    }

    return false;

}

void Nrf24DcServer::setSessionTimeout(uint16_t timeOut)
{
    sessionTimeout_ = timeOut;
}

uint16_t Nrf24DcServer::sessionTimeout()
{
    return sessionTimeout_;
}

void Nrf24DcServer::setReadingTimeout(uint16_t timeout)
{
    readingTimeout_ = timeout;
}

uint16_t Nrf24DcServer::readingTimeout()
{
    return readingTimeout_;
}

void Nrf24DcServer::setKeepAliveTimeout(int16_t timeout)
{
    keepAliveTimeout_ = timeout;
}

int16_t Nrf24DcServer::keepAliveTimeout() const
{
    return keepAliveTimeout_;
}

bool Nrf24DcServer::addClient(int16_t deviceId)
{
    if (currnetDeviceCount_ < DC_MAX_CLIENT_NUMBER && !isDeviceHandled(deviceId))
    {
        //Serial.println(deviceId);
        handledDevicesId_[currnetDeviceCount_] = deviceId;
        ++currnetDeviceCount_;
        return true;
    }
    else
        return false;
}

bool Nrf24DcServer::addClientByRange(int16_t from, int16_t count)
{
    //Serial.println(count);
    if ((currnetDeviceCount_ + count) <= DC_MAX_CLIENT_NUMBER)
    {
        for (int i = 0; i < count; ++i)
        {
            //Serial.println(from+i);
            addClient(from + i);
        }
        return true;
    }
    else
        return false;

}

bool Nrf24DcServer::removeClient(int16_t clientId)
{
    int16_t idPos = -1;

    for (int i = 0; i < currnetDeviceCount_; ++i)
    {
        if (handledDevicesId_[i] == clientId)
        {
            idPos = i;
            break;
        }
    }

    if (idPos == -1)
    {
        return false;
    }

    for (int i = idPos + 1; i < currnetDeviceCount_; ++i)
    {
        handledDevicesId_[i - 1] = handledDevicesId_[i];
    }

    --currnetDeviceCount_;
    return true;
}

bool Nrf24DcServer::removeClientsByRange(int16_t from, int16_t count)
{
    int16_t last = from + count;
    for (int i = from; i < last; ++i)
    {
        removeClient(i);
    }

    return true;
}

bool Nrf24DcServer::setBroadcastMode()
{
    driver_.flush_tx();
    driver_.setChannel(broadcastChannel());
    driver_.setAutoAck(false);


    return true;
}

bool Nrf24DcServer::setSingleClientMode(uint64_t clientAddr)
{

    //long long clientAddress = networkAddress_ + device_id;
    //driver_.powerDown();
    //driver_.powerUp();

    driver_.stopListening();

    driver_.setAutoAck(true);
    driver_.setChannel(workChannel());
    driver_.openReadingPipe(0, clientAddr);
    driver_.openReadingPipe(1, clientAddr);
    driver_.openWritingPipe(clientAddr);

    return true;
}

int16_t Nrf24DcServer::clientIdAt(int16_t index)
{
    if (index >= 0 && index < currnetDeviceCount_)
        return handledDevicesId_[index];
    else
        return -1;
}

int16_t Nrf24DcServer::clientIndexById(int16_t id)
{
    uint16_t index = -1;
    for (int i = 0; i < currnetDeviceCount_; ++i)
        if (handledDevicesId_[i] == id)
        {
            index = i;
            break;
        }

    return index;
}


bool Nrf24DcServer::sendRequestForSession()
{
    String command = "ssch:";
    command += String(sessionTimeout());


    return this->sendBroadcastRequestCommand(command);
}

bool Nrf24DcServer::sendRequestForLookup(int timeout)
{
    String command = "lookup:";
    command += String(timeout);


    return this->sendBroadcastRequestCommand(command);
}

bool Nrf24DcServer::sendStartSesionTag(uint8_t times)
{
    uint8_t buf[] = DC_START_SESSION_TAG_STR;
    bool res = false;

    for (int k = 0; k < times; ++k)
    {
        res = write(buf, strlen(DC_START_SESSION_TAG_STR));

        if (res)
        {
            sendEndSessionTag();
            break;
        }

        yield();
    }

    driver_.txStandBy();
    driver_.flush_tx();

    return res;

}

bool Nrf24DcServer::sendStartSesionTagWithData(const uint8_t * dataPtr, const uint8_t len)
{
    uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET];
    strcpy((char*)buf, DC_START_SESSION_TAG_STR);
    uint8_t ssMsgLen = strlen(DC_START_SESSION_TAG_STR);
    uint8_t msgLen = ssMsgLen + len;
    uint8_t bytesToCopy = len;

    if (msgLen > DC_MAX_SIZE_OF_RF_PACKET)
    {
        msgLen = DC_MAX_SIZE_OF_RF_PACKET;
        bytesToCopy = msgLen - ssMsgLen;
    }


    memcpy(buf + ssMsgLen, dataPtr, bytesToCopy);

    //Serial.println(ssMsgLen);
    //Serial.println(msgLen);
    //Serial.println(bytesToCopy);
    //Serial.println((char*)buf);
    bool res = write(buf, msgLen);
    sendEndSessionTag();

    driver_.txStandBy();
    driver_.flush_tx();

    return res;
}

int16_t Nrf24DcServer::startSession()
{
    prepareArrays();
    sendRequestForSession();
    //delay(3);
    //driver_.txStandBy();

    uint64_t startSessionTime = millis();
    int16_t deviceCount = handledClientsCount();
    uint8_t rLen;
    //Serial.println(deviceCount);

    int i = -1;
    int numberOfHandledDevices = 0;

    while (true)
    {
        ++i;

        if (i == deviceCount)
            i = 0;

        yield();

        if (numberOfHandledDevices == deviceCount)
            break;

        if (commsStatus_[i])
            continue;

        if ((millis() - startSessionTime) > sessionTimeout_)
        {
            Serial.println("Session timeout.");
            break;
        }

        driver_.flush_tx();
        driver_.txStandBy();

        uint64_t clientAddr = clientIdAt(i) + networkAddr();

        setSingleClientMode(clientAddr);

        bool res = sendStartSesionTagWithData(dataBufferToClients_[i], DC_MAX_SIZE_OF_DATA_FOR_SENDING);

        if (!res)
        {
            //Serial.println("Start not sended ");
            //driver_.printDetails();
            continue;
        }

        yield();
        uint64_t  recvStartTime = millis();

        bool isRecvData = false;
        rLen = DC_MAX_SIZE_OF_RF_PACKET;
        driver_.startListening();

        while ((millis() - recvStartTime) <= readingTimeout_)
        {
            yield();
            if (driver_.available())
            {
                rLen = driver_.getDynamicPayloadSize();
                memset(dataBufferFromClients_[i], 0, DC_MAX_SIZE_OF_RF_PACKET);
                read(dataBufferFromClients_[i], rLen);
                dataBufferFromClientsSize_[i] = rLen;
                isRecvData = true;
                waitEndSessionTag();
                break;
            }
        }

        driver_.stopListening();

        if (!isRecvData)
        {
            //Serial.println("Data from client not received");
            continue;
        }

        commsStatus_[i] = 1;
        ++numberOfHandledDevices;

    }

    return numberOfHandledDevices;

}

uint8_t * Nrf24DcServer::receivedDataById(uint16_t id)
{
    int16_t idx = clientIndexById(id);

    if (idx == -1)
        return nullptr;
    else
        return dataBufferFromClients_[idx];
}

uint8_t * Nrf24DcServer::receivedDataByIndex(uint16_t idx)
{
    if (idx >= currnetDeviceCount_)
        return nullptr;
    else
        return dataBufferFromClients_[idx];
}

uint8_t Nrf24DcServer::lenfgthOfReceivedBufferByIndex(uint16_t idx)
{
    if (idx >= currnetDeviceCount_)
        return -1;
    else
        return dataBufferFromClientsSize_[idx];
}

uint8_t Nrf24DcServer::lenfgthOfReceivedBufferById(uint16_t id)
{
    int16_t idx = clientIndexById(id);

    if (idx == -1)
        return -1;
    else
        return lenfgthOfReceivedBufferByIndex(idx);
}

void Nrf24DcServer::putSendedData(int16_t idx, const void * data, uint8_t len)
{
    if (idx >= currnetDeviceCount_ && idx != -1)
        return;
    else
    {
        if (len > DC_MAX_SIZE_OF_DATA_FOR_SENDING)
            len = DC_MAX_SIZE_OF_DATA_FOR_SENDING;

        if (idx == -1)
        {
            for (int i = 0; i < DC_MAX_CLIENT_NUMBER; ++i)
            {
                memset(dataBufferToClients_[i], 0, DC_MAX_SIZE_OF_DATA_FOR_SENDING);
                memcpy(&dataBufferToClients_[i][0], (uint8_t*)data, len);
            }
        }
        else
        {
            memset(dataBufferToClients_[idx], 0, DC_MAX_SIZE_OF_DATA_FOR_SENDING);
            memcpy(dataBufferToClients_[idx], data, len);
        }
    }
}

void Nrf24DcServer::testChannel()
{
    if (isChannelBussy(workChannel()))
    {
        int16_t freeChannel = lookForFreeChannel();

        if (freeChannel != -1)
        {
            setWorkChannel(freeChannel);
            Serial.print("Found new free channel ");
            Serial.println(freeChannel);
        }
    }

    return;
}

void Nrf24DcServer::sendKeepAliveMsg()
{
    static uint32_t startTime = millis();
    uint32_t currentTime = millis();
    //Serial.println(broadcastChannel());
    if ((currentTime - startTime) > keepAliveTimeout())
    {
        startTime = currentTime;
        String cmd = "keepAlive:";
        cmd += DC_DEFAULT_OFFLINE_TIMEOUT;
        sendBroadcastRequestCommand(cmd);
    }
}

void Nrf24DcServer::serverLoop()
{
    testChannel();
    sendKeepAliveMsg();
}

void Nrf24DcServer::setEncryption(bool flag)
{
    isEncrypt_ = flag;
}

bool Nrf24DcServer::encryption()
{
    return isEncrypt_;
}

void Nrf24DcServer::setEcryptKeyPointer(uint8_t * pointer, uint8_t len)
{
    keyPtr_ = pointer;
    keySize_ = len;
}

void Nrf24DcServer::encryptMsg(uint8_t * msg, uint8_t size)
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

void Nrf24DcServer::decryptMsg(uint8_t * msg, uint8_t size)
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

void Nrf24DcServer::sendEndSessionTag()
{
    delayMicroseconds(135);
    driver_.setAutoAck(false);
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    write(DC_END_SESSION_TAG_STR, strlen(DC_END_SESSION_TAG_STR));
    driver_.setAutoAck(true);
}

bool Nrf24DcServer::isChannelBussy(uint8_t channel)
{
    driver_.setChannel(channel);
    driver_.startListening();
    delayMicroseconds(225);
    bool res = driver_.testCarrier();
    driver_.stopListening();
    return res;
}

void Nrf24DcServer::scanChannels()
{
    memset(channelsActivity_, 0, DC_CHANNEL_COUNT);

    for (int rep = 0; rep < DC_REPS_COUNT; ++rep)
    {
        for (int ch = 0; ch < DC_CHANNEL_COUNT; ++ch)
        {
            if (isChannelBussy(ch))
                ++channelsActivity_[ch];
        }
    }
}

int16_t Nrf24DcServer::lookForFreeChannel()
{
    scanChannels();

    for (int r = 3; r > 0; --r)
    {
        for (int i = workChannel() + 1; i < DC_CHANNEL_COUNT; ++i)
        {
            if (isChannelFreeRadius(i, r))
            {
                return i;
            }
        }


        for (int i = 0; i < workChannel(); ++i)
        {
            if (isChannelFreeRadius(i, r))
            {
                return i;
            }

        }
    }
    return -1;
}

bool Nrf24DcServer::isChannelFreeRadius(int8_t channel, int8_t radius)
{
    bool res = true;
    int16_t left = channel - radius;
    int16_t right = channel + radius;

    if (left < 0)
        left = 0;

    if (right >= DC_CHANNEL_COUNT)
        right = DC_CHANNEL_COUNT - 1;

    for (int ch = left; ch <= right; ++ch)
    {
        
        if (channelsActivity_[ch] > 2)
        {
            res = false;
            break;
        }
    }

    return res;
}

void Nrf24DcServer::waitEndSessionTag()
{
    auto startTime = millis();

    while ((millis() - startTime) < receivingEndSessionMsgTimeout_)
    {
        if (driver_.available())
        {
            uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET] = { 0 };
            read(buf, DC_MAX_SIZE_OF_RF_PACKET);
            break;
        }
    }
}

void Nrf24DcServer::read(void * buf, uint8_t len)
{
    driver_.read(buf, len);
    //Serial.println((char*)buf);

    if (isEncrypt_)
    {
        uint8_t *msgPtr = (uint8_t*)buf;
        decryptMsg(msgPtr, len);
    }

    //Serial.println(len);
    //Serial.println((char*)buf);
    //Serial.println(len);


}

bool Nrf24DcServer::write(const void * buf, const uint8_t len)
{
    uint8_t *msgPtr = (uint8_t*)buf;
    uint8_t tmpBuf[32] = { 0 };

    if (isEncrypt_)
    {
        memcpy(tmpBuf, msgPtr, len);
        msgPtr = &tmpBuf[0];
        encryptMsg(msgPtr, len);
    }

    bool res = driver_.write(msgPtr, len);
    driver_.txStandBy();
    return res;
}

void Nrf24DcServer::prepareArrays()
{
    for (int i = 0; i < DC_MAX_CLIENT_NUMBER; ++i)
    {
        commsStatus_[i] = 0;
        dataBufferFromClientsSize_[i] = 0;
    }

}

bool Nrf24DcServer::sendBroadcastRequestCommand(String command)
{
    driver_.stopListening();
    //driver_.powerDown();
    //driver_.powerUp();

    if (!setBroadcastMode())
        return false;

    //Serial.println(driver_.getChannel());

//    uint64_t aa = networkAddr();
    //uint64_t aa = serverAddress_;
    //uint32_t hb = *(((uint32_t*)&aa));
    //uint32_t lb = *(((uint32_t*)&aa) + 1);
    //Serial.print(lb, HEX);
    //Serial.println(hb, HEX);


    //Serial.println(driver_.getDataRate());

    yield();
    driver_.openWritingPipe(serverAddress_);

    yield();
    //driver.printRegisters();

    //Serial.println("loop to broadcast request for data");
    for (int i = 0; i < DC_NUMBER_OF_BROADCAST_REQUESTS; ++i) {
        const uint8_t len = command.length() > 32 ? 32 : command.length();
        uint8_t buf[32];
        memcpy(buf, command.c_str(), len);
        write(buf, len);
        yield();
    }
    //driver_.printDetails();

    return true;
}

