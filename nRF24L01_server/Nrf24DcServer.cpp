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
    uint64_t startSessionTime = millis();
    int16_t lookupTime = this->keepAliveTimeout()*1.2*DC_CHANNEL_COUNT;
    uint64_t endTime = startSessionTime + ((this->sessionTimeout() > lookupTime) ? this->sessionTimeout() : lookupTime) * 1.2;
    Serial.print("Waiting ");
    Serial.print((int)(endTime - startSessionTime));
    Serial.println(" msec ");

    while (millis() < endTime)
    {
        sendKeepAliveMsg();
        myDelay(keepAliveTimeout());
        yield();
    }

    Serial.print("Scaning RF (wait ");
    Serial.print(timeout);
    Serial.println(" msec).");

    currnetDeviceCount_ = 0;
    sendRequestForLookup(timeout);
    myDelay(500);
    bool trash;
    driver_.whatHappened(trash, trash, trash);
    //driver_.printDetails();
    //driver_.txStandBy();
    //driver_.stopListening();
    //digitalWrite(9, LOW);
    //myDelay(1);
    //driver_.flush_tx();
    startSessionTime = millis();

    for (int id = 1; id <= DC_MAX_CLIENT_ID; ++id)
    {

        //Serial.println(id);
        if ((millis() - startSessionTime) > timeout)
        {
            Serial.println("Lookup session timeout.");
            //Serial.println(id);
            break;
        }

        //uint64_t clientAddr = (uint64_t)id + networkAddr();

        setSingleClientMode(id);
        //driver_.openWritingPipe(clientAddr);

        if (sendStartSesionTag())
        {
            addClient(id);
            //driver_.printDetails();
            Serial.println(id);


            if (handledClientsCount() >= DC_MAX_CLIENT_NUMBER)
                break;
        }
        //driver_.flush_tx();

        yield();
    }

    return handledClientsCount();
}

uint16_t Nrf24DcServer::lookForClientWithDifferentPALevel(int timeout)
{
    uint16_t resN = 0;
    uint8_t resPa = 0;

    for (int i = 0; i < 4; ++i) {
        Serial.print(F("Looking for clients with PA level "));
        Serial.println(i);
        driver_.stopListening();
        this->setRF_PA_Level(i);
        this->setPaLevelForClient(-1, i);
        uint16_t n = lookForClient(timeout);

        Serial.print(F("Found "));
        Serial.print(n);
        Serial.println(F(" clients."));

        if (n > resN)
        {
            resN = n;
            resPa = i;
        }
    }

    if (resN) {
        Serial.print(F("Looking for clients with PA level "));
        Serial.println(resPa);
        this->setPaLevelForClient(-1, resPa);
        this->setRF_PA_Level(resPa);
        uint16_t resN = lookForClient(timeout);
    }

    return resN;
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
        number = 10;
        break;

    case RF24_1MBPS:
        delay = 4;
        number = 10;
        break;

    case RF24_2MBPS:
        delay = 3;
        number = 10;
        break;

    default:
        delay = 4;
        number = 10;
        break;
    }

    receivingEndSessionMsgTimeout_ = delay * (number + 1) * 0.250;
    driver_.setRetries(delay, number);
}

void Nrf24DcServer::setRF_PA_Level(uint8_t level)
{
    rfPaLevel_ = level;
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

bool Nrf24DcServer::setBroadcastMode(int16_t id)
{
    //driver_.flush_tx();
    driver_.stopListening();

    if (id == -1)
        driver_.setPALevel(this->rfPaLevel_);
    else
        driver_.setPALevel(this->txPaLevel_[this->clientIndexById(id)]);

    driver_.setChannel(broadcastChannel());
    driver_.setAutoAck(false);
    return true;
}

bool Nrf24DcServer::setSingleClientMode(uint64_t client_id)
{

    uint64_t clientAddr = networkAddress_ + client_id;
    //driver_.powerDown();
    //driver_.powerUp();

    driver_.stopListening();

    driver_.setAutoAck(true);
    driver_.setChannel(workChannel());
    driver_.setRetries(3, 4);

    uint8_t paLevel;
    int16_t idx = this->clientIndexById(client_id);
    if (idx != -1)
        paLevel = this->txPaLevel_[idx];
    else
        paLevel = this->rfPaLevel_;

    //Serial.print("PA ");
    //Serial.println(paLevel);
    driver_.setPALevel(paLevel);
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


bool Nrf24DcServer::sendRequestForSession(int16_t id, uint8_t *data, uint8_t dataLen)
{
    String command = "ssch:";
    command += String(id);
    command += ",";
    for (int i = 0; i < dataLen; ++i)
        command.append(data[i]);

    //Serial.println(command);
    return this->sendBroadcastRequestCommand(command, id);
}

bool Nrf24DcServer::sendRequestForLookup(int32_t timeout)
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
            //sendEndSessionTag();
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
    int16_t deviceCount = handledClientsCount();

    if (deviceCount == 0)
        return 0;

    prepareArrays();
    uint64_t startSessionTime = millis();
    uint8_t rLen;
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

        if (!this->singleClientSession(clientIdAt(i), dataBufferToClients_[i], DC_MAX_SIZE_OF_DATA_FOR_SENDING, dataBufferFromClients_[i], dataBufferFromClientsSize_[i]))
        {
            //Serial.println("Data from client not received");
            continue;
        }

        commsStatus_[i] = 1;
        ++numberOfHandledDevices;
    }

    delay(1);
    return numberOfHandledDevices;

}

bool Nrf24DcServer::singleClientSession(const int16_t id, const void * dataToSend, int16_t dataToSendLen, void * receivedData, uint8_t &receivedDataLen)
{
    int16_t clientIdx = clientIndexById(id);
    if (clientIdx == -1)
        return false;

    sendRequestForSession(id, (uint8_t*)dataToSend, dataToSendLen);
    //uint64_t clientAddr = clientIdAt(i) + networkAddr();

    setSingleClientMode(id);
    driver_.startListening();

    yield();
    uint64_t  recvStartTime = millis();

    bool isRecvData = false;
    uint8_t rLen = DC_MAX_SIZE_OF_RF_PACKET;

    while ((millis() - recvStartTime) <= DC_DEFAULT_READING_TIMEOUT)
    {
        yield();
        if (driver_.available())
        {
            rLen = driver_.getDynamicPayloadSize();
            memset(dataBufferFromClients_[clientIdx], 0, DC_MAX_SIZE_OF_RF_PACKET);
            read(dataBufferFromClients_[clientIdx], rLen);
            receivedDataLen = rLen;
            dataBufferFromClientsSize_[clientIdx] = rLen;
            isRecvData = true;
            waitEndSessionTag(1);
            break;
        }
    }

    driver_.stopListening();
    return isRecvData;
}

int8_t Nrf24DcServer::tuneTxPa(int16_t clientId)
{
    String cmd = "tuneTxPa:";
    cmd += clientId;
    Serial.println(cmd);
    sendBroadcastRequestCommand(cmd);

    setSingleClientMode(clientId);
    driver_.startListening();
    int32_t results[4] = { 0 };
    uint8_t buf[DC_MAX_SIZE_OF_RF_PACKET] = { 0 };

    for (int i = 0; i < 4; ++i) {
        auto startTime = millis();
        int32_t r = 0;
        //driver_.printDetails();

        while ((millis() - startTime) < DC_TUNNIG_PA_TIMEOUT) {
            if (driver_.available()) {
                driver_.read(buf, DC_MAX_SIZE_OF_RF_PACKET);
                ++r;
            }
        }
        results[i] = r;
    }

    Serial.println("Results TX: ");

    for (int i = 0; i < 4; ++i) {
        Serial.println(results[i]);
    }



    return 0;
}

int8_t Nrf24DcServer::tuneRxPa(int16_t clientId)
{

    String cmd = "tuneRxPa:";
    cmd += clientId;
    Serial.println(cmd);
    sendBroadcastRequestCommand(cmd);

    setSingleClientMode(clientId);
    driver_.stopListening();
    int32_t results[4] = { 0 };
    int32_t resultsFalls[4] = { 0 };
    uint8_t buf[32] = { 0 };

    for (int i = 0; i < 4; ++i) {
        auto startTime = millis();
        driver_.stopListening();
        driver_.setPALevel(i);
        int res = 0;
        int fall = 0;
        //client_->driver.printDetails();
        while ((millis() - startTime) < DC_TUNNIG_PA_TIMEOUT) {
            if (driver_.write("12345678909876543212345678909876", 32))
                ++res;
            else
                ++fall;
        }

        results[i] = res;
        resultsFalls[i] = fall;
        driver_.txStandBy();
    }

    Serial.println("Results Rx: ");
    for (int i = 0; i < 4; ++i) {
        Serial.println(results[i]);
        Serial.println(resultsFalls[i]);
    }

    uint8_t pa = getBestPaLevelResult(results, resultsFalls, 4);
    Serial.print("New PA level: ");
    Serial.println(pa);

    this->setPaLevelForClient(this->clientIndexById(clientId), pa);
    this->setRF_PA_Level(pa);
    return pa;
}

void Nrf24DcServer::tunePA()
{
    for (int i = 0; i < this->handledClientsCount(); ++i)
    {
        this->tuneTxPa(this->clientIdAt(i));
        this->sendKeepAliveMsg();
        myDelay(100);
        this->tuneRxPa(this->clientIdAt(i));
        this->sendKeepAliveMsg();
        myDelay(100);
        this->tuneTxPa(this->clientIdAt(i));
        this->sendKeepAliveMsg();
        myDelay(100);
        this->tuneRxPa(this->clientIdAt(i));
        this->sendKeepAliveMsg();
        myDelay(100);

        uint8_t paCount[4] = { 0 };
        for (int i = 0; i < handledClientsCount(); ++i) {
            ++paCount[this->txPaLevel_[i]];
        }

        int16_t maxCount = paCount[0];
        uint8_t paLevel = 0;
        for (int i = 1; i < 4; ++i) {
            if (paCount[i] > maxCount)
            {
                maxCount = paCount[i];
                paLevel = i;
            }
        }

        setRF_PA_Level(paLevel);
    }
}

void Nrf24DcServer::changeChannelOnClients(uint8_t newChannel)
{
    String cmd = "cc:";
    cmd += newChannel;

    sendBroadcastRequestCommand(cmd);
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
            Serial.print("Found new free channel ");
            Serial.println(freeChannel);

            for (int i = 0; i < 10; ++i)
                changeChannelOnClients(freeChannel);

            setWorkChannel(freeChannel);
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

void Nrf24DcServer::waitEndSessionTag(uint64_t leftTime)
{
    delayMicroseconds(DC_END_SESSION_TIME);
    return;
    auto startTime = millis();

    while ((millis() - startTime + leftTime) < receivingEndSessionMsgTimeout_)
    {
        if (!driver_.testCarrier())
            break;
        yield();
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
    //driver_.txStandBy();
    return res;
}

void Nrf24DcServer::write3(const void * buf, const uint8_t len)
{
    uint8_t *msgPtr = (uint8_t*)buf;
    uint8_t tmpBuf[32] = { 0 };

    if (isEncrypt_)
    {
        memcpy(tmpBuf, msgPtr, len);
        msgPtr = &tmpBuf[0];
        encryptMsg(msgPtr, len);
    }

    //driver_.setRetries(0, 0);
    driver_.setAutoAck(false);
    driver_.startFastWrite(msgPtr, len, false);
    driver_.startFastWrite(msgPtr, len, false);
    //driver_.startFastWrite(msgPtr, len, false);
    //driver_.write(msgPtr, len, false);
    //driver_.write(msgPtr, len, false);
    driver_.txStandBy();
}

void Nrf24DcServer::setPaLevelForClient(int16_t clientId, uint8_t level)
{
    if ((level > RF24_PA_MAX || level < RF24_PA_MIN) && (level != -1))
        return;

    if (clientId == -1)
    {
        Serial.print("Set ");
        Serial.println(level);
        for (int i = 0; i < DC_MAX_CLIENT_NUMBER; ++i)
        {
            txPaLevel_[i] = level;
        }
    }
    else
    {
        int16_t idx = this->clientIndexById(clientId);

        if (idx != -1)
        {
            txPaLevel_[idx] = level;
        }
    }

}

void Nrf24DcServer::prepareArrays()
{
    for (int i = 0; i < DC_MAX_CLIENT_NUMBER; ++i)
    {
        commsStatus_[i] = 0;
        dataBufferFromClientsSize_[i] = 0;
    }

}

bool Nrf24DcServer::sendBroadcastRequestCommand(String command, int16_t id)
{
    if (!setBroadcastMode(id))
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

    //driver.printRegisters();

    //if (command.substring(0, 9) != "keepAlive") {
    //    Serial.print("Send broadcast ");
    //    Serial.println(command);
    //}


    //for (int i = 0; i < DC_NUMBER_OF_BROADCAST_REQUESTS; ++i) {
        const uint8_t len = command.length() > 32 ? 32 : command.length();
        uint8_t buf[32];
        memcpy(buf, command.c_str(), len);
        write3(buf, len);
    //    yield();
    //}
    //driver_.printDetails();

    return true;
}

