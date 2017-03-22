#include "Nrf24DcServer.h"


Nrf24DcServer::Nrf24DcServer(rfDriver& drv)
  : driver_(drv)
{
  currnetDeviceCount_ = 0;
  setSessionTimeout(DC_DEFAULT_SESSION_TIMEOUT);
  setReadingTimeout(DC_DEFAULT_READING_TIMEOUT);
}

bool Nrf24DcServer::init()
{
  return driver_.begin();
}

void Nrf24DcServer::setWorkChannel(uint8_t ch)
{
  workChannel_ = ch;
}

void Nrf24DcServer::setBroadcastChannel(uint8_t ch)
{
  broadcastChannel_ = ch;
}

uint8_t Nrf24DcServer::workChannel()
{
  return workChannel_;
}

uint8_t Nrf24DcServer::broadcastChannel()
{
  return broadcastChannel_;
}

void Nrf24DcServer::setNetworkAddr(uint64_t nAddr)
{
  networkAddress_ = nAddr;
  
  while ( networkAddress_ > 0xFFFFFFLL )
    networkAddress_ /= 0x100;

  while ( networkAddress_*0x100LL < 0xFFFFFFLL)
    networkAddress_ *= 0x100LL;

  networkAddress_ *= 0x10000LL;
  serverAddress_ = networkAddress_ + 0xFFFFLL;
}

uint64_t  Nrf24DcServer::networkAddr()
{
  return networkAddress_;
}

int Nrf24DcServer::handledClientsCount()
{
  return currnetDeviceCount_;
}

bool Nrf24DcServer::isDeviceHandled(int16_t id)
{
  for(int i = 0; i < currnetDeviceCount_; ++i)
  {
    if ( id == handledDevicesId_[i] )
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

bool Nrf24DcServer::addClient(int16_t deviceId)
{
	if (currnetDeviceCount_ < DC_MAX_CLIENT_NUMBER && !isDeviceHandled(deviceId) )
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
	if ( (currnetDeviceCount_ + count) <= DC_MAX_CLIENT_NUMBER)
    {
		for (int i = 0; i < count; ++i)
		{
			//Serial.println(from+i);
			addClient(from+i);
		}
    return true;
	}
    else
		return false;
  
}

bool Nrf24DcServer::removeClient(int16_t clientId)
{
  int16_t idPos = -1;
  
  for(int i = 0; i < currnetDeviceCount_; ++i)
  {
    if ( handledDevicesId_[i] == clientId)
    {
      idPos = i;
      break;
    }
  }

  if ( idPos == -1)
  {
    return false;
  }

  for (int i = idPos+1; i < currnetDeviceCount_; ++i)
  {
    handledDevicesId_[i-1] = handledDevicesId_[i];
  }

  --currnetDeviceCount_;
  return true;
}

bool Nrf24DcServer::removeClientsByRange(int16_t from, int16_t count)
{
  int16_t last = from+count;
  for (int i = from; i < last; ++i)
  {
    removeClient(i);
  }

  return true;
}

bool Nrf24DcServer::setBroadcastMode()
{
  driver_.flush_tx();
  driver_.setChannel(broadcastChannel_);
  driver_.setAutoAck(false);

    
  return true;
}

bool Nrf24DcServer::setSingleClientMode(int16_t device_id)
{
  
  long long clientAddress = networkAddress_ + device_id;
  driver_.setAutoAck(true);
  driver_.setChannel(workChannel_);
    
  return true;  
}

int16_t Nrf24DcServer::clientIdAt(int16_t index)
{
  if (index >=0 && index < currnetDeviceCount_)
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
	//driver_.setAutoAck(false);
	//Serial.print("1 ");
	sprintf((char*)sendBuffer_, "ssch:%d", workChannel_);
	//yield();
	//Serial.print("2 ");
	//Serial.println((char*)sendBuffer_);
	uint8_t len = strlen((char*)sendBuffer_)+1;
	//Serial.print("3 ");
	//long int s = micros();
	//yield();
	//Serial.print(len);
	//driver_.printDetails();
	driver_.write(sendBuffer_, len);
	//Serial.print("5 ");
	//yield();
	//driver_.waitPacketSentAck();
	//long int s2 = micros();
	//Serial.print("One packet no ack sent time = ");
	//Serial.println(s2-s);

	return true;
}

int16_t Nrf24DcServer::startSession()
{
    prepareArrays();
    sendBroadcastRequestCommand();
    driver_.txStandBy();

    uint64_t startSessionTime = millis();
    int16_t deviceCount = handledClientsCount();
    uint8_t rLen;
    Serial.println(deviceCount);

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

        driver_.stopListening();

        driver_.flush_tx();
        uint64_t clientAddr = clientIdAt(i) + networkAddr();

        setSingleClientMode(clientAddr);
        driver_.openWritingPipe(clientAddr);

        for (int k = 0; k < 1; ++k)
        {
            yield();
            if (driver_.write("S", 2))
            {
                //Serial.println("Start not sended");
                continue;
            }
            else
            {
                //driver.printDetails();
                break;
            }

        }
        yield();
        driver_.txStandBy();
        driver_.flush_tx();
        uint64_t  recvStartTime = millis();
        bool isRecvData = false;
        rLen = 32;
        driver_.openReadingPipe(1, clientAddr);
        driver_.startListening();
        yield();

        while ((millis() - recvStartTime) <= readingTimeout_)
        {
            yield();
            if (driver_.available())
            {
                rLen = driver_.getDynamicPayloadSize();
                driver_.read(dataBufferFromClients_[i], rLen);
                dataBufferFromClientsSize_[i] = rLen;
                isRecvData = true;
                break;
            }
        }

        if (!isRecvData)
        {
            //Serial.println("Data from client not received");
            continue;
        }
        yield();

        driver_.flush_tx();
        driver_.stopListening();
        driver_.openWritingPipe(clientAddr);
        yield();


        if (driver_.write(dataBufferToClients_[i], DC_MAX_SIZE_OF_DATA_FOR_SENDING))
        {
            commsStatus_[i] = 1;
            ++numberOfHandledDevices;
        }
        driver_.txStandBy();

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
    if (idx >= currnetDeviceCount_ )
        return;
    else
    {
        if (len > DC_MAX_SIZE_OF_DATA_FOR_SENDING)
            len = DC_MAX_SIZE_OF_DATA_FOR_SENDING;

        if (idx == -1)
        {
            for (int i = 0; i < handledDevicesCount_; ++i)
            {
                memset(dataBufferToClients_[i], 0, DC_MAX_SIZE_OF_DATA_FOR_SENDING);
                memcpy(dataBufferToClients_[i], data, len);
            }
        }
        else
        {
            memset(dataBufferToClients_[idx], 0, DC_MAX_SIZE_OF_DATA_FOR_SENDING);
            memcpy(dataBufferToClients_[idx], data, len);
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

void Nrf24DcServer::sendBroadcastRequestCommand()
{
    driver_.stopListening();
    setBroadcastMode();

    yield();
    driver_.openWritingPipe(serverAddress_);

    yield();
    //driver.printRegisters();

    //Serial.println("loop to broadcast request for data");
    for (int i = 0; i < DC_NUMBER_OF_BROADCAST_REQUESTS; ++i) {
        sendRequestForSession();
        yield();
    }
}

