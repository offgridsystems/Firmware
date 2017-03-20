#include "Nrf24DcClient.h"
//#include <RH_NRF24.h>

int freeRam();

Nrf24DcClient::Nrf24DcClient(rfDriver& drv)
    : driver(drv)

{
    isBroadcastMode_ = false;
    cmdCount_ = 0;
    //  driver.setRF(rfDriver::DC_DEFAULT_BOUDRATE, rfDriver::DC_DEFAULT_TRANSMITT_POWER);
    //  broadcastChannel_ = DC_DEFAULT_BRADCAST_CNANNEL;
    //  workChannel_ = DC_DEFAULT_WORK_CNANNEL;
    //  setNetworkAddr(DC_DEFAULT_NETWORK_ADDR);
    //  setDeviceId(DC_DEFAULT_DEVICE_ADDR);
    //  bufferLen_ = -1;
}

bool Nrf24DcClient::init()
{
    return driver.begin();
}

void Nrf24DcClient::setWorkChannel(uint8_t ch)
{
    workChannel_ = ch;
}

void Nrf24DcClient::setBroadcastChannel(uint8_t ch)
{
    broadcastChannel_ = ch;
}

uint8_t Nrf24DcClient::workChannel()
{
    return workChannel_;
}

uint8_t Nrf24DcClient::broadcastChannel()
{
    return broadcastChannel_;
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

void Nrf24DcClient::setDeviceId(int16_t id)
{
    clientAddress_ = networkAddress_ + id;
}

int16_t Nrf24DcClient::deviceId()
{
    return clientId;
}

uint64_t Nrf24DcClient::clientAddress()
{
    return clientAddress_;
}

void Nrf24DcClient::setSessionTimeout(uint16_t timeout)
{
    sessionTimeout_ = timeout;
}

uint16_t Nrf24DcClient::sessionTimeout()
{
    return sessionTimeout_;
}

bool Nrf24DcClient::addCommand(AbstractClientCommand * cmd)
{
    if (cmdCount_ >= DC_NUMBER_OF_COMMANDS)
        return false;

    cmdArray_[cmdCount_] = cmd;
    ++cmdCount_;

    return true;
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

int Nrf24DcClient::listenBroadcast()
{
    if (!isBroadcastMode_)
    {
        Serial.println(F("Set broadcast mode"));
        Serial.println(freeRam());

        digitalWrite(5, LOW);
        delay(10);

        driver.setChannel(broadcastChannel_);
        driver.setAutoAck(false);
        driver.openReadingPipe(1, serverAddress_);
        driver.startListening();
        isBroadcastMode_ = true;
        //driver.printDetails();
    }

    //driver.printRegisters();

    int8_t receivedPacketSize = 0;

    if (driver.available())
    {
        receivedPacketSize = driver.getDynamicPayloadSize();
        driver.read(buffer_, 32);
        
        Serial.print(F(" received packet "));
        Serial.println(receivedPacketSize);
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
            //Serial.println(spliterPos);
            //Serial.println(command);
            //Serial.println(parametr);

            for (int i = 0; i < cmdCount_; ++i)
                if (cmdArray_[i]->isCommand(String(command)))
                {
                    //Serial.println(F("Satrt command ssh"));
                    driver.stopListening();
                    isBroadcastMode_ = false;

                    for (int j = 1; j < 301; ++j)
                    {
                        setDeviceId(j);
                        cmdArray_[i]->run(String(parametr));
                    }

                    break;
                }
        }
    }

    return 0x00;
}

bool Nrf24DcClient::startSession()
{
    isBroadcastMode_ = false;
    driver.setAutoAck(true);

    driver.setChannel(workChannel_);

    return true;
}






