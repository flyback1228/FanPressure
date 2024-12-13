#include "stm32serial.h"
#include "QtSerialPort/QSerialPortInfo"
#include "QThread"
Stm32Serial::Stm32Serial() : stop_flag_(false) {

}

Stm32Serial::~Stm32Serial()
{
    stop_flag_ = true;
    if(port_.isOpen()){
        port_.close();
    }
    QThread::sleep(150);
}

void Stm32Serial::setPortName(QString port)
{
    port_name_ = port;
}

void Stm32Serial::openPort()
{
    //QSerialPortInfo info;
    port_.setBaudRate(QSerialPort::Baud115200);
    port_.setDataBits(QSerialPort::Data8);
    port_.setParity(QSerialPort::NoParity);
    port_.setStopBits(QSerialPort::OneStop);
    port_.setFlowControl(QSerialPort::NoFlowControl);

    port_.setPortName(port_name_);

    if(port_.open(QIODevice::ReadWrite)){
        message(QString("Open %1 successfully").arg(port_name_));
    }else{
        message(QString("Open %1 failed").arg(port_name_));
    }

}

void Stm32Serial::closePort()
{
    if(port_.isOpen())
        port_.close();
}

void Stm32Serial::writePort(QByteArray data)
{
    if(!port_.isOpen()){
        message(QString("Port is not open!").arg(port_name_));
    }
    port_.write(data);
}

void Stm32Serial::run()
{
    while(!stop_flag_){
        if(port_.isOpen()){
            port_.readAll()
        }
        QThread::sleep(100);
    }
}
