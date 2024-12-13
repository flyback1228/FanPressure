#ifndef STM32SERIAL_H
#define STM32SERIAL_H

#include <QObject>
#include <QRunnable>
#include <QtSerialPort/QSerialPort>

class Stm32Serial : public QRunnable
{
public:
    Stm32Serial();
    ~Stm32Serial();


    void setPortName(QString port);
    void openPort();
    void closePort();

    void writePort(QByteArray);

signals:
    void message(QString);
    void data(QByteArray);


private:
    void run() override;
    QSerialPort port_;
    QString port_name_;

    bool stop_flag_;
};

#endif // STM32SERIAL_H
