#ifndef PARAMETER_H
#define PARAMETER_H


#include <cstdint>
typedef struct Q_PACKED
{
    char header[3] = {'S','Y','R'};
    uint8_t bk1697_panel_locked;
    float bk1697_voltage;
    float bk1697_current;
    char tailer[3] = {'C','O','E'};


} parameter_t;

#endif // PARAMETER_H
