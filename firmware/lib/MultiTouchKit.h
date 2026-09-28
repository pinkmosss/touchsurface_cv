#ifndef MULTITOUCHKIT_H
#define MULTITOUCHKIT_H

#include "Arduino.h"


#define MTK_NUM_TX      8
#define MTK_NUM_RX      9
#define MTK_RX_START    1
#define MTK_RX_COUNT    (MTK_NUM_RX - MTK_RX_START)
#define MTK_NUM_CELLS   (MTK_NUM_TX * MTK_RX_COUNT)


class MultiTouchKit {
public:
    MultiTouchKit(void);
    void setup_sensor(void);
    int read(uint8_t txIndex, uint8_t rxIndex);
    // Internal ISR methods
    void startFrameScan();
    void adcISRHandler();
    bool consumeFrame();
    void pauseAdc();
    void resumeAdc();

private:
    void setupPWM();
    void setupTimer1();
    void selectChannelOut(int channel);
    void selectChannelIn(int channel);
    void flushAllChannels();

    // ISR State Variables
    volatile uint16_t touchMatrix[MTK_NUM_TX][MTK_NUM_RX];
    volatile uint8_t currentTx;
    volatile uint8_t currentRx;
    volatile bool scanInProgress;
    volatile bool isDummyRead;
    volatile uint32_t frameCounter;
};

#endif
