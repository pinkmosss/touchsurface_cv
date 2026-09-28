#include "MultiTouchKit.h"
#include <util/atomic.h>

#define RX_OFFSET                   MTK_RX_START
#define TX_PWM_PIN_ARDUINO          3

#define TX_MUX_SEL_0_PIN_ARDUINO    12
#define TX_MUX_SEL_1_PIN_ARDUINO    11
#define TX_MUX_SEL_2_PIN_ARDUINO    10
#define TX_MUX_SEL_3_PIN_ARDUINO    9
#define TX_MUX_SEL_0_PORT           PORTB
#define TX_MUX_SEL_1_PORT           PORTB
#define TX_MUX_SEL_2_PORT           PORTB
#define TX_MUX_SEL_3_PORT           PORTB
#define TX_MUX_SEL_0_PIN            PORTB4
#define TX_MUX_SEL_1_PIN            PORTB3
#define TX_MUX_SEL_2_PIN            PORTB2
#define TX_MUX_SEL_3_PIN            PORTB1
#define RX_MUX_SEL_0_PIN_ARDUINO    8
#define RX_MUX_SEL_1_PIN_ARDUINO    7
#define RX_MUX_SEL_2_PIN_ARDUINO    6
#define RX_MUX_SEL_3_PIN_ARDUINO    5
#define RX_MUX_SEL_0_PORT           PORTB
#define RX_MUX_SEL_1_PORT           PORTD
#define RX_MUX_SEL_2_PORT           PORTD
#define RX_MUX_SEL_3_PORT           PORTD
#define RX_MUX_SEL_0_PIN            PORTB0
#define RX_MUX_SEL_1_PIN            PORTD7
#define RX_MUX_SEL_2_PIN            PORTD6
#define RX_MUX_SEL_3_PIN            PORTD5
#define RX_ADC                      A3

MultiTouchKit *mtkInstance = nullptr;

MultiTouchKit::MultiTouchKit(void) {
    mtkInstance = this;
}

void MultiTouchKit::setup_sensor(void) {
    scanInProgress = false;
    isDummyRead = false;

    // Set TX select lines of the multiplexer as output
    pinMode(TX_MUX_SEL_0_PIN_ARDUINO, OUTPUT);
    pinMode(TX_MUX_SEL_1_PIN_ARDUINO, OUTPUT);
    pinMode(TX_MUX_SEL_2_PIN_ARDUINO, OUTPUT);
    pinMode(TX_MUX_SEL_3_PIN_ARDUINO, OUTPUT);
    // Set RX select lines of the multiplexer as output
    pinMode(RX_MUX_SEL_0_PIN_ARDUINO, OUTPUT);
    pinMode(RX_MUX_SEL_1_PIN_ARDUINO, OUTPUT);
    pinMode(RX_MUX_SEL_2_PIN_ARDUINO, OUTPUT);
    pinMode(RX_MUX_SEL_3_PIN_ARDUINO, OUTPUT);

    for (uint8_t i = 0; i < MTK_NUM_TX; i++) {
        for (uint8_t j = 0; j < MTK_NUM_RX; j++) {
            touchMatrix[i][j] = 0;
        }
    }
    setupPWM();

    // Enable ADC channel and its interrupt
    ADMUX = (1 << REFS1) | (1 << REFS0) | ((RX_ADC - A0) & 0x0F);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    setupTimer1();
}

void MultiTouchKit::startFrameScan() {
    if (scanInProgress)
        return;
    scanInProgress = true;
    currentTx = 0;
    currentRx = RX_OFFSET;

    // Switch multiplexers
    selectChannelOut(currentTx);
    flushAllChannels();
    selectChannelIn(currentRx);

    // We can mark the upcoming read as a dummy and fire the hardware for some settling time.
    // The ADC takes ~104us to convert, acting as a perfect non-blocking timer.
    isDummyRead = true;
    ADCSRA |= (1 << ADSC);
}

void MultiTouchKit::adcISRHandler() {
    // Reading hardware registers to clear the ADC logic
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    if (isDummyRead) {
        isDummyRead = false;
        ADCSRA |= (1 << ADSC);
        return;
    }

    uint16_t value = (high << 8) | low;
    touchMatrix[currentTx][currentRx] = value;

    // Advance coordinates
    currentRx++;
    if (currentRx >= MTK_NUM_RX) {
        currentRx = RX_OFFSET;
        currentTx++;
    }
    // Check if frame is complete
    if (currentTx >= MTK_NUM_TX) {
        scanInProgress = false;
        frameCounter++;
        return;
    }

    // Setup multiplexers for the next cell
    selectChannelOut(currentTx);
    flushAllChannels();
    selectChannelIn(currentRx);

    // Trigger a dummy read to allow the newly selected cell to settle
    isDummyRead = true;
    ADCSRA |= (1 << ADSC);
}

int MultiTouchKit::read(uint8_t txIndex, uint8_t rxIndex) {
    noInterrupts();
    int val = touchMatrix[txIndex][rxIndex];
    interrupts();
    return val;
}

void MultiTouchKit::setupTimer1() {
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 249; // 16 MHz / (256 * (249 + 1)) = 250 Hz
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void MultiTouchKit::selectChannelOut(int channel) {
    TCCR2A &= ~_BV(COM2B1);
    PORTD &= ~_BV(PORTD3);

    uint8_t muxPattern = ((channel & 0x01) << 4) |
                         ((channel & 0x02) << 2) |
                         ((channel & 0x04))      |
                         ((channel & 0x08) >> 2);
    PORTB = (PORTB & ~0x1E) | muxPattern;

    TCCR2A |= _BV(COM2B1);
}

void MultiTouchKit::selectChannelIn(int channel) {
    // ---------------------------------------------
    if (channel & 1) {
        RX_MUX_SEL_0_PORT |= _BV(RX_MUX_SEL_0_PIN);
    } else {
        RX_MUX_SEL_0_PORT &= ~_BV(RX_MUX_SEL_0_PIN);
    }
    // ---------------------------------------------
    if (channel & 2) {
        RX_MUX_SEL_1_PORT |= _BV(RX_MUX_SEL_1_PIN);
    } else {
        RX_MUX_SEL_1_PORT &= ~_BV(RX_MUX_SEL_1_PIN);
    }
    // ---------------------------------------------
    if (channel & 4) {
        RX_MUX_SEL_2_PORT |= _BV(RX_MUX_SEL_2_PIN);
    } else {
        RX_MUX_SEL_2_PORT &= ~_BV(RX_MUX_SEL_2_PIN);
    }
    // ---------------------------------------------
    if (channel & 8) {
        RX_MUX_SEL_3_PORT |= _BV(RX_MUX_SEL_3_PIN);
    } else {
        RX_MUX_SEL_3_PORT &= ~_BV(RX_MUX_SEL_3_PIN);
    }
}

void MultiTouchKit::flushAllChannels() {
    DDRC |= _BV(PORTC3);        // pinMode(RX_ADC, OUTPUT);
    PORTC &= ~_BV(PORTC3);      // digitalWrite(RX_ADC, LOW);
    for (uint8_t i = 0; i < MTK_NUM_RX; i++) {
        selectChannelIn(i);
        delayMicroseconds(1);
    }
    DDRC &= ~_BV(PORTC3);       // pinMode(RX_ADC, INPUT);
}

void MultiTouchKit::setupPWM() {
    pinMode(TX_PWM_PIN_ARDUINO, OUTPUT);
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);
    OCR2A = 3;  // Frequency = 4 MHz
    OCR2B = 1;  // Duty cycle = 25%
}

bool MultiTouchKit::consumeFrame() {
    auto consumed = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (frameCounter > 0) {
            frameCounter--;
            consumed = true;
        }
    }
    return consumed;
}

void MultiTouchKit::pauseAdc() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TIMSK1 &= ~(1 << OCIE1A); // Stop Timer1 from starting new scans
        ADCSRA &= ~(1 << ADIE);   // Stop ADC_vect from calling MTK ISR
        scanInProgress = false;
        isDummyRead = false;
    }
    while((ADCSRA & (1 << ADSC)));
}

void MultiTouchKit::resumeAdc() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        scanInProgress = false;
        isDummyRead = false;
        currentTx = 0;
        currentRx = RX_OFFSET;
        frameCounter = 0;

        ADMUX = (1 << REFS1) | (1 << REFS0) | ((RX_ADC - A0) & 0x0F);
        // Clear any pending ADC interrupt flag, then re-enable ADC + ADC interrupt.
        ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADIF) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
        // Allow Timer1 to start frames again.
        TIMSK1 |= (1 << OCIE1A);
    }
}

ISR(TIMER1_COMPA_vect) {
    if (mtkInstance) {
        mtkInstance->startFrameScan();
    }
}

ISR(ADC_vect) {
    if (mtkInstance) {
        mtkInstance->adcISRHandler();
    }
}
