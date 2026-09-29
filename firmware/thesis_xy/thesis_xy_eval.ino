#include "MultiTouchKit.h"
#include <Adafruit_MCP4728.h>
#include <util/atomic.h>
#define ENABLE_CONTROL_PADS 1
#if ENABLE_CONTROL_PADS
#include <ADCTouch.h>
#endif
#include <Wire.h>


// Sensor configuration
#define ACTIVE_TOUCH_BYTES        ((MTK_NUM_CELLS + 7) / 8)

// Touch detection
#define TOUCH_THRESHOLD           30
#define TOUCH_RELEASE_PERCENT     20
#define TOUCH_RELEASE_THRESHOLD   ((TOUCH_THRESHOLD * TOUCH_RELEASE_PERCENT) / 100)
#define TOUCH_POLARITY            1
#define BASELINE_SAMPLES          16
#define BASELINE_SAMPLE_DELAY_MS  1
#define BASELINE_TRACK_SHIFT      3
#define ENABLE_TOUCH_STATE_DEBUG  0

// Recording configuration (RAM reservation)
#define RECORD_BUFFER_SAMPLES     512
#define RECORD_NO_TOUCH_CODE      0xFF

// Shared recording/playback clock
#define FALLBACK_SAMPLE_INTERVAL_MS 500

// External trigger input
#define TRIG_PIN                  2
#define TRIG_MAX_PENDING_PULSES   255

// Control pads (ADCTouch ADC channels)
#if ENABLE_CONTROL_PADS
#define RECORD_PAD_ADC_CHANNEL    A2
#define PLAYBACK_PAD_ADC_CHANNEL  A1
#define CONTROL_PAD_CAL_SAMPLES   1
#define CONTROL_PAD_READ_SAMPLES  1
#define CONTROL_PAD_THRESHOLD     50
#define CONTROL_PAD_SAMPLE_INTERVAL_MS 200
#endif

// Debug LED (PC0)
#define RECORD_LED_TOGGLE_MS      50

// --- Evaluation instrumentation (see evaluation-protocol.md) ---
// 0 = normal operation (no instrumentation). 1-6 select one test at a time;
// build/flash separately per test so instrumentation never mixes with
// normal operation or with another test.
//   1 = frame rate            4 = hysteresis effectiveness
//   2 = internal latency      5 = size-estimate correlation
//   3 = positional accuracy   6 = recording/playback fidelity
#define EVAL_MODE 0
// Test 4 only: set to 1 to run the "hysteresis disabled" condition
// (release threshold == touch threshold) for the flicker-count comparison.
#define EVAL_TEST4_DISABLE_HYSTERESIS 0

#if EVAL_MODE == 4 && EVAL_TEST4_DISABLE_HYSTERESIS
#define EVAL_RELEASE_THRESHOLD TOUCH_THRESHOLD
#else
#define EVAL_RELEASE_THRESHOLD TOUCH_RELEASE_THRESHOLD
#endif

// --- MIDI demo mode (touch-to-MIDI bridge for the Logic Pro demo) ---
// Off by default so it never interferes with EVAL_MODE or normal operation.
// When on, sendMidiFromTouch() writes raw MIDI bytes to the same Serial
// link Hairless reads as a continuous byte stream, so it can't share that
// link with the human-readable debug prints (printActiveTouches,
// printControlModes, the setup banner) -- those are suppressed below
// whenever this is set.
#define MIDI_DEMO_MODE 0

#if MIDI_DEMO_MODE && EVAL_MODE != 0
#error "MIDI_DEMO_MODE requires EVAL_MODE 0 -- MIDI bytes and eval text can't share the serial link"
#endif

// DAC routing
#define DAC_MAX_VALUE             4095
#define DAC_CHANNEL_X             MCP4728_CHANNEL_A
#define DAC_CHANNEL_Y             MCP4728_CHANNEL_B
#define DAC_CHANNEL_SIZE          MCP4728_CHANNEL_C
#define DAC_CHANNEL_TOUCH_FLAG    MCP4728_CHANNEL_D
#define DAC_TOUCH_LOW             0
#define DAC_TOUCH_HIGH            DAC_MAX_VALUE
#define DAC_I2C_CLOCK_HZ          400000UL
#define MAX_DOMINANT_CLUSTER_SIZE 9

struct DominantTouch {
  bool valid;
  uint8_t tx;
  uint8_t rxOffset;
  uint8_t size;
};

// Static variables
static MultiTouchKit mtk = {};
static Adafruit_MCP4728 mcp;
static uint16_t baseline[MTK_NUM_CELLS];
static uint16_t baselineAccum[MTK_NUM_CELLS];
static uint16_t touchDelta[MTK_NUM_CELLS];
static uint8_t activeTouchBits[ACTIVE_TOUCH_BYTES];
static uint16_t lastDacX = 0xFFFF;
static uint16_t lastDacY = 0xFFFF;
static uint16_t lastDacSize = 0xFFFF;
static uint16_t lastDacTouchFlag = 0xFFFF;
static uint8_t recordedTouchPoints[RECORD_BUFFER_SAMPLES];
static uint16_t recordWriteIndex = 0;
static uint16_t recordSampleCount = 0;
static uint16_t replayReadIndex = 0;
static uint16_t replaySamplesRemaining = 0;
static bool replayActive = false;
static volatile uint8_t trigPendingPulses = 0;
static uint32_t lastSequenceStepMs = 0;
static bool recordingInProgress = false;
#if ENABLE_CONTROL_PADS
static bool lastRecordPadTouched = false;
static bool lastPlaybackPadTouched = false;
static bool cachedRecordPadTouched = false;
static bool cachedPlaybackPadTouched = false;
static bool recordModeEnabled = false;
static bool playbackModeEnabled = false;
static int recordPadReference = 0;
static int playbackPadReference = 0;
static uint32_t lastControlPadSampleMs = 0;
#endif
static bool recordLedState = false;
static uint32_t lastRecordLedToggleMs = 0;

void stopReplayRecordedTouches();
uint16_t mapTxToDac(uint8_t tx);
uint16_t mapRxToDac(uint8_t rxOffset);
void writeDacOutputsIfChanged(uint16_t xValue,
                              uint16_t yValue,
                              uint16_t sizeValue,
                              uint16_t touchFlagValue,
                              bool forceWrite = false);

void onTrigRising() {
  if (trigPendingPulses < TRIG_MAX_PENDING_PULSES) {
    trigPendingPulses++;
  }
}

bool consumeTrigPulse() {
  bool consumed = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (trigPendingPulses > 0) {
      trigPendingPulses--;
      consumed = true;
    }
  }
  return consumed;
}

void clearTrigPulses() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    trigPendingPulses = 0;
  }
}

void resetSequenceStepClock() {
  clearTrigPulses();
  lastSequenceStepMs = millis();
}

bool isSequenceStepDue() {
  if (consumeTrigPulse()) {
    lastSequenceStepMs = millis();
    return true;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastSequenceStepMs) < FALLBACK_SAMPLE_INTERVAL_MS) {
    return false;
  }

  lastSequenceStepMs = now;
  return true;
}

void initTrigInput() {
  DDRD &= ~_BV(DDD2);       // PD2 / Arduino D2 as input.
  PORTD &= ~_BV(PORTD2);    // No internal pullup; Eurorack input should be externally conditioned.
  clearTrigPulses();
  attachInterrupt(digitalPinToInterrupt(TRIG_PIN), onTrigRising, RISING);
}

uint8_t toPhysicalRx(uint8_t rxOffset) {
  return rxOffset + MTK_RX_START;
}

uint8_t toCellIndex(uint8_t tx, uint8_t rxOffset) {
  return tx * MTK_RX_COUNT + rxOffset;
}

bool isTouchActive(uint8_t cellIndex) {
  uint8_t byteIndex = cellIndex >> 3;
  uint8_t bitMask = 1 << (cellIndex & 0x7);
  return (activeTouchBits[byteIndex] & bitMask) != 0;
}

bool isTouchBitSet(const uint8_t *touchBits, uint8_t cellIndex) {
  uint8_t byteIndex = cellIndex >> 3;
  uint8_t bitMask = 1 << (cellIndex & 0x7);
  return (touchBits[byteIndex] & bitMask) != 0;
}

void setTouchActive(uint8_t cellIndex, bool isActive) {
  uint8_t byteIndex = cellIndex >> 3;
  uint8_t bitMask = 1 << (cellIndex & 0x7);
  if (isActive) {
    activeTouchBits[byteIndex] |= bitMask;
  } else {
    activeTouchBits[byteIndex] &= ~bitMask;
  }

#if ENABLE_TOUCH_STATE_DEBUG
  char dbgMsg[24];
  snprintf(dbgMsg, 24, "Cell index %u --> %u\n\r", cellIndex, isActive);
  Serial.print(dbgMsg);
#endif
}

void clearActiveTouches() {
  for (uint8_t i = 0; i < ACTIVE_TOUCH_BYTES; i++) {
    activeTouchBits[i] = 0;
  }
}

void trackBaseline(uint8_t cellIndex, uint16_t reading, uint8_t shift) {
  uint16_t oldBaseline = baseline[cellIndex];
  baseline[cellIndex] = (uint16_t)((((uint32_t)oldBaseline << shift) - oldBaseline + reading) >> shift);
}

uint16_t touchStrength(uint16_t reading, uint16_t base) {
  int16_t signedDelta = (int16_t)reading - (int16_t)base;
#if TOUCH_POLARITY < 0
  signedDelta = -signedDelta;
#endif
  return signedDelta > 0 ? (uint16_t)signedDelta : 0;
}

uint8_t packTouchPoint(uint8_t tx, uint8_t rx) {
  return (uint8_t)(((tx & 0x0F) << 4) | (rx & 0x0F));
}

uint8_t encodeDominantTouch(const DominantTouch &dominant) {
  if (!dominant.valid) {
    return RECORD_NO_TOUCH_CODE;
  }
  return packTouchPoint(dominant.tx, toPhysicalRx(dominant.rxOffset));
}

void appendRecordedSample(uint8_t encodedSample) {
  recordedTouchPoints[recordWriteIndex] = encodedSample;
  recordWriteIndex++;
  if (recordWriteIndex >= RECORD_BUFFER_SAMPLES) {
    recordWriteIndex = 0;
  }

  if (recordSampleCount < RECORD_BUFFER_SAMPLES) {
    recordSampleCount++;
  }
}

void clearRecordedTouches() {
  recordWriteIndex = 0;
  recordSampleCount = 0;
  stopReplayRecordedTouches();
}

void recordDominantTouchIfDue(const DominantTouch &dominant) {
  if (!isSequenceStepDue()) {
    return;
  }

#if EVAL_MODE == 6
  Serial.print(F("RECORD_SAMPLE,"));
  if (dominant.valid) {
    Serial.print(dominant.tx);
    Serial.print(',');
    Serial.println(toPhysicalRx(dominant.rxOffset));
  } else {
    Serial.println(F("NONE"));
  }
#endif

  appendRecordedSample(encodeDominantTouch(dominant));
}

bool decodeRecordedTouchPoint(uint8_t encodedSample, uint8_t *tx, uint8_t *rxOffset) {
  if (encodedSample == RECORD_NO_TOUCH_CODE) {
    return false;
  }

  uint8_t decodedTx = (encodedSample >> 4) & 0x0F;
  uint8_t decodedRx = encodedSample & 0x0F;
  if (decodedTx >= MTK_NUM_TX || decodedRx < MTK_RX_START || decodedRx >= MTK_NUM_RX) {
    return false;
  }

  *tx = decodedTx;
  *rxOffset = decodedRx - MTK_RX_START;
  return true;
}

void outputReplaySampleToDac(uint8_t encodedSample) {
  uint16_t xValue = 0;
  uint16_t yValue = 0;
  uint16_t sizeValue = 0;
  uint16_t touchFlagValue = DAC_TOUCH_LOW;
  uint8_t tx = 0;
  uint8_t rxOffset = 0;

  bool hasTouch = decodeRecordedTouchPoint(encodedSample, &tx, &rxOffset);
  if (hasTouch) {
    xValue = mapTxToDac(tx);
    yValue = mapRxToDac(rxOffset);
    touchFlagValue = DAC_TOUCH_HIGH;
  }

#if EVAL_MODE == 6
  Serial.print(F("REPLAY_SAMPLE,"));
  Serial.print(millis());
  Serial.print(',');
  if (hasTouch) {
    Serial.print(tx);
    Serial.print(',');
    Serial.println(toPhysicalRx(rxOffset));
  } else {
    Serial.println(F("NONE"));
  }
#endif

  writeDacOutputsIfChanged(xValue, yValue, sizeValue, touchFlagValue);
}

void startReplayRecordedTouches() {
  if (recordSampleCount == 0) {
    replayActive = false;
    return;
  }

  replayReadIndex = (recordWriteIndex + RECORD_BUFFER_SAMPLES - recordSampleCount) % RECORD_BUFFER_SAMPLES;
  replaySamplesRemaining = recordSampleCount;
  resetSequenceStepClock();
  outputReplaySampleToDac(RECORD_NO_TOUCH_CODE);
  replayActive = true;
}

void stopReplayRecordedTouches() {
  replayActive = false;
  replaySamplesRemaining = 0;
}

void replayRecordedTouches() {
  if (!replayActive || replaySamplesRemaining == 0) {
    return;
  }

  if (!isSequenceStepDue()) {
    return;
  }

  outputReplaySampleToDac(recordedTouchPoints[replayReadIndex]);

  replayReadIndex++;
  if (replayReadIndex >= RECORD_BUFFER_SAMPLES) {
    replayReadIndex = 0;
  }

  replaySamplesRemaining--;
  if (replaySamplesRemaining == 0) {
    replayActive = false;
  }
}

#if ENABLE_CONTROL_PADS

static void pauseAdc() {
  mtk.pauseAdc();
}

static void resumeAdc() {
  DDRC &= ~(_BV(DDC1) | _BV(DDC2));      // input mode
  PORTC &= ~(_BV(PORTC1) | _BV(PORTC2)); // pullups off
  mtk.resumeAdc();
}

void calibrateControlPads() {
  pauseAdc();
  recordPadReference = ADCTouch.read(RECORD_PAD_ADC_CHANNEL, CONTROL_PAD_CAL_SAMPLES);
  playbackPadReference = ADCTouch.read(PLAYBACK_PAD_ADC_CHANNEL, CONTROL_PAD_CAL_SAMPLES);
  resumeAdc();
}

bool isControlPadTouched(uint8_t adcChannel, int reference) {
  int value = ADCTouch.read(adcChannel, CONTROL_PAD_READ_SAMPLES);
  int delta = abs(value - reference);
  return delta > CONTROL_PAD_THRESHOLD;
}

void sampleControlPadsIfDue() {
  uint32_t now = millis();
  if (lastControlPadSampleMs != 0 &&
      (uint32_t)(now - lastControlPadSampleMs) < CONTROL_PAD_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastControlPadSampleMs = now;
  pauseAdc();
  cachedRecordPadTouched = isControlPadTouched(RECORD_PAD_ADC_CHANNEL, recordPadReference);
  cachedPlaybackPadTouched = isControlPadTouched(PLAYBACK_PAD_ADC_CHANNEL, playbackPadReference);
  resumeAdc();
}

void setRecordLed(bool isOn) {
  recordLedState = isOn;
  if (isOn) {
    PORTC |= _BV(PC0);
  } else {
    PORTC &= ~_BV(PC0);
  }
}

void initRecordLed() {
  DDRC |= _BV(PC0);
  setRecordLed(false);
  lastRecordLedToggleMs = 0;
}

void updateRecordLed() {
  if (!recordingInProgress) {
    setRecordLed(false);
    lastRecordLedToggleMs = 0;
    return;
  }

  uint32_t now = millis();
  if (lastRecordLedToggleMs != 0 &&
      (uint32_t)(now - lastRecordLedToggleMs) < RECORD_LED_TOGGLE_MS) {
    return;
  }

  lastRecordLedToggleMs = now;
  setRecordLed(!recordLedState);
}

void printControlModes() {
#if MIDI_DEMO_MODE
  return;
#endif
  Serial.print(F("Modes: Record="));
  Serial.print(recordModeEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" Playback="));
  Serial.println(playbackModeEnabled ? F("ON") : F("OFF"));
}

bool handleControlPads(const DominantTouch &dominant) {
  sampleControlPadsIfDue();
  bool recordTouched = cachedRecordPadTouched;
  bool playbackTouched = cachedPlaybackPadTouched;
  bool recordReleased = lastRecordPadTouched && !recordTouched;
  bool playbackReleased = lastPlaybackPadTouched && !playbackTouched;

  if (playbackReleased) {
    playbackModeEnabled = !playbackModeEnabled;
    if (playbackModeEnabled) {
      recordModeEnabled = false;
      startReplayRecordedTouches();
      if (!replayActive) {
        playbackModeEnabled = false;
        Serial.println(F("Playback: buffer empty"));
      }
    } else {
      stopReplayRecordedTouches();
    }
    printControlModes();
  }

  if (recordReleased && !playbackModeEnabled) {
    recordModeEnabled = !recordModeEnabled;
    if (recordModeEnabled) {
      clearRecordedTouches();
      resetSequenceStepClock();
      Serial.println(F("Recording: buffer cleared"));
    }
    printControlModes();
  }

  recordingInProgress = recordModeEnabled;

  if (playbackModeEnabled) {
    replayRecordedTouches();
    if (!replayActive) {
      playbackModeEnabled = false;
      printControlModes();
      lastRecordPadTouched = recordTouched;
      lastPlaybackPadTouched = playbackTouched;
      return false;
    }
    lastRecordPadTouched = recordTouched;
    lastPlaybackPadTouched = playbackTouched;
    return true;
  }

  if (recordModeEnabled) {
    recordDominantTouchIfDue(dominant);
  }

  lastRecordPadTouched = recordTouched;
  lastPlaybackPadTouched = playbackTouched;
  return false;
}
#else
void calibrateControlPads() {
}

bool handleControlPads(const DominantTouch &dominant) {
  (void)dominant;
  recordingInProgress = false;
  return false;
}
#endif

bool initDac() {
  if (!mcp.begin()) {
    return false;
  }

  Wire.setClock(DAC_I2C_CLOCK_HZ);
  writeDacOutputsIfChanged(0, 0, 0, DAC_TOUCH_LOW, true);
  return true;
}

void clearTouchState() {
  for (uint8_t i = 0; i < MTK_NUM_CELLS; i++) {
    baseline[i] = 0;
    touchDelta[i] = 0;
  }

  clearActiveTouches();
}

void calibrateBaseline() {
  for (uint8_t i = 0; i < MTK_NUM_CELLS; i++) {
    baselineAccum[i] = 0;
  }

  for (uint8_t sample = 0; sample < BASELINE_SAMPLES; sample++) {
    delay(BASELINE_SAMPLE_DELAY_MS);
    for (uint8_t tx = 0; tx < MTK_NUM_TX; tx++) {
      uint8_t cellIndex = tx * MTK_RX_COUNT;
      for (uint8_t rxOffset = 0; rxOffset < MTK_RX_COUNT; rxOffset++, cellIndex++) {
        baselineAccum[cellIndex] += (uint16_t)mtk.read(tx, toPhysicalRx(rxOffset));
      }
    }
  }

  for (uint8_t i = 0; i < MTK_NUM_CELLS; i++) {
    baseline[i] = baselineAccum[i] / BASELINE_SAMPLES;
  }
}

DominantTouch scanTouchMatrix(bool *stateChanged) {
  uint8_t previousTouchBits[ACTIVE_TOUCH_BYTES];
  DominantTouch dominant = {false, 0, 0, 0};
  uint16_t maxDelta = 0;

  for (uint8_t i = 0; i < ACTIVE_TOUCH_BYTES; i++) {
    previousTouchBits[i] = activeTouchBits[i];
  }

  clearActiveTouches();

  for (uint8_t tx = 0; tx < MTK_NUM_TX; tx++) {
    uint8_t cellIndex = tx * MTK_RX_COUNT;
    for (uint8_t rxOffset = 0; rxOffset < MTK_RX_COUNT; rxOffset++, cellIndex++) {
      uint16_t reading = (uint16_t)mtk.read(tx, toPhysicalRx(rxOffset));
      uint16_t base = baseline[cellIndex];
      uint16_t delta = touchStrength(reading, base);
      bool wasActive = isTouchBitSet(previousTouchBits, cellIndex);
      bool active = delta >= (wasActive ? EVAL_RELEASE_THRESHOLD : TOUCH_THRESHOLD);

#if EVAL_MODE == 4
      if (active != wasActive) {
        Serial.print(F("TRANSITION,"));
        Serial.print(millis());
        Serial.print(',');
        Serial.println(active ? F("ACTIVE") : F("INACTIVE"));
      }
#endif

      setTouchActive(cellIndex, active);
      touchDelta[cellIndex] = active ? delta : 0;

      if (active && delta > maxDelta) {
        maxDelta = delta;
        dominant.valid = true;
        dominant.tx = tx;
        dominant.rxOffset = rxOffset;
      }

      if (!active) {
        trackBaseline(cellIndex, reading, BASELINE_TRACK_SHIFT);
      }
    }
  }

  if (dominant.valid) {
    for (int8_t txOffset = -1; txOffset <= 1; txOffset++) {
      for (int8_t rxOffset = -1; rxOffset <= 1; rxOffset++) {
        int8_t tx = (int8_t)dominant.tx + txOffset;
        int8_t rx = (int8_t)dominant.rxOffset + rxOffset;
        if (tx >= 0 && tx < MTK_NUM_TX && rx >= 0 && rx < MTK_RX_COUNT &&
            isTouchActive(toCellIndex((uint8_t)tx, (uint8_t)rx))) {
          dominant.size++;
        }
      }
    }
  }

  *stateChanged = false;
  for (uint8_t i = 0; i < ACTIVE_TOUCH_BYTES; i++) {
    if (activeTouchBits[i] != previousTouchBits[i]) {
      *stateChanged = true;
      break;
    }
  }

  return dominant;
}

uint16_t mapRangeToDac(int value, int inputMin, int inputMax) {
  if (inputMax <= inputMin) {
    return 0;
  }

  int clamped = constrain(value, inputMin, inputMax);
  long scaled = (long)(clamped - inputMin) * DAC_MAX_VALUE;
  return (uint16_t)(scaled / (inputMax - inputMin));
}

uint16_t mapTxToDac(uint8_t tx) {
  return mapRangeToDac((int)tx, 0, MTK_NUM_TX - 1);
}

uint16_t mapRxToDac(uint8_t rxOffset) {
  return mapRangeToDac((int)rxOffset, 0, MTK_RX_COUNT - 1);
}

uint16_t mapSizeToDac(uint8_t size) {
  return mapRangeToDac((int)size, 0, MAX_DOMINANT_CLUSTER_SIZE);
}

uint16_t mapTouchPresenceToDac(bool isTouchPresent) {
  return isTouchPresent ? DAC_TOUCH_HIGH : DAC_TOUCH_LOW;
}

void writeDacOutputsIfChanged(uint16_t xValue,
                              uint16_t yValue,
                              uint16_t sizeValue,
                              uint16_t touchFlagValue,
                              bool forceWrite) {
  if (!forceWrite &&
      lastDacX == xValue &&
      lastDacY == yValue &&
      lastDacSize == sizeValue &&
      lastDacTouchFlag == touchFlagValue) {
    return;
  }

  if (mcp.fastWrite(xValue, yValue, sizeValue, touchFlagValue)) {
    lastDacX = xValue;
    lastDacY = yValue;
    lastDacSize = sizeValue;
    lastDacTouchFlag = touchFlagValue;
  }
}

void updateDacFromDominantTouch(const DominantTouch &dominant) {
  uint16_t xValue = 0;
  uint16_t yValue = 0;
  uint16_t sizeValue = 0;
  uint16_t touchFlagValue = mapTouchPresenceToDac(dominant.valid);

  if (dominant.valid) {
    xValue = mapTxToDac(dominant.tx);
    yValue = mapRxToDac(dominant.rxOffset);
    sizeValue = mapSizeToDac(dominant.size);
  }

  writeDacOutputsIfChanged(xValue, yValue, sizeValue, touchFlagValue);
}

uint16_t hashActiveTouches() {
  uint16_t hash = 21661U;

  for (uint8_t i = 0; i < ACTIVE_TOUCH_BYTES; i++) {
    hash = (uint16_t)((hash ^ activeTouchBits[i]) * 167U);
  }

  return hash;
}

uint8_t countActiveCells() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MTK_NUM_CELLS; i++) {
    if (isTouchActive(i)) {
      count++;
    }
  }
  return count;
}

void printActiveTouches(bool stateChanged) {
#if MIDI_DEMO_MODE
  return;
#endif
  static uint16_t lastPrintedHash = 0xFFFF;
  uint8_t activeCellCount = countActiveCells();
  uint16_t frameHash = hashActiveTouches();

  if (!stateChanged && activeCellCount == 0) {
    return;
  }

  if (frameHash == lastPrintedHash) {
    return;
  }

  lastPrintedHash = frameHash;

  if (activeCellCount == 0) {
    Serial.println(F("none"));
    return;
  }

  Serial.print(F("cells "));
  Serial.print(activeCellCount);
  Serial.print(F(": "));

  bool first = true;
  for (uint8_t tx = 0; tx < MTK_NUM_TX; tx++) {
    for (uint8_t rxOffset = 0; rxOffset < MTK_RX_COUNT; rxOffset++) {
      if (!isTouchActive(toCellIndex(tx, rxOffset))) {
        continue;
      }

      if (!first) {
        Serial.print(' ');
      }

      Serial.print('(');
      Serial.print(tx);
      Serial.print(',');
      Serial.print(toPhysicalRx(rxOffset));
      Serial.print(')');
      first = false;
    }
  }

  Serial.println();
}

#if MIDI_DEMO_MODE
void sendMidiFromTouch(const DominantTouch &dt, bool changed) {
  static bool wasValid = false;
  static uint8_t lastNoteSent = 0;

  if (!changed) {
    return;
  }

  if (dt.valid && !wasValid) {
    // Map TX (0..MTK_NUM_TX-1) to a pentatonic scale so it sounds musical
    // rather than chromatic-and-random under fast movement.
    static const uint8_t pentatonicScale[MTK_NUM_TX] = {60, 62, 65, 67, 69, 72, 74, 77};
    uint8_t note = pentatonicScale[dt.tx];
    uint8_t velocity = (uint8_t)map(dt.size, 0, MAX_DOMINANT_CLUSTER_SIZE, 40, 120);
    Serial.write((uint8_t)0x90);
    Serial.write(note);
    Serial.write(velocity);
    lastNoteSent = note;
    wasValid = true;
  } else if (!dt.valid && wasValid) {
    Serial.write((uint8_t)0x80);
    Serial.write(lastNoteSent);
    Serial.write((uint8_t)0);
    wasValid = false;
  }

  if (dt.valid) {
    // CC74 (filter cutoff) from Y, continuous while touched -- audibly
    // sweeps dark-to-bright, unlike CC1/mod wheel which most patches
    // barely react to.
    uint8_t ccValue = (uint8_t)map(dt.rxOffset, 0, MTK_RX_COUNT - 1, 0, 127);
    Serial.write((uint8_t)0xB0);
    Serial.write((uint8_t)74);
    Serial.write(ccValue);
  }
}
#endif

void setup() {
  clearTouchState();
  mtk.setup_sensor();
  initTrigInput();

  Serial.begin(115200);
#if ENABLE_CONTROL_PADS
  initRecordLed();
#endif

  if (!initDac()) {
    Serial.println(F("Failed to find MCP4728 chip"));
    while (1) {
      delay(10);
    }
  }

  delay(250);
  calibrateBaseline();
#if ENABLE_CONTROL_PADS
  calibrateControlPads();
#endif
#if !MIDI_DEMO_MODE
  Serial.println("Setup done...");
#if EVAL_MODE != 0
  Serial.print(F("Eval mode: "));
  Serial.println(EVAL_MODE);
#endif
#endif
}

#if EVAL_MODE == 1
static unsigned long evalFrameCount = 0;
static unsigned long evalLastReportMs = 0;
#endif
#if EVAL_MODE == 3 || EVAL_MODE == 5
static bool evalWasValidLastFrame = false;
#endif

void loop() {
  bool stateChanged = false;

  if (!mtk.consumeFrame()) {
    return;
  }

#if EVAL_MODE == 1
  evalFrameCount++;
#endif

#if EVAL_MODE == 2
  unsigned long evalT0 = micros();
#endif

  DominantTouch dominant = scanTouchMatrix(&stateChanged);

#if MIDI_DEMO_MODE
  sendMidiFromTouch(dominant, stateChanged);
#endif

#if EVAL_MODE == 3
  if (dominant.valid && !evalWasValidLastFrame) {
    Serial.print(F("TOUCH,"));
    Serial.print(dominant.tx);
    Serial.print(',');
    Serial.println(toPhysicalRx(dominant.rxOffset));
  }
  evalWasValidLastFrame = dominant.valid;
#endif

#if EVAL_MODE == 5
  if (dominant.valid && !evalWasValidLastFrame) {
    Serial.print(F("SIZE,"));
    Serial.print(dominant.tx);
    Serial.print(',');
    Serial.print(toPhysicalRx(dominant.rxOffset));
    Serial.print(',');
    Serial.println(dominant.size);
  }
  evalWasValidLastFrame = dominant.valid;
#endif

#if ENABLE_CONTROL_PADS
  bool replaying = handleControlPads(dominant);
  updateRecordLed();
#else
  bool replaying = false;
#endif

  if (!replaying) {
    updateDacFromDominantTouch(dominant);
  }

#if EVAL_MODE == 2
  unsigned long evalT1 = micros();
  Serial.print(F("LATENCY_US,"));
  Serial.println(evalT1 - evalT0);
#endif

  printActiveTouches(stateChanged);

#if EVAL_MODE == 1
  unsigned long evalNow = millis();
  if (evalNow - evalLastReportMs >= 1000) {
    Serial.print(F("FPS,"));
    Serial.println(evalFrameCount);
    evalFrameCount = 0;
    evalLastReportMs = evalNow;
  }
#endif
}
