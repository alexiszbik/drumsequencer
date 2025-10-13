
#include <uClock.h>
#include "MidiOut.h"
#include "Mux.h"
#include "MuxSwitch.h"

#define SR_LATCH_PIN 8
#define SR_CLOCK_PIN 12
#define SR_DATA_PIN 11

#define SW_SELECT 6
#define SW_SHIFT 7
#define SW_BARS 10 //note sure yet

#define POT_TEMPO A7
#define POT_GROOVE A6
#define POT_VELOCITY A5

byte sequence[maxChanCount][stepCount * maxBarCount];
bool isMuted[maxChanCount];

byte seqPos = 0;
byte currentBar = 0; // (0 -> 7)
byte currentBarCount = 1; // (1 -> 8)

MuxSwitch* switches[stepCount];

const byte ppqn = 96;
const byte stepLen = ppqn/4;
const byte halfStepLen = stepLen/2;

float groove = 0.f;
byte velocity = 0;
byte selectedChannel = 0;

MidiOut midiOut;

enum Mode {
    sequencer,
    selectChannel,
    muteChannel, 
    eraseChannel,
    selectBars
};

Mode currentMode = sequencer;

uint16_t boolArrayToUint16(bool* bits) {
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        if (bits[i]) {
            value |= (1 << i);
        }
    }
    return value;
}

int getStepOffset() {
    return (currentBar * stepCount);
}

void checkPotentiometers() {
    int potTempo = analogRead(POT_TEMPO);
    int tempo = map(potTempo, 0, 1023, 40, 230);

    uClock.setTempo(tempo);

    float potGroove = (float)analogRead(POT_GROOVE);
    groove = potGroove/1023.f;

    int potVelocity = analogRead(POT_VELOCITY);
    velocity = map(potVelocity, 0, 1023, 1, 127);
}

void onOutputPPQNCallback(uint32_t tick) {

    digitalWrite(SR_LATCH_PIN, LOW);

    bool binState[stepCount];
    memset(binState, 0, stepCount*sizeof(bool));

    bool isOddStep = (tick % (stepLen*2) >= stepLen);

    if (currentMode == selectBars) {
        for (byte i = 0; i < 8; i++) {
            binState[i] = i < currentBarCount;
        }
        byte bc = currentBar + 8;
        for (byte i = 8; i < 16; i++) {
            binState[i] = i == bc;
        }
    }

    if (currentMode == muteChannel) {
        for (byte i = 0; i < maxChanCount; i++) {
            binState[i] = !isMuted[i];
        }
    }

    if (currentMode == sequencer) {
        //fill with sequencer state
        int stepOffset = getStepOffset();
        for (byte i = 0; i < stepCount; i++) {
            binState[i] = sequence[selectedChannel][i + stepOffset] > 0;
        }
        //blink !
        //TODO : blink only the current bar
        byte stepPos = seqPos % stepCount;
        if (tick % stepLen < halfStepLen) {
            binState[stepPos] = !binState[stepPos];
        }
    }

    if (currentMode == selectChannel) {
        binState[selectedChannel] = true;
    }

    byte grooveOffset = 0;

    if (isOddStep) {
        grooveOffset = groove*halfStepLen;
    }

    if (tick % stepLen == grooveOffset) {
        midiOut.release();
    }

    for (byte c = 0; c < maxChanCount; c++) {
        byte noteValue = sequence[c][seqPos];
        if (noteValue > 0 && !isMuted[c]) {
            if (tick % stepLen == grooveOffset) {
                midiOut.trigChannel(c, noteValue);
            }
            if (currentMode == selectChannel || currentMode == muteChannel || currentMode == eraseChannel) {
                binState[c] = (tick % stepLen) > halfStepLen ? binState[c] : !binState[c];
            }
        }
    }
    
    uint16_t binValue = boolArrayToUint16(binState);

    byte lowByte  = binValue & 0xFF;        // bits 0-7
    byte highByte = (binValue >> 8) & 0xFF; // bits 8-15

    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, highByte);
    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, lowByte);

    digitalWrite(SR_LATCH_PIN, HIGH);

    if (tick % stepLen == (stepLen-1)) {
        seqPos++;
        checkPotentiometers();
    }

    seqPos = seqPos % (stepCount * currentBarCount);
}

void doEraseChannel(byte c) {
    for (byte i = 0; i < stepCount * maxBarCount; i++) {
        sequence[c][i] = 0;
    }
}

void selectBarCount(byte barCount) {
    if (barCount >= 1 && barCount <= maxBarCount) {
        currentBarCount = barCount;
    }
}

void setup() {

    for (int i = 0; i < stepCount; i++) {
        switches[i] = new MuxSwitch(i);
    }
/* for debug
    for (int i = 16 ; i < 32 ; i++) {
        sequence[0][i] = 127;
    }
*/

    pinMode(SR_LATCH_PIN, OUTPUT);
    pinMode(SR_CLOCK_PIN, OUTPUT);
    pinMode(SR_DATA_PIN, OUTPUT);

    pinMode(SW_SELECT, INPUT_PULLUP);
    pinMode(SW_SHIFT, INPUT_PULLUP);
    pinMode(SW_BARS, INPUT_PULLUP);

    uClock.setOutputPPQN(uClock.PPQN_96);
    uClock.setOnOutputPPQN(onOutputPPQNCallback);

    MIDI.begin(MIDI_CHANNEL_OMNI);

    checkPotentiometers();

    uClock.init();
    uClock.setTempo(120);
    uClock.start();
}

void loop() {

    uClock.run();

    bool selectButtonIsDown = digitalRead(SW_SELECT) == HIGH;
    bool shiftButtonIsDown = digitalRead(SW_SHIFT) == HIGH;
    bool barsButtonIsDown = digitalRead(SW_BARS) == HIGH;

    if (shiftButtonIsDown && barsButtonIsDown) {
        currentMode = eraseChannel;
    } else if (shiftButtonIsDown && selectButtonIsDown) {
        currentMode = muteChannel;
    } else if (barsButtonIsDown) {
        currentMode = selectBars;
    } else if (selectButtonIsDown) {
        currentMode = selectChannel;
    } else {
        currentMode = sequencer;
    }

    for (byte i = 0; i < stepCount; i++) {
        if (switches[i]->debounce()) {
            if (switches[i]->getState() == true) {
                if (currentMode == selectBars) {
                    if (i < 8) {
                        selectBarCount(i + 1);
                    } else {
                        currentBar = i - 8;
                    }
                } else if (currentMode == eraseChannel) {
                    doEraseChannel(i);
                } else if (currentMode == muteChannel) {
                    isMuted[i] = !isMuted[i];
                } else if (currentMode == selectChannel) {
                    selectedChannel = i;
                } else {
                    int step = i + getStepOffset();
                    byte stepValue = sequence[selectedChannel][step];
                    if (stepValue > 0) {
                        sequence[selectedChannel][step] = 0;
                    } else {
                        sequence[selectedChannel][step] = velocity;
                    }
                }
            }
        }
    }
}
