
#include <uClock.h>
#include "MIDI.h"
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

const byte stepCount = 16;
const byte maxChanCount = stepCount;

byte sequence[maxChanCount][stepCount];
byte seqPos = 0;
bool isMuted[maxChanCount];

MuxSwitch* switches[stepCount];

const byte ppqn = 96;
const byte stepLen = ppqn/4;
const byte halfStepLen = stepLen/2;

float groove = 0.f;
byte velocity = 0;
byte selectedChannel = 0;

enum Mode {
    sequencer,
    selectChannel,
    muteChannel, 
    eraseChannel
};

Mode currentMode = sequencer;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

uint16_t boolArrayToUint16(bool* bits) {
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        if (bits[i]) {
            value |= (1 << i);
        }
    }
    return value;
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

    if (currentMode == muteChannel) {
        for (byte i = 0; i < maxChanCount; i++) {
            binState[i] = !isMuted[i];
        }
    }

    if (currentMode == sequencer) {
        for (byte i = 0; i < stepCount; i++) {
            binState[i] = sequence[selectedChannel][i] > 0;
        }
        if (tick % stepLen < halfStepLen) {
            binState[seqPos] = !binState[seqPos];
        }
    }

    if (currentMode == selectChannel) {
        binState[selectedChannel] = true;
    }

    byte grooveOffset = 0;

    if (isOddStep) {
        grooveOffset = groove*halfStepLen;
    }

    for (byte c = 0; c < maxChanCount; c++) {
        byte noteValue = sequence[c][seqPos];
        if (noteValue > 0 && !isMuted[c]) {
            if (tick % stepLen == grooveOffset) {
                MIDI.sendNoteOn(36 + c, noteValue, 1); //TODO => store midi state !
            }
            if (tick % stepLen == (halfStepLen + grooveOffset)) { //TODO => this should be called elsewhere !
                MIDI.sendNoteOff(36 + c, 127, 1);
            }
            if (currentMode == selectChannel || currentMode == muteChannel || currentMode == eraseChannel) { //select is down => we blink buttons that correspond to channel playing
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

    seqPos = seqPos % stepCount;
}

void doEraseChannel(byte c) {
    for (byte i = 0; i < stepCount; i++) {
        sequence[c][i] = 0;
    }
}

void setup() {

    for (int i = 0; i < stepCount; i++) {
        switches[i] = new MuxSwitch(i);
    }

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

    bool selectButtonIsDown = digitalRead(SW_SELECT) == HIGH;
    bool shiftButtonIsDown = digitalRead(SW_SHIFT) == HIGH;
    bool barsButtonIsDown = digitalRead(SW_BARS) == HIGH;

    if (shiftButtonIsDown && barsButtonIsDown) {
        currentMode = eraseChannel;
    } else if (shiftButtonIsDown && selectButtonIsDown) {
        currentMode = muteChannel;
    } else if (selectButtonIsDown) {
        currentMode = selectChannel;
    } else {
        currentMode = sequencer;
    }

    for (byte i = 0; i < stepCount; i++) {
        
        if (switches[i]->debounce()) {
            if (switches[i]->getState() == true) {
                if (currentMode == eraseChannel) {
                    doEraseChannel(i);
                } else if (currentMode == muteChannel) {
                    isMuted[i] = !isMuted[i];
                } else if (currentMode == selectChannel) {
                    selectedChannel = i;
                } else {
                    byte stepValue = sequence[selectedChannel][i];
                    if (stepValue > 0) {
                        sequence[selectedChannel][i] = 0;
                    } else {
                        sequence[selectedChannel][i] = velocity;
                    }
                }
                
            }
        }
    }

    delay(2);
}
