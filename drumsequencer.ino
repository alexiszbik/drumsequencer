
#include <uClock.h>
#include "MIDI.h"
#include "Mux.h"
#include "Switch.h"

#define SR_LATCH_PIN 8
#define SR_CLOCK_PIN 12
#define SR_DATA_PIN 11

#define SW_SELECT 6

#define POT_TEMPO A7
#define POT_GROOVE A6

const byte stepCount = 16;
const byte maxChanCount = stepCount;

bool seq[maxChanCount][stepCount];
byte seqPos = 0;

Switch* switches[stepCount];

bool selectButtonIsDown = false;

byte selectedChannel = 0;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

const byte ppqn = 96;
const byte stepLen = ppqn/4;
const byte halfStepLen = stepLen/2;

float groove = 0.f;

void checkPotentiometers() {
    int potTempo = analogRead(POT_TEMPO);
    int tempo = map(potTempo, 0, 1023, 40, 230);

    uClock.setTempo(tempo);

    float potGroove = (float)analogRead(POT_GROOVE);
    groove = potGroove/1023.f;
}

uint16_t boolArrayToUint16(bool* bits) {
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        if (bits[i]) {
            value |= (1 << i);
        }
    }
    return value;
}

void onOutputPPQNCallback(uint32_t tick) {

    digitalWrite(SR_LATCH_PIN, LOW);

    bool binState[stepCount];
    memset(binState, 0, stepCount*sizeof(bool));

    //bool isOddStep = tick > 

    if (!selectButtonIsDown) {
        memcpy(binState, seq[selectedChannel], stepCount*sizeof(bool)); //copy state of sequencer
        if (tick % stepLen < halfStepLen) {
            binState[seqPos] = !binState[seqPos];
        }
    }

    if (selectButtonIsDown) {
        binState[selectedChannel] = true;
    }

    for (byte c = 0; c < maxChanCount; c++) {
        if (seq[c][seqPos]) {
            if (tick % stepLen == 0) {
                MIDI.sendNoteOn(36 + c, 127, 1);
            }
            if (tick % stepLen == halfStepLen) {
                MIDI.sendNoteOff(36 + c, 127, 1);
            }
            if (selectButtonIsDown) { //select is down => we blink buttons that correspond to channel playing
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

    seqPos = seqPos % 16;
}

void setup() {

    for (int i = 0; i < stepCount; i++) {
        switches[i] = new Switch(i);
    }

    pinMode(SR_LATCH_PIN, OUTPUT);
    pinMode(SR_CLOCK_PIN, OUTPUT);
    pinMode(SR_DATA_PIN, OUTPUT);

    pinMode(SW_SELECT, INPUT_PULLUP);

    uClock.setOutputPPQN(uClock.PPQN_96);
    uClock.setOnOutputPPQN(onOutputPPQNCallback);

    MIDI.begin(MIDI_CHANNEL_OMNI);

    uClock.init();
    uClock.setTempo(120);
    uClock.start();
  
}

void loop() {
    selectButtonIsDown = digitalRead(SW_SELECT) == HIGH;

    for (byte i = 0; i < stepCount; i++) {
        
        if (switches[i]->debounce()) {
            if (switches[i]->getState() == true) {
                if (selectButtonIsDown) {
                    selectedChannel = i;
                } else {
                    seq[selectedChannel][i] = !seq[selectedChannel][i];
                }
                
            }
        }
    }

    delay(2);
}
