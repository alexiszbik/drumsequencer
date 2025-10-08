
#include <uClock.h>
#include "MIDI.h"
#include "Mux.h"
#include "Switch.h"

#define SR_LATCH_PIN 8
#define SR_CLOCK_PIN 12
#define SR_DATA_PIN 11

#define SW_SELECT 6

const byte stepCount = 16;
const byte maxChanCount = 16;

bool seq[maxChanCount][stepCount];
byte seqPos = 0;

Switch* switches[stepCount];

bool select = false;

byte chan = 0;

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

void onOutputPPQNCallback(uint32_t tick) {

    digitalWrite(SR_LATCH_PIN, LOW);

    bool binState[stepCount];
    memset(binState, 0, stepCount*sizeof(bool));
    if (!select) {
        memcpy(binState, seq[chan], stepCount*sizeof(bool)); //copy state of sequencer
        if (tick % 6 < 3) {
            binState[seqPos] = !binState[seqPos];
        }
    }

    for (byte c = 0; c < maxChanCount; c++) {
        if (seq[c][seqPos]) {
            if (tick % 6 == 0) {
                MIDI.sendNoteOn(36 + c, 127, 1);
                if (select) {
                    binState[c] = true; //clignotte trop vite
                }
            }
            if (tick % 6 == 3) {
                MIDI.sendNoteOff(36 + c, 127, 1);
                if (select) {
                    binState[c] = false;
                }
            }
        }
    }

    if (select) {
        binState[chan] = true;
    }
    
    uint16_t binValue = boolArrayToUint16(binState);

    byte lowByte  = binValue & 0xFF;        // bits 0-7
    byte highByte = (binValue >> 8) & 0xFF; // bits 8-15

    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, highByte);
    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, lowByte);

    digitalWrite(SR_LATCH_PIN, HIGH);

    if (tick % 6 == 5) {
        seqPos++;
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

    uClock.setOutputPPQN(uClock.PPQN_24);
    uClock.setOnOutputPPQN(onOutputPPQNCallback);

    MIDI.begin(MIDI_CHANNEL_OMNI);

    uClock.init();
    uClock.setTempo(120);
    uClock.start();
  
}

void loop() {
    select = digitalRead(SW_SELECT) == HIGH;

    for (byte i = 0; i < stepCount; i++) {
        
        if (switches[i]->debounce()) {
            if (switches[i]->getState() == true) {
                if (select) {
                    chan = i;
                } else {
                    seq[chan][i] = !seq[chan][i];
                }
                
            }
        }
    }

    delay(2);
}
