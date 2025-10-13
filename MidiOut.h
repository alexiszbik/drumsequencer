
#include "MIDI.h"
#include "Const.h"

#define MIDI_MIN 36
#define MIDI_CHANNEL 1

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

class MidiOut {
private:
    bool trigState[maxChanCount];
public:
    void trigChannel(byte c, byte velocity) {
        MIDI.sendNoteOn(MIDI_MIN + c, velocity, MIDI_CHANNEL); 
        trigState[c] = true;
    }

    void release() {
        for (byte c = 0; c < maxChanCount; c++) {
            if (trigState[c]) {
                MIDI.sendNoteOff(MIDI_MIN + c, 127, MIDI_CHANNEL);
                trigState[c] = false; 
            }
        } 
    }
    
};