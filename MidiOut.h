
#include "MIDI.h"
#include "Const.h"

#define MIDI_MIN 36
#define MIDI_CHANNEL 10

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

class MidiOut {
private:
    struct NoteState {
        bool triggered = false;
        byte value = 0;
    };

    NoteState noteState[maxChanCount];
    bool liveState[maxChanCount];
    unsigned long time = 0;

public:
    MidiOut() {
        Serial1.begin(31250);
        memset(liveState, 0, maxChanCount * sizeof(bool));
    }

    void loadNote(byte c, byte velocity) {
        noteState[c].value = velocity;
        noteState[c].triggered = false;
    }

    void setTime(unsigned long time) {
        this->time = time;
    }

    void release() {
        for (byte c = 0; c < maxChanCount; c++) {
            if (noteState[c].value > 0) {
                MIDI.sendNoteOff(MIDI_MIN + c, 127, MIDI_CHANNEL);
                noteState[c].value = 0; 
            }
        } 
    }

    void sendOutput(unsigned long time) {
        if (this->time <= time) {
            for (byte c = 0; c < maxChanCount; c++) {
                if (noteState[c].value > 0 && noteState[c].triggered == false) {
                    MIDI.sendNoteOn(MIDI_MIN + c, noteState[c].value, MIDI_CHANNEL); 
                    noteState[c].triggered = true; 
                }
            }
        }
    }

    void performNoteNow(byte c, byte velocity, bool isRepeat) {
        if (!isRepeat) {
            MIDI.sendNoteOn(MIDI_MIN + c, velocity, MIDI_CHANNEL); 
        }
        liveState[c] = true;
    }

    void releaseNoteNow(byte c, bool isRepeat) {
        if (liveState[c] && !isRepeat) {
            MIDI.sendNoteOff(MIDI_MIN + c, 127, MIDI_CHANNEL);
        }
        liveState[c] = false;
    }

    void releaseAllLiveNotes(bool isRepeat) {
        for (byte c = 0; c < maxChanCount; c++) {
            releaseNoteNow(c, isRepeat);
        }
    }

    bool getLiveState(byte c) {
        return liveState[c];
    }
    
};