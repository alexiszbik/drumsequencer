
#include "MIDI.h"
#include "Const.h"

#define MIDI_MIN 36
#define MIDI_CHANNEL 4

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

class MidiOut {
private:
    struct NoteState {
        bool triggered = false;
        byte value = 0;
    };

    NoteState noteState[maxChanCount];
    bool liveState[maxChanCount];
    bool midiInState[maxChanCount];
    unsigned long time = 0;

public:
    MidiOut() {
        for (byte i = 0; i < maxChanCount; i++) {
            liveState[i] = false;
            midiInState[i] = false;
        }
    }

    void begin() {
        Serial1.begin(31250);
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

    bool sendOutput(unsigned long time) {
        bool result = false;
        if (this->time <= time) {
            for (byte c = 0; c < maxChanCount; c++) {
                if (noteState[c].value > 0 && noteState[c].triggered == false) {
                    MIDI.sendNoteOn(MIDI_MIN + c, noteState[c].value, MIDI_CHANNEL); 
                    noteState[c].triggered = true; 
                    result = true;
                }
            }
        }
        return result;
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

    void setMidiInState(byte note, bool state) {
        int channel = note - MIDI_MIN;
        if (channel >= 0 && channel < 16) {
            midiInState[channel] = state;
        }
    }   

    bool getLiveState(byte c) {
        return liveState[c];
    }

    bool getLEDState(byte c) {
        return liveState[c] || midiInState[c];
    }
};