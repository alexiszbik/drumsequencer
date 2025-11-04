
#include <uClock.h>
#include "MidiOut.h"
#include "Mux.h"
#include "MuxSwitch.h"
#include "Switch.h"
#include "LEDGroup.h"
#include "TimedTask.h"

#define SR_LATCH_PIN 8
#define SR_CLOCK_PIN 12
#define SR_DATA_PIN 11

#define SW_SELECT 6
#define SW_SHIFT 7
#define SW_BARS 10 //note sure yet

#define POT_TEMPO A7
#define POT_GROOVE A6
#define POT_VELOCITY A5

#define SYNC_OUT A0
#define PLAY_BUTTON A1

byte sequence[maxChanCount][stepCount * maxBarCount];
bool isMuted[maxChanCount];

byte seqPos = 0;
byte currentBar = 0; // (0 -> 7)
byte currentBarCount = 1; // (1 -> 8)

MuxSwitch* switches[stepCount];

Switch playButton = Switch(PLAY_BUTTON);

float groove = 0.f;
byte velocity = 0;
byte selectedChannel = 0;

MidiOut midiOut;

LEDGroup ledGroup = LEDGroup(SR_LATCH_PIN, SR_CLOCK_PIN, SR_DATA_PIN);

bool isPlaying = true;

enum Mode {
    sequencer,
    selectChannel,
    muteChannel, 
    eraseChannel,
    selectBars
};

Mode currentMode = sequencer;

bool selectButtonIsDown = false;
bool shiftButtonIsDown = false;
bool barsButtonIsDown = false;

void playButtonCallback() {
    if (playButton.debounce()) {
        if (playButton.getState()) {
            if (shiftButtonIsDown && isPlaying) {
                setIsPlaying(false);
                setIsPlaying(true);
            } else {
                setIsPlaying(!isPlaying);
            }
        }
    }
}

TimedTask playButtonCheck(20, playButtonCallback);

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

void processLEDs() {
    
}

void onOutputPPQNCallback(uint32_t tick) {
    bool isHalfStep = (tick % stepLen) <= halfStepLen;
    byte currentPlayedBar = seqPos / stepCount;

    bool binState[stepCount];
    memset(binState, 0, stepCount*sizeof(bool));

    bool isOddStep = (tick % (stepLen*2) >= stepLen);

    if (currentMode == selectBars) {
        for (byte i = 0; i < 8; i++) {
            binState[i] = (i < currentBarCount);
            if (i == currentPlayedBar) {
                binState[i] = binState[i] && isHalfStep;
            }
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
        //blink only the current bar
        byte stepPos = seqPos % stepCount;
        if (isHalfStep && currentPlayedBar == currentBar) {
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

    if (tick % stepLen == halfStepLen) {
      digitalWrite(SYNC_OUT, LOW);
    }

    if (tick % stepLen == 0) {
      digitalWrite(SYNC_OUT, HIGH);
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
    
    ledGroup.process(binState);

    if (tick % stepLen == (stepLen-1)) {
        seqPos++;
        checkPotentiometers(); //TODO maybe use a callback for this
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

    pinMode(SW_SELECT, INPUT_PULLUP);
    pinMode(SW_SHIFT, INPUT_PULLUP);
    pinMode(SW_BARS, INPUT_PULLUP);

    pinMode(SYNC_OUT, OUTPUT);

    uClock.setOutputPPQN(uClock.PPQN_96);
    uClock.setOnOutputPPQN(onOutputPPQNCallback);

    MIDI.begin(MIDI_CHANNEL_OMNI);

    checkPotentiometers();

    uClock.init();
    uClock.setTempo(120);
    uClock.start();
}

void setIsPlaying(bool state) {
    isPlaying = state;
    if (!isPlaying) {
        uClock.stop();
        midiOut.release();
        seqPos = 0;
    } else {
        uClock.start();
    }
}

void loop() {

    uClock.run();

    selectButtonIsDown = digitalRead(SW_SELECT) == HIGH;
    shiftButtonIsDown = digitalRead(SW_SHIFT) == HIGH;
    barsButtonIsDown = digitalRead(SW_BARS) == HIGH;

    playButtonCheck.update();

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
            if (switches[i]->getState()) {
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
