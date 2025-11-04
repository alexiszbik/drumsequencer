
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

bool isPlaying = true;
float groove = 0.f;
byte velocity = 0;
byte selectedChannel = 0;
uint32_t currentTick = 0;
byte seqPos = 0;
byte currentBar = 0; // (0 -> 7)
byte currentBarCount = 1; // (1 -> 8)
bool needRestart = false;

MuxSwitch* switches[stepCount];

Switch selectButton = Switch(SW_SELECT);
Switch shiftButton = Switch(SW_SHIFT);
Switch barsButton = Switch(SW_BARS);
Switch playButton = Switch(PLAY_BUTTON);

MidiOut midiOut;

LEDGroup ledGroup = LEDGroup(SR_LATCH_PIN, SR_CLOCK_PIN, SR_DATA_PIN);

enum Mode {
    sequencer,
    selectChannel,
    muteChannel, 
    eraseChannel, //TODO : update erase channel LEDs => display LED HIGH if channel got at least one step with > 0
    selectBars
};

Mode currentMode = sequencer;

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

byte getGrooveOffset() {
    byte grooveOffset = 0;

    if (seqPos % 2 == 1) {
        grooveOffset = groove*halfStepLen;
    }
    return grooveOffset;
}

void processLEDs() {
    bool binState[stepCount];
    memset(binState, 0, stepCount*sizeof(bool));

    bool isHalfStep = (currentTick % stepLen) <= halfStepLen;
    byte currentPlayedBar = seqPos / stepCount;

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
        if (isPlaying) {
            byte stepPos = seqPos % stepCount;
            if (isHalfStep && currentPlayedBar == currentBar) {
                binState[stepPos] = !binState[stepPos];
            }
        }
    }

    if (currentMode == selectChannel) {
        binState[selectedChannel] = true;
    }

    if (currentTick % stepLen == halfStepLen) {
      digitalWrite(SYNC_OUT, LOW);
    }

    if (currentTick % stepLen == 0) {
      digitalWrite(SYNC_OUT, HIGH);
    }

    if (isPlaying) {
        for (byte c = 0; c < maxChanCount; c++) {
            byte noteValue = sequence[c][seqPos];
            if (noteValue > 0 && !isMuted[c]) {
                if (currentMode == selectChannel || currentMode == muteChannel || currentMode == eraseChannel) {
                    binState[c] = (currentTick % stepLen) > halfStepLen ? binState[c] : !binState[c];
                }
            }
        }
    }
    
    
    ledGroup.process(binState);
}

void onOutputPPQNCallback(uint32_t tick) {
    
    currentTick = tick;

    processLEDs();

    if (currentTick % stepLen == getGrooveOffset()) {
        midiOut.release();
    }

    for (byte c = 0; c < maxChanCount; c++) {
        byte noteValue = sequence[c][seqPos];
        if (noteValue > 0 && !isMuted[c]) {
            if (currentTick % stepLen == getGrooveOffset()) {
                midiOut.trigChannel(c, noteValue);
            }
        }
    }

    if (currentTick % stepLen == (stepLen-1)) {
        seqPos++;
        checkPotentiometers(); //TODO maybe use a callback for this
        if (needRestart) {
            needRestart = false;
            seqPos = 0;
        }
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
    delay(200);

    for (int i = 0; i < stepCount; i++) {
        switches[i] = new MuxSwitch(i);
    }

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
        currentTick = 0;
        processLEDs();
    } else {
        uClock.start();
    }
}

void restartBars() {
    needRestart = true;
}

void playButtonCallback() {
    if (playButton.debounce()) {
        if (playButton.getState()) {
            if (shiftButton.getState() && isPlaying) {
                restartBars();
            } else {
                setIsPlaying(!isPlaying);
            }
        }
    }
}

TimedTask playButtonCheck(20, playButtonCallback);

void loop() {

    uClock.run();

    bool needsLedUpdate = false;

    if (selectButton.debounce() || shiftButton.debounce() || barsButton.debounce()) {
        needsLedUpdate = !isPlaying;
    }

    bool selectButtonIsDown = selectButton.getState();
    bool shiftButtonIsDown = shiftButton.getState();
    bool barsButtonIsDown = barsButton.getState();

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
            needsLedUpdate = !isPlaying;
        }
    }

    if (needsLedUpdate) {
        processLEDs();
    }
}
