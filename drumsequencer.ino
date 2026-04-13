
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
#define SW_BARS 10
#define SW_STEPS A2

#define POT_TEMPO A7
#define POT_GROOVE A6
#define POT_VELOCITY A5

#define SYNC_OUT A0
#define PLAY_BUTTON A1

byte sequence[maxChanCount][stepCount * maxBarCount];
bool isMuted[maxChanCount];
bool stepState[maxChanCount];

bool isPlaying = false;
byte groove = 0;
byte velocity = 0;
byte selectedChannel = 0;
uint32_t currentTick = 0;
uint32_t midiTick = 0;
bool isMidiSynced = false;
byte seqPos = 0;
byte currentBar = 0;       // (0 -> 7)
byte currentBarCount = 1;  // (1 -> 8)
bool needRestart = false;
bool binState[stepCount];
byte tickModStep = 0;

unsigned long lifeTime = 0;
unsigned long currentTime = 0;

MuxSwitch switches[stepCount];

Switch selectButton = Switch(SW_SELECT);
Switch shiftButton = Switch(SW_SHIFT);
Switch barsButton = Switch(SW_BARS);
Switch stepsButton = Switch(SW_STEPS);
Switch playButton = Switch(PLAY_BUTTON);

MidiOut midiOut;

LEDGroup ledGroup = LEDGroup(SR_LATCH_PIN, SR_CLOCK_PIN, SR_DATA_PIN);

enum Mode {
  sequencer,
  selectChannel,
  muteChannel,
  eraseChannel,  //TODO : update erase channel LEDs => display LED HIGH if channel got at least one step with > 0
  selectBars,
  enableSteps
};

Mode currentMode = sequencer;

bool isPerform = false;
bool isRepeat = false;
bool needsLedUpdate = false;
bool shiftButtonIsDown = false;

int getStepOffset() {
  return (currentBar * stepCount);
}

void processLEDs() {
  
  for (byte i = 0; i < stepCount; i++) {
    binState[i] = 0;
  }

  
  bool isHalfStep = tickModStep <= halfStepLen;
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
    if (isPerform) {
      for (byte i = 0; i < stepCount; i++) {
        binState[i] = midiOut.getLiveState(i);
      }
    } else {
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
  }

  if (currentMode == selectChannel) {
    binState[selectedChannel] = true;
  }

  if (currentMode == enableSteps) {
    for (byte i = 0; i < maxChanCount; i++) {
      binState[i] = stepState[i];
    }
  }

  if (tickModStep == halfStepLen) {
    digitalWrite(SYNC_OUT, LOW);
  }

  if (tickModStep == 0) {
    digitalWrite(SYNC_OUT, HIGH);
  }

  if (isPlaying) {
    for (byte c = 0; c < maxChanCount; c++) {
      byte noteValue = sequence[c][seqPos];
      if (noteValue > 0 && !isMuted[c]) {
        if (currentMode == selectChannel || currentMode == muteChannel || currentMode == eraseChannel || isPerform) {
          binState[c] = tickModStep > halfStepLen ? binState[c] : !binState[c];
        }
      }
    }
  }

  ledGroup.process(binState);
}

void onTick(uint32_t tick) {

  unsigned long delta = currentTime - lifeTime;
  lifeTime = currentTime;

  currentTick = tick;

  tickModStep = currentTick % stepLen;

  if ((tickModStep & 1) == 0) {
    needsLedUpdate = true;
  }

  if (tickModStep == 0) {

    bool isOddStep = (seqPos & 1);
    unsigned long stepDuration = delta * stepLen;

    unsigned long offset = isOddStep ? ((stepDuration >> 1) * groove) >> 8 : 0;  // idem à (stepDuration / 2) * (groove / 256)

    for (byte c = 0; c < maxChanCount; c++) {
      byte noteValue = sequence[c][seqPos];
      if (midiOut.getLiveState(c) && isRepeat && isPerform) {
        midiOut.loadNote(c, velocity);
      } else if ((noteValue > 0 && !isMuted[c])) {
        midiOut.loadNote(c, noteValue);
      }
    }

    midiOut.setTime(currentTime + offset);
  }

  if (tickModStep >= (stepLen - 1)) {
    midiOut.release();
    seqPos++;
    if (needRestart) {
      needRestart = false;
      seqPos = 0;
    }
  }

  seqPos = seqPos % (stepCount * currentBarCount);
}

void onOutputPPQNCallback(uint32_t tick) {
  onTick(tick);
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
    switches[i].init(i);
  }

  memset(stepState, 1, maxChanCount * sizeof(bool));

  pinMode(SYNC_OUT, OUTPUT);

  uClock.setOutputPPQN(uClock.PPQN_24);
  uClock.setOnOutputPPQN(onOutputPPQNCallback);

  MIDI.setHandleStart(handleStart);
  MIDI.setHandleStop(handleStop);
  MIDI.setHandleClock(handleClock);

  MIDI.begin(MIDI_CHANNEL_OMNI);

  uClock.init();

  checkPotentiometersCallback();
}

void handleStart() {
  if (!isPlaying) {
    isMidiSynced = true;
    midiTick = 0;
    setIsPlaying(true);
  }
}

void handleStop() {
  setIsPlaying(false);
  isMidiSynced = false;
}

void handleClock() {
  if (isMidiSynced && isPlaying) {
    onTick(midiTick);
    midiTick = midiTick + 1;
  }
}

void setIsPlaying(bool state) {
  isPlaying = state;
  if (!isPlaying) {
    uClock.stop();
    midiOut.release();
    seqPos = 0;
    currentTick = 0;
    needsLedUpdate = true;
  } else {
    if (!isMidiSynced) {
      uClock.start();
    }
  }
}

void restartBars() {
  needRestart = true;
}


void checkPotentiometersCallback() {
  static byte currentPot = 0;
  switch (currentPot) {
    case 0:
      {
        int potTempo = analogRead(POT_TEMPO);
        int tempo = map(potTempo, 0, 1023, 40, 230);
        uClock.setTempo(tempo);
        break;
      }
    case 1:
      {
        int potGroove = analogRead(POT_GROOVE);
        groove = potGroove >> 2;  //division par 4
        break;
      }
    case 2:
      {
        int potVelocity = analogRead(POT_VELOCITY);
        velocity = map(potVelocity, 0, 1023, 1, 127);
        break;
      }
  }

  currentPot++;
  if (currentPot >= 3) currentPot = 0;
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

void otherButtonCallback() {
  if (selectButton.debounce() || shiftButton.debounce() || barsButton.debounce() || stepsButton.debounce()) {
    needsLedUpdate = !isPlaying;
  }

  bool selectButtonIsDown = selectButton.getState();
  shiftButtonIsDown = shiftButton.getState();
  bool barsButtonIsDown = barsButton.getState();
  bool stepsButtonIsDown = stepsButton.getState();

  if (shiftButtonIsDown && barsButtonIsDown) {
    currentMode = eraseChannel;
  } else if (shiftButtonIsDown && selectButtonIsDown) {
    currentMode = muteChannel;
  } else if (barsButtonIsDown) {
    currentMode = selectBars;
  } else if (selectButtonIsDown) {
    currentMode = selectChannel;
  } else if (stepsButtonIsDown) {
    currentMode = enableSteps;
  } else {
    currentMode = sequencer;
  }
}

void drumButtonCallback() {

  static byte i = 0;
  if (switches[i].debounce()) {
    if (switches[i].getState()) {
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
      } else if (currentMode == enableSteps) {
        stepState[i] = !stepState[i];
      } else if (currentMode == selectChannel) {
        selectedChannel = i;
      } else if (shiftButtonIsDown) {  //Select new modes
        if (i == 0) {
          isPerform = !isPerform;
          if (!isPerform) {
            midiOut.releaseAllLiveNotes(isRepeat);
          }
        } else if (i == 1) {
          isRepeat = !isRepeat;
        }
      } else if (isPerform) {
        midiOut.performNoteNow(i, velocity, isRepeat);
      } else {
        int step = i + getStepOffset();
        byte &stepValue = sequence[selectedChannel][step];
        stepValue = (stepValue > 0) ? 0 : velocity;
      }
    } else {
      if (isPerform) {
        midiOut.releaseNoteNow(i, isRepeat);
      }
    }
    needsLedUpdate = !isPlaying;
  }

  i++;
  if (i >= stepCount) i = 0;
}

void checkLEDs() {
  if (needsLedUpdate) {
    processLEDs();
    needsLedUpdate = false;
  }
}

void inputCheckCallback() {
  static byte checkIndex = 0;

  switch (checkIndex) {

    case 0: playButtonCallback(); break;

    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      drumButtonCallback();
      break;

    case 9: otherButtonCallback(); break;

    case 10: checkLEDs(); break;

    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
      drumButtonCallback();
      break;

    case 19: checkPotentiometersCallback(); break;
    case 20: checkLEDs(); break;
  }

  checkIndex++;
  if (checkIndex >= 21) checkIndex = 0;
}

TimedTask inputCheck(1, inputCheckCallback);

void loop() {

  if (lifeTime == 0) {
    lifeTime = millis();
    return;
  }

  currentTime = millis();

  if (!isMidiSynced) {
    uClock.run();
  }

  inputCheck.update(currentTime);

  midiOut.sendOutput(currentTime);
  MIDI.read();
}
