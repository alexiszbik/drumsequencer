#ifndef SWITCH_H
#define SWITCH_H

using namespace admux;
Mux mux(Pin(9, INPUT, PinType::Digital), Pinset(2, 3, 4, 5));

class MuxSwitch {
public:
  MuxSwitch(byte pin, bool inverted = false) : pin(pin), inverted(inverted) {
  }

  bool debounce() {
    byte readValue = mux.read(pin);
    bool currentState = inverted ? (readValue == HIGH) : (readValue == LOW);
    if (currentState != state) {
      state = currentState;
      stateChanged = true;
    }
    
    return stateChanged;
  }

  bool getState() {
    stateChanged = false;
    return state;
  }

private:
  byte pin;
  bool analogPin;
  bool inverted;

  bool state = false;
  bool stateChanged = false;
};


#endif //SWITCH_H