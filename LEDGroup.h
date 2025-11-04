#ifndef LEDGROUP_H
#define LEDGROUP_H

class LEDGroup {
public:
    LEDGroup(byte latchPin, byte clockPin, byte dataPin) : latchPin(latchPin), clockPin(clockPin), dataPin(dataPin) {
        pinMode(latchPin, OUTPUT);
        pinMode(clockPin, OUTPUT);
        pinMode(dataPin, OUTPUT);
    }

private:
    uint16_t boolArrayToUint16(bool* bits) {
        uint16_t value = 0;
        for (int i = 0; i < 16; i++) {
            if (bits[i]) {
                value |= (1 << i);
            }
        }
        return value;
    }
public:
    //binState should be size of 16 !!!
    void process(bool* binState) {
        uint16_t binValue = boolArrayToUint16(binState);

        byte lowByte  = binValue & 0xFF;        // bits 0-7
        byte highByte = (binValue >> 8) & 0xFF; // bits 8-15

        digitalWrite(latchPin, LOW);

        shiftOut(dataPin, clockPin, MSBFIRST, highByte);
        shiftOut(dataPin, clockPin, MSBFIRST, lowByte);

        digitalWrite(latchPin, HIGH);
    }

private:
    byte latchPin, clockPin, dataPin;
};

#endif //LEDGROUP_H