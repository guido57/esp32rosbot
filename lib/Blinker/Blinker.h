#ifndef BLINKER_H
#define BLINKER_H

class Blinker {
public:
    Blinker(int pin);
    void begin();
    void toggle();
    bool getState() const;

private:
    int _pin;
    bool _state;
};

#endif
