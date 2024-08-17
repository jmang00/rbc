/**
 * An LED
 * 
 */

class LED {
private:
    int _pin;
public:
    LED(int pin) {
        _pin = pin;
        pinMode(_pin, OUTPUT);
    };

    void blink(int n, int t) {
        // Blink n times, for t ms each time
        for (int i=0; i<n-1; i++) {
        digitalWrite(_pin, HIGH);
        delay(t);
        digitalWrite(_pin, LOW);
        delay(t);
        };
        digitalWrite(_pin, LOW);
    };
};