#ifndef BUTTON_HPP
#define BUTTON_HPP

class Button {
public:
    
    Button(uint8_t buttonPin)
        : buttonPin_(buttonPin), lastState_(HIGH), lastDebounceTime_(0), debounceDelay_(50) {
        pinMode(buttonPin_, INPUT_PULLUP);  // Set pin as input with internal pull-up resistor
    }

    bool pressed() {
        /**
         * Read the current state of the button and return true if the button
         * is pressed. A debounce time is used to filter out noise from the button press.
         */
        int currentState = digitalRead(buttonPin_);
        updateState(currentState);
        return (lastState_ == LOW && stateChanged_ && currentState_ == LOW);
    }

    bool released() {
        /**
         * Read the current state of the button and return true if the button
         * is released. A debounce time is used to filter out noise from the button release.
         */
        int currentState = digitalRead(buttonPin_);
        updateState(currentState);
        return (lastState_ == HIGH && stateChanged_ && currentState_ == HIGH);
    }

    int getPin() {
        return buttonPin_;
    }

private:

    uint8_t buttonPin_;
    int lastState_;
    int currentState_;
    bool stateChanged_;
    unsigned long lastDebounceTime_;
    const unsigned long debounceDelay_;

    void updateState(int currentState) {
        /**
         * Update the button state and check for changes. This function handles
         * the debouncing of the button press.
         */

        unsigned long currentTime = millis();

        if (currentState != lastState_) {
            lastDebounceTime_ = currentTime;
        }

        if ((currentTime - lastDebounceTime_) > debounceDelay_) {
            if (currentState != currentState_) {
                currentState_ = currentState;
                stateChanged_ = true;
            } else {
                stateChanged_ = false;
            }
        }

        lastState_ = currentState;
    }
};

#endif // BUTTON_HPP
