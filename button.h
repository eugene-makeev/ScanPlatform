/* 
 * button.h
 *
 * Created: 04.05.2015 21:38:42
 * Author: Yogi
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <Arduino.h>

#define debounceDelay 50

typedef void (*callback)(void);

class Button
{

private:
    long debounce;
    byte state :1;
    byte lastState :1;
    byte pin;
    callback cb = NULL;
    Button(const Button &c);
    Button& operator=(const Button &c);

public:

    Button(byte p, callback c) :
            debounce(0), state(1), lastState(1), cb(c), pin(p)
    {
        pinMode(pin, INPUT_PULLUP);
    }
 
  Button(byte p) :
      debounce(0), state(1), lastState(1), pin(p)
  {
    pinMode(pin, INPUT_PULLUP);
  }

  void setCallback(callback c)
  {
    cb = c;
  }

  bool getState(void)
  {
    return state; // debounced state
  }

    bool check()
    {
        long t = millis();

        if (digitalRead(pin) != lastState)
        {
            lastState = !lastState;
            debounce = t;
        }
        else if ((t - debounce) > debounceDelay)
        {
            if (state != lastState)
            {
                state = lastState;

                if (!state)
                {
                    if (cb != NULL)
                    {
                      (*cb)();
                    }
          
                    return true;
                }
            }
        }
    
        return false;
    }
};
//button

#endif //__BUTTON_H__
