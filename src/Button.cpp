#include "config.h"

void check_buttons(Button *fbuttons)
{
    uint8_t i;
    uint8_t buttonState;

    // check the buttons
    for ( i = 0 ; i < BUTTON_COUNT ; i ++ )
    {
        if ( fbuttons[i].mcp != 0 )
        {
            buttonState = fbuttons[i].mcp->digitalRead(fbuttons[i].pin);
        }
        else
        {
            buttonState = digitalRead(fbuttons[i].pin);
        }
        if ( buttonState != fbuttons[i].lastState )
        {
            if ( millis() < fbuttons[i].debounceTime )
            {
                fbuttons[i].debounceTime = millis();
            }
            if ( buttonState == fbuttons[i].debounceState )
            {
                if ( millis() - fbuttons[i].debounceTime > BUTTON_DEBOUNCE_DELAY )
                {
                    if ( fbuttons[i].lastState == LOW && buttonState == HIGH )
                    {
                        if ( millis() - fbuttons[i].pressTime > BUTTON_LONGPRESS_THRESHOLD )
                        {
                            button_onLongClick(&fbuttons[i]);
                        }
                        else
                        {
                            button_onClick(&fbuttons[i]);
                        }
                    }
                    button_onRelease(&fbuttons[i]);

                    if ( fbuttons[i].lastState == HIGH && buttonState == LOW )
                    {
                        button_onPress(&fbuttons[i]);
                        fbuttons[i].pressTime = millis();
                    }

                    fbuttons[i].lastState = buttonState;
                }
            }
            else
            {
                fbuttons[i].debounceState = buttonState;
                fbuttons[i].debounceTime = millis();
            }
        }
    }
}

void setup_buttons(Button *fbuttons)
{
    uint8_t i;

    for ( i = 0 ; i < BUTTON_COUNT ; i ++ )
    {
        Log.info("setting up %s", fbuttons[i].name);
        if ( fbuttons[i].mcp != 0 )
        {
            fbuttons[i].mcp->pinMode(fbuttons[i].pin, INPUT);
            fbuttons[i].mcp->pullUp(fbuttons[i].pin, HIGH);  // turn on a 100K pullup internally
        }
        else
        {
            pinMode(fbuttons[i].pin, INPUT_PULLUP);
        }
        fbuttons[i].debounceState = HIGH;
        fbuttons[i].lastState = HIGH;
    }
}
