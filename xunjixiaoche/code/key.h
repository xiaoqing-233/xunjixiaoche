#ifndef KEY_H
#define KEY_H

#include <stdint.h>

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1_PRESSED,
    KEY_EVENT_2_PRESSED,
    KEY_EVENT_3_PRESSED
} KeyEvent;

void key_control(void);
KeyEvent key_get_last_event(void);
KeyEvent key_take_last_event(void);

#endif
