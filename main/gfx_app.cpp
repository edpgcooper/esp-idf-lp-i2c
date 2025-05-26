#include "is31fl_display.h"

IS31FL_Display display;

extern "C" void gfx_app_start(void) 
{
    
    display.begin();
    display.clear();

    display.setCursor(0, 0);
    display.setTextColor(255);
    display.setTextSize(1);
    //display.print("h");
    display.print("HELLO\nWORLD");
    //display.print('c');
    display.display();
}


extern "C" void gfx_app_update(void) 
{
    display.display();
}