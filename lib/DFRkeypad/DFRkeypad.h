
// ##############################################
// # DFRkeypad.h                                #
// # DFRobot LCD Keypad Shield library          #
// #                                            #
// # Jim Mayhugh, 2014                          #
// #                                            #
// # This library is free software; you can     #
// # redistribute it and/or modify it under the #
// # terms of the GNU Lesser General Public     #
// # License as published by the Free Software  #
// # Foundation; either version 2.1 of the      #
// # License, or (at your option) any later     #
// # version.                                   #
// #                                            #
// # This library is distributed in the hope    #
// # that it will be useful, but WITHOUT ANY    #
// # WARRANTY; without even the implied         #
// # warranty of MERCHANTABILITY or FITNESS     #
// # FOR A PARTICULAR PURPOSE.  See the GNU     #
// # Lesser General Public License for more      #
// # details.                                   #
// #                                            #
// # You should have received a copy of the     #
// # GNU Lesser General Public License along    #
// # with this library; if not, write to the    #
// # Free Software Foundation, Inc.,            #
// # 51 Franklin St, Fifth Floor, Boston, MA    #
// # 02110-1301, USA.                           #
// ##############################################


#ifndef DFRkeypad_h
#define DFRkeypad_h

#include "Arduino.h"

namespace DFRkeypad
{
    #ifndef KEYPAD
    #define KEYPAD 0
    #endif

    extern int iDEFAULT_THRESHOLD;
    extern int iKEY_DEBOUNCE_DELAY;
    extern int iKEY_REPEAT_DELAY;
    extern byte LastKey;
    extern unsigned long LastTime;
    extern bool bRepeat;
    enum eKEY_VALUES { eNO_KEY=0, eRIGHT, eUP, eDOWN, eLEFT, eSELECT, eINVALID_KEY, eNUM_KEYS };

    extern const char* sKEY[eNUM_KEYS];
    extern const int iARV_VALUES[eNUM_KEYS];
    extern int iKEY[eNUM_KEYS];

    void FastADC(bool bFast=false);
    bool AnyKey();
    byte GetKey();
    const char* const KeyName(int val);
};

#endif // #ifndef DFRkeypad_h
