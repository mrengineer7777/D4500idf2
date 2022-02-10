#pragma once

//https://isocpp.org/wiki/faq/mixing-c-and-cpp#include-c-hdrs-system

#include "stdint.h"

//--------------------Constants--------------------
#define WDT_TIMEOUT 5                                                               //5 second WDT timeout

//--------------------Structures--------------------

//--------------------Variables--------------------

//--------------------Functions--------------------

//-------------------Macros------------------------

class MicroframeClass
{
public:
    MicroframeClass();
    ~MicroframeClass();
protected:
private:
};

extern MicroframeClass Microframe;