//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef TIMER_H_
#define TIMER_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)


extern volatile uint32_t timer_delayCount;

void
timer_start (void);

void
timer_sleep (uint32_t ticks);

// ----------------------------------------------------------------------------

#endif // TIMER_H_
