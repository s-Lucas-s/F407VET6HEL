#ifndef __LASER_H
#define __LASER_H

#include "main.h"
#include <stdint.h>

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_SetState(uint8_t state);

#endif
