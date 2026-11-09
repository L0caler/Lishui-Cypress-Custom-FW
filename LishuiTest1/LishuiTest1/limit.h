// limit.h - battery current target from Cfg and Live.  Main loop only.
#ifndef LIMIT_H
#define LIMIT_H

#include "config.h"

void Limit_Update(void);     // after Live or Cfg changed

#endif
