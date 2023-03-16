#ifndef DMXBOARD_H
#define DMXBOARD_H

#define REV_II 2
#define REV_IIIA 3

/* #define REV	REV_II */
#define REV REV_IIIA

#ifndef REV
#pragma message "Please set a board revision in dmxboard.h"
#endif

#define GIT "Not set"	/* please set manually before flashing */

#if REV == REV_II
#define DMXBOARDREV "dmxboard rev II"
#endif

#if REV == REV_IIIA
#define DMXBOARDREV "dmxboard rev IIIA"
#endif

#endif //  DMXBOARD_H
