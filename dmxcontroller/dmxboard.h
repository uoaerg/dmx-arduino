#ifndef DMXBOARD_H
#define DMXBOARD_H

#define REV_II 2
#define REV_IIIA 3

//#define REV	REV_II
#define REV REV_IIIA

#ifndef REV
#pragma message "please set a board revision in dmxboard.h"
#endif

#define GIT "not set"	/* please set manually before flashing */

#ifdef REV == REV_II
#define DMXBOARDREV "dmxboard rev II"
#endif

#ifdef REV == REV_IIIA
#define DMXBOARDREV "dmxboard rev IIIA"
#endif

#endif //  DMXBOARD_H
