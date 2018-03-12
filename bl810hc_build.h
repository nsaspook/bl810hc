
#ifndef VTOUCH_BUILD_H
#define	VTOUCH_BUILD_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "bl810hc.h"

	const uint8_t *build_date = __DATE__, *build_time = __TIME__,
		build_version[] = " V0.07";


#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_BUILD_H */

