/* 
 * File:   vtouch_build.h
 * Author: root
 *
 * Created on September 20, 2017, 12:30 PM
 */

#ifndef VTOUCH_BUILD_H
#define	VTOUCH_BUILD_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "bl810hc.h"

	const uint8_t *build_date = __DATE__, *build_time = __TIME__,
		build_version[] = " V0.05";


#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_BUILD_H */

