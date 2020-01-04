#ifndef UTM_H
#define UTM_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "define.h"

void UTM_Deg2utm(double dCoordinate[][2], double dUTM_Coordinate[][2], uint8_t ui8_Utmzone[][2], uint32_t ui32_Coor_Size);
double Path_Heading_Calculate ( double dUTM_Coordinate[][2] , uint32_t Waypoint_Index_1, uint32_t Waypoint_Index_2 );

#endif /* UTM_H_ */

