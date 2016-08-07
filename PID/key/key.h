

#ifndef __KEY_H
#define __KEY_H


/*The Menu status machine*/

#define NONE                    0
#define MAIN_MENU               1
#define SET_TEMP_MENU           2
#define SET_TIME_MENU           3
#define MAX_TEMP_VALUE          99
#define TYPE_TEMP_VALUE			40
#define MIN_TEMP_VALUE          1
#define MAX_TIME_VALUE          200
#define TYPE_TIME_VALUE			20
#define MIN_TIME_VALUE          1
#define STOP                    0
#define START					1

 //init the HW key GPIO port and pin opreation that it's must be call init of system start ....
 void ConfigHWkey(void);
 //scane key board ever 500MS-100MS price once again 
 void KeyScane(uint32_t Time);











#endif