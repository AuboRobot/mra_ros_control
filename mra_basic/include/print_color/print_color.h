#ifndef _PRINT_COLOR_H
#define _PRINT_COLOR_H

//example: ROS_INFO("\033[01;34m Ready catch planning ready is %s %s","hello",Color_end);

// Color Start
#define Color_black "\033[22;30m"
#define Color_red "\033[22;31m" 
#define Color_green "\033[22;32m" 
#define Color_brown "\033[22;33m" 
#define Color_blue "\033[22;34m" 
#define Color_magenta "\033[22;35m" 
#define Color_cyan "\033[22;36m" 
#define Color_gray "\033[22;37m" 
#define Color_dark_gray "\033[01;30m" 
#define Color_light_red "\033[01;31m" 
#define Color_light_green "\033[01;32m" 
#define Color_yellow "\033[01;33m" 
#define Color_light_blue "\033[01;34m" 
#define Color_light_magenta "\033[01;35m" 
#define Color_light_cyan "\033[01;36m" 
#define Color_white "\033[01;37m" 

// To flush out prev settings
#define Color_end "\33[0m" 

#endif
