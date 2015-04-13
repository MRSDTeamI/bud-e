#ifndef ROBOT_CART_CONSTANTS_H_INCLUDED
#define ROBOT_CART_CONSTANTS_H_INCLUDED

// -----------------------------------------------------------------------------------

// Connection : 2011/08 for No.2
#define ROBOT_CART_IP     "172.24.49.7"
#define ROBOT_CART_PORT   10001

// Connection : old setting 2011/04
// #define ROBOT_CART_IP     "192.168.20.107"
// #define ROBOT_CART_PORT   10002

// Connection : old setting 2010/09
// #define ROBOT_CART_IP     "192.168.20.14"
// #define ROBOT_CART_PORT   10001

// 500 and -500 are the real maximum and minimum as commands
#define MAX_TRANS_VELOCITY  150
#define MIN_TRANS_VELOCITY  -150

#define MAX_ROT_VELOCITY   500
#define MIN_ROT_VELOCITY  -500

#endif // ROBOT_CART_CONSTANTS_H_INCLUDED


