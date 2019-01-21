#ifndef MSG_H
#define MSG_H
namespace cubot
{

    typedef struct {
    double x;
    double y;
    double z;
    } Liner;
    typedef struct {
    double x;
    double y;
    double z;
    } Angular;

    typedef struct
    {
    Liner linear;
    Angular angular;
    } Twist;

    #define KEYCODE_W 0x77
    #define KEYCODE_A 0x61
    #define KEYCODE_S 0x73
    #define KEYCODE_D 0x64

    #define KEYCODE_A_CAP 0x41
    #define KEYCODE_D_CAP 0x44
    #define KEYCODE_S_CAP 0x53
    #define KEYCODE_W_CAP 0x57



}
#endif
