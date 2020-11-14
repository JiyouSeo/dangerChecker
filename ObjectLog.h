#include "CenterPoint.h"
struct ObjectLog
{
    long frame;
    double X;
    double Y;
    long objectId;
    ObjectLog(long _f,double _x, double _y, long _o) : frame(_f), X(_x), Y(_y),objectId(_o) {};
};

