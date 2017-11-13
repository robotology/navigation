#ifndef __IKARTNAV_TARGETFILTER_H__
#define __IKARTNAV_TARGETFILTER_H__

#include "Vec2D.h"

class TargetFilter
{
public:
    TargetFilter();
    ~TargetFilter();

    void addPoint(Vec2D P,Vec2D U,double maxRange);
    int numSamples();
    void reset();
    Vec2D getTarget();

protected:
    Vec2D Q,W;
    Vec2D mTarget;
    int mNumSamples;
};

#endif
