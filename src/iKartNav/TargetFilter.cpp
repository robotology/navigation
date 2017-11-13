#include "TargetFilter.h"
#include "yarp/os/Log.h"

TargetFilter::TargetFilter()
{
    mNumSamples=0;
    Q=Vec2D(100.0,100.0);
}

TargetFilter::~TargetFilter()
{
}

void TargetFilter::addPoint(Vec2D P,Vec2D U,double maxRange)
{
    if (maxRange==0.0) return;

    if ((P-Q).mod2()<0.16) return;

    if (mNumSamples==0)
    {
        yDebug("0 samples");
            
        Q=P; W=U;
        mTarget=P+maxRange*U;
        mNumSamples=1;
        return;
    }

    if (mNumSamples==1)
    {
        yDebug("1 sample");
        
        double k=W*U;
        double d=1.0-k*k;

        if (d==0.0)
        {
            Q=P; W=U;
            mTarget=P+maxRange*U;
            return;
        }

        d=1.0/d;
        double t=((Q-P)*(U-k*W))*d;
        double s=((P-Q)*(W-k*U))*d;

        Q=P; W=U;
            
        if (t<0.5 || s<0.5 || t>15.0 || s>15.0)
        {
            mTarget=P+maxRange*U;
            return;
        }

        mTarget=P+t*U;
        mNumSamples=2;
        return;
    }

    if (mNumSamples==2)
    {
        yDebug("2 samples");
            
        double k=W*U;
        double d=1.0-k*k;

        if (d==0.0)
        {
            Q=P; W=U;
            mTarget=P+maxRange*U;
            mNumSamples=1;
            return;
        }

        d=1.0/d;
        double t=((Q-P)*(U-k*W))*d;
        double s=((P-Q)*(W-k*U))*d;

        Q=P; W=U;
            
        if (t<0.5 || s<0.5 || t>15.0 || s>15.0)
        {
            mTarget=P+maxRange*U;
            mNumSamples=1;
            return;
        }

        Vec2D newTarget=P+t*U;

        if ((newTarget-mTarget).mod2()>1.0)
        {
            mTarget=P+maxRange*U;
            mNumSamples=1;
            return;
        }

        mTarget=0.75*mTarget+0.25*newTarget;
    }
}

int TargetFilter::numSamples()
{
    return mNumSamples;
}

void TargetFilter::reset()
{
    mNumSamples=0;
}

Vec2D TargetFilter::getTarget()
{
    return mTarget;
}

