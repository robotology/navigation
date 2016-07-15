#ifndef __IKARTNAV_TARGETFILTER_H__
#define __IKARTNAV_TARGETFILTER_H__

#include "Vec2D.h"

class TargetFilter
{
public:

    TargetFilter(void)
    {
        mNumSamples=0;
        Q=Vec2D(100.0,100.0);
    }

    ~TargetFilter(void)
    {
    }

    void addPoint(Vec2D P,Vec2D U,double maxRange)
    {
        if (maxRange==0.0) return;

        if ((P-Q).mod2()<0.16) return;

        if (mNumSamples==0)
        {
            printf("0 samples\n");
            
            Q=P; W=U;
            mTarget=P+maxRange*U;
            mNumSamples=1;
            return;
        }

        if (mNumSamples==1)
        {
            printf("1 sample\n");
        
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
            printf("2 samples\n");
            
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

    int numSamples()
    {
        return mNumSamples;
    }

    void reset()
    {
        mNumSamples=0;
    }

    Vec2D getTarget()
    {
        return mTarget;
    }

protected:
    Vec2D Q,W;
    Vec2D mTarget;

    int mNumSamples;
};

#endif
