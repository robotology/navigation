#ifndef __IKARTNAV_NAVIGATOR_H__
#define __IKARTNAV_NAVIGATOR_H__

#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "Vec2D.h"
#include "CtrlThread.h"
#include "TargetFilter.h"

#define DIM      128
#define DIM_BY_2  64

#define THR       0.666   // 0.666
#define SAFETY    1.5     // 1.25

//#define THR       0.7  // 0.666
//#define SAFETY    1.5  // 1.25

#define NAVIGATOR_PRIORITY    1
#define TRIANGULATOR_PRIORITY 2

class Navigator : public yarp::os::RateThread
{
public:
    Navigator(yarp::os::ResourceFinder *rf);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

    void updateOdometry(Vec2D P,double H)
    {
        mOdoP=P;
        mOdoH=H;

        int xc=XWorld2Grid(mOdoP.x);
        int yc=YWorld2Grid(mOdoP.y);

        if (xc> DIM_BY_2) shiftMapSouth();
        if (xc<-DIM_BY_2) shiftMapNorth();

        if (yc> DIM_BY_2) shiftMapEast();
        if (yc<-DIM_BY_2) shiftMapWest();
    }

protected:
    int abs(int x){ return x>0?x:-x; }

    void setOmega(double omega)
    {
        if (omega>mMaxOmega)
        {
            mOmegaRef=mMaxOmega;
        }
        else if (omega<-mMaxOmega)
        {
            mOmegaRef=-mMaxOmega;
        }
        else
        {
            mOmegaRef=omega;
        }
    }

    void setVel(const Vec2D& vel)
    {
        if (vel.mod()>mMaxSpeed)
        {
            mVelRef=vel.norm(mMaxSpeed);
        }
        else
        {
            mVelRef=vel;
        }
    }

    void sendBrake()
    {
        mOmega=mOmegaRef=0.0;
        mVel=mVelRef=Vec2D::zero;
        mKartCtrl->setCtrlRef(0.0,0.0,0.0,NAVIGATOR_PRIORITY);
    }

    void sendCtrlRef()
    {
        double timeNew=yarp::os::Time::now();
        static double timeOld=timeNew;  
        double step=timeNew-timeOld;
        timeOld=timeNew;

        if (mVelRef!=mVel)
        {
            Vec2D Verr=mVelRef-mVel;

            if (Verr.mod()>step*mLinAcc) mVel+=Verr.norm(step*mLinAcc); else mVel=mVelRef;
        }

        if (mOmegaRef!=mOmega)
        {
            double Werr=mOmegaRef-mOmega;

            if (Werr>step*mRotAcc) mOmega+=step*mRotAcc; else if (Werr<-step*mRotAcc) mOmega-=step*mRotAcc; else mOmega=mOmegaRef;
        }

        mKartCtrl->setCtrlRef(-mVel.arg(),mVel.mod(),-mOmega,NAVIGATOR_PRIORITY);
    }

    inline int XWorld2Grid(double& x){ return int(10.0*x)-mOx; }
    inline int YWorld2Grid(double& y){ return int(10.0*y)-mOy; }

    inline int XWorld2GridRound(double& x){ return int(10.0*x+0.5)-mOx; }
    inline int YWorld2GridRound(double& y){ return int(10.0*y+0.5)-mOy; }

    inline double XGrid2World(int x){ return 0.1*double(x+mOx); }
    inline double YGrid2World(int y){ return 0.1*double(y+mOy); }
    inline Vec2D Grid2World(int x,int y) { return Vec2D(0.1*double(x+mOx),0.1*double(y+mOy)); }

    void setVisionTarget(double heading)
    {
        int angle=RAYS_BY_2+int(0.5+4.0*mod180(heading));
        
        mTargetFilter.addPoint(mOdoP,Vec2D(heading+mOdoH),mRanges[angle]);

        if (mTargetFilter.numSamples()<2)
        {
            printf("Vision target not complete (%d samples)\n",mTargetFilter.numSamples());

            mHaveTarget=false;            
            return;
        }
        
        mTarget=mTargetFilter.getTarget();
        
        printf("Vision target X=%.3f Y=%.3f\n",mTarget.x,mTarget.y);
    
        replaceTarget(mTarget);
    }    

    void setUserTarget(double heading,double distance)
    {
        mHaveTarget=true;
        mTarget=mOdoP+distance*Vec2D(mOdoH+heading);
        printf("NEW TARGET X=%.3f Y=%.3f\n",mTarget.x,mTarget.y);
        fflush(stdout);
    
        replaceTarget(mTarget);
    }
    
    void replaceTarget(Vec2D& target) 
    {
        static double timeOld=0.0;

        int xr=XWorld2GridRound(target.x);
        int yr=YWorld2GridRound(target.y);

        if (xr<=-DIM || xr>=DIM || yr<=-DIM || yr>=DIM) return;

        Vec2D direction,gradient;
        double curvature,zeta;

        Vec2D X=target;
        Vec2D D=mOdoP-target;

        int n=(int)(100.0*D.mod());

        mHaveTarget=false;

        for (int i=0; i<n; ++i)
        {
            int result=followGNF(X,direction,curvature,zeta,gradient);

            if (result==GNF_OUT_OF_GRID)
            {
                printf("WARNING: the target is out of grid\n");
                mHaveTarget=true;
                
                if (mTargetPortO.getOutputCount()>0)
                {
                    double timeNew=yarp::os::Time::now();

                    if (timeNew-timeOld>1.0)
                    {
                        timeOld=timeNew;
                        D=target-mOdoP;
                        yarp::os::Bottle msg;
                        msg.addDouble(-D.arg());
                        msg.addDouble( D.mod());
                        mTargetPortO.write(msg);
                    }
                }

                return;
            }

            if (zeta<THR)
            {
                if (i)
                {
                    if (!mHaveTargetH)
                    {
                        mHaveTargetH=true;
                        mTargetH=(target-X).arg();
                    }

                    printf("Target replaced X=%.3f  Y=%.3f\n",X.x,X.y);
                    fflush(stdout);
                }

                mHaveTarget=true;
                target=X;

                if (mTargetPortO.getOutputCount()>0)
                {
                    double timeNew=yarp::os::Time::now();

                    if (timeNew-timeOld>1.0)
                    {
                        timeOld=timeNew;
                        D=target-mOdoP;
                        yarp::os::Bottle msg;
                        msg.addDouble(-D.arg());
                        msg.addDouble( D.mod());
                        mTargetPortO.write(msg);
                    }
                }
                
                return;
            }

            X+=(gradient*D>=0.0?0.01:-0.01)*gradient;
        }

        printf("Can't replace target in a free space position (Z=%f - Zmax=%f)\n",zeta,THR);
        fflush(stdout);
    }
    
    void addEvent(double heading,double distance,double radius);

    void updateMap(yarp::sig::Vector& rangeData);
    void updateZeta();
    void updateGNF();

    enum { GNF_OK=0, GNF_TARGET_UNREACHABLE=1, GNF_OUT_OF_GRID=2 };
    int followGNF(Vec2D P,Vec2D &direction,double& curvature,double &zeta,Vec2D &gradient);

    void shiftMapSouth();
    void shiftMapNorth();
    void shiftMapWest();
    void shiftMapEast();

    // config
    double mRadius;
    Vec2D mRFpos;
    double mMinSpeed;
    double mMaxSpeed;
    double mMaxOmega;
    double mLinAcc;
    double mRotAcc;    
    double mRangeMax;
    double mAngularRes;
    double mAngleMax;

    Vec2D  mOdoP;
    double mOdoH;

    int mOx,mOy;

    Vec2D mVel;
    double mOmega;

    Vec2D mVelRef;
    double mOmegaRef;

    Vec2D *mRays;
    double *mRanges;
    int RAYS;
    int RAYS_BY_2;

    CtrlThread* mKartCtrl;

    bool mHaveTarget;
    Vec2D mTarget;
    double mTargetH;
    bool mHaveTargetH;
    bool mPaused;

    double **mD;
    unsigned short **mReach;
    bool **mQueued;
    double **mZeta;
    unsigned char **mPrio;

    double **mSamples;

    int *mQueueX;
    int *mQueueY;

    ///////////////////////
    double mK;
    double **mMask;
    ///////////////////////

    TargetFilter mTargetFilter;

    //bool mImAlive;

    double T[4],Tx[4],Txx[4],S[4],Sy[4],Syy[4];
    double Mb[4][4];

    yarp::os::ResourceFinder *mRF;
    
    yarp::os::Port mResetOdometryPortO;
    yarp::os::Port mStatusPortO;
    yarp::os::Port mTargetPortO;

    yarp::os::BufferedPort<yarp::sig::Vector> mLaserPortI;

    yarp::os::BufferedPort<yarp::os::Bottle> mOdometryPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mUserPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mVisionPortI;
    yarp::os::BufferedPort<yarp::os::Bottle> mEventPortI;
};

#endif

