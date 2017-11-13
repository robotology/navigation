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
    void updateOdometry(Vec2D P,double H);

protected:
    inline int abs(int x){ return x>0?x:-x; }
    void setOmega(double omega);

    void setVel(const Vec2D& vel);

    void sendBrake();
    void sendCtrlRef();

    inline int XWorld2Grid(double& x){ return int(10.0*x)-mOx; }
    inline int YWorld2Grid(double& y){ return int(10.0*y)-mOy; }

    inline int XWorld2GridRound(double& x){ return int(10.0*x+0.5)-mOx; }
    inline int YWorld2GridRound(double& y){ return int(10.0*y+0.5)-mOy; }

    inline double XGrid2World(int x){ return 0.1*double(x+mOx); }
    inline double YGrid2World(int y){ return 0.1*double(y+mOy); }
    inline Vec2D Grid2World(int x,int y) { return Vec2D(0.1*double(x+mOx),0.1*double(y+mOy)); }

    void setVisionTarget(double heading);
    void setUserTarget(double heading,double distance);
    void replaceTarget(Vec2D& target);
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

