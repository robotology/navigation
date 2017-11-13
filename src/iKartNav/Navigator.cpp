#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

#include "Navigator.h"

#define PRIO_LASER  1
#define PRIO_VISION 2

void Navigator::updateOdometry(Vec2D P,double H)
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

void Navigator::setOmega(double omega)
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

void Navigator::setVisionTarget(double heading)
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

void Navigator::setUserTarget(double heading,double distance)
{
    mHaveTarget=true;
    mTarget=mOdoP+distance*Vec2D(mOdoH+heading);
    printf("NEW TARGET X=%.3f Y=%.3f\n",mTarget.x,mTarget.y);
    fflush(stdout);
    
    replaceTarget(mTarget);
}
    
void Navigator::sendCtrlRef()
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

void Navigator::sendBrake()
{
    mOmega=mOmegaRef=0.0;
    mVel=mVelRef=Vec2D::zero;
    mKartCtrl->setCtrlRef(0.0,0.0,0.0,NAVIGATOR_PRIORITY);
}

void Navigator::setVel(const Vec2D& vel)
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

void Navigator::replaceTarget(Vec2D& target)
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

Navigator::Navigator(yarp::os::ResourceFinder *rf)
    :
        RateThread(10),
        mRF(rf),
        mVel(0.0,0.0),
        mOmega(0.0),
        mVelRef(0.0,0.0),
        mOmegaRef(0.0)
{
    mOx=mOy=0;

    int SIZE=2*DIM+1;

    mD=new double*[SIZE];               mD[0]=new double[SIZE*SIZE];
    mQueued=new bool*[SIZE];            mQueued[0]=new bool[SIZE*SIZE];
    mZeta=new double*[SIZE];            mZeta[0]=new double[SIZE*SIZE];
    mSamples=new double*[SIZE];         mSamples[0]=new double[SIZE*SIZE];
    mReach=new unsigned short*[SIZE];   mReach[0]=new unsigned short[SIZE*SIZE];
    mPrio=new unsigned char*[SIZE];     mPrio[0]=new unsigned char[SIZE*SIZE];
                
    for (int i=1; i<SIZE; ++i)
    {
        mD[i]=mD[i-1]+SIZE;
        mZeta[i]=mZeta[i-1]+SIZE;
        mSamples[i]=mSamples[i-1]+SIZE;
        mReach[i]=mReach[i-1]+SIZE;
        mQueued[i]=mQueued[i-1]+SIZE;
        mPrio[i]=mPrio[i-1]+SIZE;
    }
    
    for (int i=0; i<SIZE; ++i)
    {
        mD[i]+=DIM;
        mZeta[i]+=DIM;
        mSamples[i]+=DIM;
        mReach[i]+=DIM;
        mQueued[i]+=DIM;
        mPrio[i]+=DIM;
    }

    mD+=DIM;
    mZeta+=DIM;
    mSamples+=DIM;
    mReach+=DIM;
    mQueued+=DIM;
    mPrio+=DIM;

    for (int x=-DIM; x<=DIM; ++x)
    {
        for (int y=-DIM; y<=DIM; ++y)
        {
            mPrio[x][y]=0;
            mZeta[x][y]=0.0;
            mSamples[x][y]=0.0;
        }
    }
    
    mQueueX=new int[SIZE*SIZE];
    mQueueY=new int[SIZE*SIZE];

    ///////////////////////////////////

    mMask=new double*[21];
    mMask[0]=new double[21*21];
    for (int i=1; i<21; ++i)
    {
        mMask[i]=mMask[i-1]+21;
    }
    for (int i=0; i<21; ++i)
    {
        mMask[i]+=10;
    }
    mMask+=10;

    const double sigma=0.5;
    mK=-1.0/(2.0*sigma*sigma);
    for (int x=-10; x<=10; ++x)
    {
        for (int y=-10; y<=10; ++y)
        {
            mMask[x][y]=exp(mK*0.01*double(x*x+y*y));
        }
    }

    mTargetH=0.0;
    mHaveTarget=false;
    mHaveTargetH=false;
    mPaused=true;
}

bool Navigator::threadInit()
{
    mHaveTarget=false;

    mRadius=mRF->check("radius",yarp::os::Value(0.3575)).asDouble();
    mRFpos.x=mRF->check("laser_pos",yarp::os::Value(0.245)).asDouble();
    mRFpos.y=0.0;
    mMinSpeed=mRF->check("min_speed",yarp::os::Value(0.05)).asDouble();
    mMaxSpeed=mRF->check("max_speed",yarp::os::Value(0.2)).asDouble();
    mMaxOmega=mRF->check("max_ang_speed",yarp::os::Value(12.0)).asDouble();
    mLinAcc=mRF->check("linear_acc",yarp::os::Value(0.25)).asDouble();
    mRotAcc=mRF->check("rot_acc",yarp::os::Value(12.0)).asDouble();
    RAYS=mRF->check("num_range_samples",yarp::os::Value(1081)).asInt();
    mRangeMax=mRF->check("range_max_dist",yarp::os::Value(9.5)).asDouble();
    mAngularRes=mRF->check("range_ang_res",yarp::os::Value(0.25)).asDouble();

    RAYS_BY_2=RAYS/2;

    mRays=new Vec2D[RAYS]; 
    mRanges=new double[RAYS];
    mAngleMax=double(RAYS_BY_2)*mAngularRes;

    for (int i=0; i<RAYS; ++i)
    {
        mRays[i]=Vec2D(double(i-RAYS_BY_2)*mAngularRes);
        mRanges[i]=0.0;
    }

    ///////////////////////////////////

    std::string local=mRF->check("local",yarp::os::Value("/ikartnav")).asString().c_str();
        
    mLaserPortI.open((local+"/laser:i").c_str());
    mUserPortI.open((local+"/user:i").c_str());
    mVisionPortI.open((local+"/vision:i").c_str());
    mEventPortI.open((local+"/event:i").c_str());
    mOdometryPortI.open((local+"/odometry:i").c_str());
    
    mResetOdometryPortO.open((local+"/resetodometry:o").c_str());
    mStatusPortO.open((local+"/status:o").c_str());
    mTargetPortO.open((local+"/target:o").c_str());
    
    mKartCtrl=new CtrlThread(local);

    mKartCtrl->start();

    return true;
}

void Navigator::onStop()
{
    mKartCtrl->stop();
}

void Navigator::threadRelease()
{
    delete mKartCtrl;
    mKartCtrl=NULL;
 
    mOdometryPortI.interrupt();
    mUserPortI.interrupt();
    mVisionPortI.interrupt();
    mEventPortI.interrupt();
    mLaserPortI.interrupt();
    mResetOdometryPortO.interrupt();
    mStatusPortO.interrupt();
    mTargetPortO.interrupt();
        
    mOdometryPortI.close();
    mUserPortI.close();
    mVisionPortI.close();
    mEventPortI.close();
    mLaserPortI.close();
    mResetOdometryPortO.close();
    mStatusPortO.close();
    mTargetPortO.close();
    
    return;

    if (mRays) delete [] mRays;
    if (mRanges) delete [] mRanges;

    mD+=DIM;       mD[0]+=DIM;
    mZeta+=DIM;    mZeta[0]+=DIM;
    mSamples+=DIM; mSamples[0]+=DIM;
    mReach+=DIM;   mReach[0]+=DIM;
    mQueued+=DIM;  mQueued[0]+=DIM;
    mPrio+=DIM;    mPrio[0]+=DIM; 
    mMask+=10;     mMask[0]+=10;

    delete [] mD[0];       delete [] mD;
    delete [] mZeta[0];    delete [] mZeta;
    delete [] mSamples[0]; delete [] mSamples;
    delete [] mReach[0];   delete [] mReach;
    delete [] mQueued[0];  delete [] mQueued;
    delete [] mMask[0];    delete [] mMask;
    delete [] mPrio[0];    delete [] mPrio;
    delete [] mQueueX;     delete [] mQueueY;    
}

void Navigator::shiftMapSouth()
{
    for (int x=-DIM; x<=DIM_BY_2; ++x)
        for (int y=-DIM; y<=DIM; ++y)
            mSamples[x][y]=mSamples[x+DIM_BY_2][y];

    for (int x=DIM_BY_2+1; x<=DIM; ++x)
        for (int y=-DIM; y<=DIM; ++y)
            mSamples[x][y]=0.0;

    mOx+=DIM_BY_2;
}

void Navigator::shiftMapNorth()
{
    for (int x=DIM; x>=-DIM_BY_2; --x)
        for (int y=-DIM; y<=DIM; ++y)
            mSamples[x][y]=mSamples[x-DIM_BY_2][y];

    for (int x=-DIM; x<=-DIM_BY_2-1; ++x)
        for (int y=-DIM; y<=DIM; ++y)
            mSamples[x][y]=0.0;

    mOx-=DIM_BY_2;
}

void Navigator::shiftMapEast()
{
    for (int x=-DIM; x<=DIM; ++x)
        for (int y=-DIM; y<=DIM_BY_2; ++y)
            mSamples[x][y]=mSamples[x][y+DIM_BY_2];

    for (int x=-DIM; x<=DIM; ++x)
        for (int y=DIM_BY_2+1; y<=DIM; ++y)
            mSamples[x][y]=0.0;

    mOy+=DIM_BY_2;
}

void Navigator::shiftMapWest()
{
    for (int x=-DIM; x<=DIM; ++x)
        for (int y=DIM; y>=-DIM_BY_2; --y)
            mSamples[x][y]=mSamples[x][y-DIM_BY_2];

    for (int x=-DIM; x<=DIM; ++x)
        for (int y=-DIM; y<=-DIM_BY_2-1; ++y)
            mSamples[x][y]=0.0;

    mOy-=DIM_BY_2;
}

void Navigator::updateMap(yarp::sig::Vector& rangeData)
{
    double dMax2=0.0;
    for (int i=0; i<RAYS; ++i)
    {
        mRanges[i]=rangeData[i];

        if (rangeData[i]>dMax2) dMax2=rangeData[i];
    }
    dMax2*=dMax2;

    int angle;
    Vec2D V;
    double d2;
    Vec2D Orf=mOdoP+mRFpos.rot(mOdoH);

    for (int x=-DIM; x<=DIM; ++x)
    {
        for (int y=-DIM; y<=DIM; ++y)
        {
            if (mSamples[x][y]>0.1)
            {
                if (mPrio[x][y]<=PRIO_LASER)
                {
                    V.x=XGrid2World(x)-Orf.x;
                    V.y=YGrid2World(y)-Orf.y;

                    d2=V.mod2();

                    if (d2<dMax2)
                    {
                        angle=RAYS_BY_2+int(0.5+4.0*mod180(V.arg()-mOdoH));

                        if (angle>=10 && angle<RAYS-10)
                        {
                            if (d2<rangeData[angle-1]*rangeData[angle-1] &&
                                d2<rangeData[angle  ]*rangeData[angle  ] &&
                                d2<rangeData[angle+1]*rangeData[angle+1])
                            {
                                mSamples[x][y]=0.0;
                            }
                        }
                    }
                }

                if (mPrio[x][y]==PRIO_LASER)
                { 
                    mSamples[x][y]*=0.9995;
                }
                else if (mPrio[x][y]==PRIO_VISION)
                {
                    mSamples[x][y]*=0.9992;
                }
            }
            else
            {
                mSamples[x][y]=0.0;
                mPrio[x][y]=0;
            }
        }
    }

    int xl,yd,xr,yu;
    //double px,py;
    double dXl,dXr,dYd,dYu;
    double dExl,dExr,dEyd,dEyu;
    double dZld,dZrd,dZru,dZlu;

    Vec2D Prf;

    for (int i=0; i<RAYS; ++i)
    {
        if (rangeData[i]>mRangeMax) continue;

        Prf=Orf+rangeData[i]*mRays[i].rot(mOdoH);

        xl=XWorld2Grid(Prf.x); xr=xl+1;

        if (xl<-DIM || xr>DIM) continue;

        yd=YWorld2Grid(Prf.y); yu=yd+1;

        if (yd<-DIM || yu>DIM) continue;

        dXl=Prf.x-XGrid2World(xl); dXr=0.1-dXl;
        dYd=Prf.y-YGrid2World(yd); dYu=0.1-dYd;

        dExl=exp(mK*dXl*dXl);
        dExr=exp(mK*dXr*dXr);
        dEyd=exp(mK*dYd*dYd);
        dEyu=exp(mK*dYu*dYu);

        dZld=dExl*dEyd;
        dZrd=dExr*dEyd;
        dZru=dExr*dEyu;
        dZlu=dExl*dEyu;

        if (mSamples[xl][yd]<dZld) mSamples[xl][yd]=dZld; 
        if (mSamples[xr][yd]<dZrd) mSamples[xr][yd]=dZrd;
        if (mSamples[xr][yu]<dZru) mSamples[xr][yu]=dZru;
        if (mSamples[xl][yu]<dZlu) mSamples[xl][yu]=dZlu;
        
        if (mPrio[xl][yd]<PRIO_LASER) mPrio[xl][yd]=PRIO_LASER;
        if (mPrio[xr][yd]<PRIO_LASER) mPrio[xr][yd]=PRIO_LASER;
        if (mPrio[xr][yu]<PRIO_LASER) mPrio[xr][yu]=PRIO_LASER;
        if (mPrio[xl][yu]<PRIO_LASER) mPrio[xl][yu]=PRIO_LASER; 
    }
}

void Navigator::updateZeta()
{
    for (int x=-DIM; x<=DIM; ++x)
    {
        for (int y=-DIM; y<=DIM; ++y)
        {
            mD[x][y]=1E+10;
            mZeta[x][y]=0.0;
            mQueued[x][y]=false;
        }
    }

    for (int xc=-DIM; xc<=DIM; ++xc)
    {
        for (int yc=-DIM; yc<=DIM; ++yc)
        {
            if (mSamples[xc][yc]>0.1)
            {           
                for (int dx=-10; dx<=10; ++dx)
                {
                    int x=xc+dx;

                    if (x>=-DIM && x<=DIM)
                    {
                        for (int dy=-10; dy<=10; ++dy)
                        {
                            int y=yc+dy;
                            
                            if (y>=-DIM && y<=DIM)
                            {
                                double dZ=mMask[dx][dy]*mSamples[xc][yc];

                                if (mZeta[x][y]<dZ) mZeta[x][y]=dZ;
                            }
                        }
                    }
                }
            }
        }
    }

    int head=0,tail=0;

    for (int x=-DIM; x<=DIM; ++x)
    {
        for (int y=-DIM; y<=DIM; ++y)
        {
            if (mZeta[x][y]<=THR)
            {
                mQueueX[tail  ]=x;
                mQueueY[tail++]=y;
                mQueued[x][y]=true;
                mReach[x][y]=0;
            }
            else
            {
                mQueued[x][y]=false;
                mReach[x][y]=0xFFFF;
            }
        }
    }

    static const int MODULE=(2*DIM+1)*(2*DIM+1);

    while(head!=tail)
    {
        head%=MODULE;

        int xc=mQueueX[head  ];
        int yc=mQueueY[head++];
        
        mQueued[xc][yc]=false;

        int xa=xc-1; if (xa<-DIM) xa=-DIM;
        int xb=xc+1; if (xb> DIM) xb= DIM;
        int ya=yc-1; if (ya<-DIM) ya=-DIM; 
        int yb=yc+1; if (yb> DIM) yb= DIM;  

        for (int x=xa; x<=xb; ++x)
        {
            for (int y=ya; y<=yb; ++y)
            {
                if (x!=xc || y!=yc)
                {
                    unsigned short k=(x==xc||y==yc)?5:7;

                    if (mReach[x][y] && mReach[x][y]>mReach[xc][yc]+k)
                    {
                        mReach[x][y]=mReach[xc][yc]+k;
                        
                        tail%=MODULE;
                        mQueueX[tail  ]=x;
                        mQueueY[tail++]=y;
                        mQueued[x][y]=true;
                    }
                }
            }
        }
    }
}

void Navigator::updateGNF()
{
    replaceTarget(mTarget);

    int xT=XWorld2Grid(mTarget.x);
    int yT=YWorld2Grid(mTarget.y);
    
    if (xT<-DIM) xT=-DIM; else if (xT>DIM-1) xT=DIM-1;
    if (yT<-DIM) yT=-DIM; else if (yT>DIM-1) yT=DIM-1;

    double dXl=mTarget.x-XGrid2World(xT),dXr=1.0-dXl;
    double dYd=mTarget.y-YGrid2World(yT),dYu=1.0-dYd;

    int head=0,tail=0;

    mD[xT]    [yT]=sqrt(dXl*dXl+dYd*dYd);
    mD[xT+1]  [yT]=sqrt(dXr*dXr+dYd*dYd);
    mD[xT]  [yT+1]=sqrt(dXl*dXl+dYu*dYu);
    mD[xT+1][yT+1]=sqrt(dXr*dXr+dYu*dYu);
    
    mQueueX[tail  ]=xT; 
    mQueueY[tail++]=yT;
    mQueued[xT  ][yT  ]=true;

    mQueueX[tail  ]=xT+1; 
    mQueueY[tail++]=yT;
    mQueued[xT+1][yT  ]=true;

    mQueueX[tail  ]=xT; 
    mQueueY[tail++]=yT+1;
    mQueued[xT  ][yT+1]=true;

    mQueueX[tail  ]=xT+1; 
    mQueueY[tail++]=yT+1;
    mQueued[xT+1][yT+1]=true;

    static const int MODULE=(2*DIM+1)*(2*DIM+1);

    while(head!=tail)
    {
        head%=MODULE;

        int xc=mQueueX[head  ];
        int yc=mQueueY[head++];
        
        mQueued[xc][yc]=false;
      
        double D=0.1*(1.0+SAFETY*mZeta[xc][yc]);
        double dNewDL=      D+mD[xc][yc];
        double dNewDT=1.414*D+mD[xc][yc];

        int xa=xc-1; if (xa<-DIM) xa=-DIM;
        int xb=xc+1; if (xb> DIM) xb= DIM;
        int ya=yc-1; if (ya<-DIM) ya=-DIM; 
        int yb=yc+1; if (yb> DIM) yb= DIM;  

        for (int x=xa; x<=xb; ++x)
        {
            for (int y=ya; y<=yb; ++y)
            {
                if (x!=xc || y!=yc)
                {
                    double dNewD=(x==xc||y==yc)?dNewDL:dNewDT;
       
                    if (mD[x][y]>dNewD)
                    {
                        mD[x][y]=dNewD;

                        if (!mQueued[x][y] && !mReach[x][y])
                        {
                            tail%=MODULE;
                            mQueueX[tail  ]=x;
                            mQueueY[tail++]=y;
                            mQueued[x][y]=true;
                        }
                    }
                }
            }
        }
    }

    head=tail=0;

    for (int x=-DIM; x<=DIM; ++x)
    {
        for (int y=-DIM; y<=DIM; ++y)
        {
            if (mReach[x][y] && mD[x][y]<1E5)
            {
                mQueueX[tail  ]=x;
                mQueueY[tail++]=y;
                mQueued[x][y]=true;
            }
        }
    }

    while(head!=tail)
    {
        head%=MODULE;

        int xc=mQueueX[head  ];
        int yc=mQueueY[head++];
        
        mQueued[xc][yc]=false;

        double D=0.1*(1.0+SAFETY*mZeta[xc][yc]);
        double dNewDL=      D+mD[xc][yc];
        double dNewDT=1.414*D+mD[xc][yc];

        int xa=xc-1; if (xa<-DIM) xa=-DIM;
        int xb=xc+1; if (xb> DIM) xb= DIM;
        int ya=yc-1; if (ya<-DIM) ya=-DIM; 
        int yb=yc+1; if (yb> DIM) yb= DIM;  

        for (int x=xa; x<=xb; ++x)
        {
            for (int y=ya; y<=yb; ++y)
            {
                if (x!=xc || y!=yc)
                {
                    double dNewD=(x==xc||y==yc)?dNewDL:dNewDT;

                    if (mReach[x][y] && mReach[x][y]>=mReach[xc][yc] && mD[x][y]>dNewD)
                    {
                        mD[x][y]=dNewD;

                        tail%=MODULE;
                        mQueueX[tail  ]=x;
                        mQueueY[tail++]=y;
                        mQueued[x][y]=true;
                    }
                }
            }
        }
    }
}

int Navigator::followGNF(Vec2D P,Vec2D &direction,double &curvature,double &zeta,Vec2D &gradient)
{
    int xr=XWorld2GridRound(P.x);
    int yr=YWorld2GridRound(P.y);

    if (xr<=-DIM || xr>=DIM || yr<=-DIM || yr>=DIM) return GNF_OUT_OF_GRID;

    double dX=10.0*(P.x-XGrid2World(xr)); // -0.5 < dX < 0.5
    double dY=10.0*(P.y-YGrid2World(yr)); // -0.5 < dY < 0.5

    double dXa=0.5-dX,dXb=0.5+dX,dYa=0.5-dY,dYb=0.5+dY;

    double dXaYa=dXa*dYa,dXaYb=dXa*dYb,dXbYa=dXb*dYa,dXbYb=dXb*dYb;

    double D00=mD[xr-1][yr-1]*dXaYa+mD[xr-1][yr  ]*dXaYb
              +mD[xr  ][yr-1]*dXbYa+mD[xr  ][yr  ]*dXbYb;

    double D10=mD[xr  ][yr-1]*dXaYa+mD[xr  ][yr  ]*dXaYb
              +mD[xr+1][yr-1]*dXbYa+mD[xr+1][yr  ]*dXbYb;

    double D01=mD[xr-1][yr  ]*dXaYa+mD[xr-1][yr+1]*dXaYb
              +mD[xr  ][yr  ]*dXbYa+mD[xr  ][yr+1]*dXbYb;

    double D11=mD[xr  ][yr  ]*dXaYa+mD[xr  ][yr+1]*dXaYb
              +mD[xr+1][yr  ]*dXbYa+mD[xr+1][yr+1]*dXbYb;


    direction.x=D00+D01-D10-D11;
    direction.y=D00-D01+D10-D11;

    double m=direction.normalize();
    
    curvature= m>0.0 ? 200.0*Vec2D::RAD2DEG*direction.x*(D00-D01-D10+D11)*direction.y : 0.0;

    int x=XWorld2Grid(P.x);
    int y=YWorld2Grid(P.y);

    dX=10.0*(P.x-XGrid2World(x)); // 0.0<=dX<1.0
    dY=10.0*(P.y-YGrid2World(y)); // 0.0<=dY<1.0

    zeta=(1.0-dX)*(1.0-dY)*mZeta[x  ][y  ]
        +(1.0-dX)*     dY *mZeta[x  ][y+1]
        +     dX *(1.0-dY)*mZeta[x+1][y  ]
        +     dX *     dY *mZeta[x+1][y+1];

    gradient.x=(1.0-dY)*mZeta[x  ][y  ]
              +     dY *mZeta[x  ][y+1]
              -(1.0-dY)*mZeta[x+1][y  ]
              -     dY *mZeta[x+1][y+1];

    gradient.y=(1.0-dX)*mZeta[x  ][y  ]
              -(1.0-dX)*mZeta[x  ][y+1]
              +     dX *mZeta[x+1][y  ]
              -     dX *mZeta[x+1][y+1];

    gradient.normalize();

    return mReach[xr][yr]?GNF_TARGET_UNREACHABLE:GNF_OK;
}

void Navigator::addEvent(double heading,double distance,double radius)
{
    Vec2D event=mOdoP+distance*Vec2D(mOdoH+heading);

    int x0=(int)(0.5+10.0*event.x);
    int y0=(int)(0.5+10.0*event.y);
    int r=1+(int)(10.0*radius);
    int r2=r*r;

    for (int x=x0-r; x<=x0+r; ++x) if (x>=-DIM && x<=DIM)
    {
        for (int y=y0-r; y<=y0+r; ++y) if (y>=-DIM && y<=DIM)
        {
            if ((x-x0)*(x-x0)+(y-y0)*(y-y0)<=r2)
            {
                mSamples[x][y]=1.0;
                
                if (mPrio[x][y]<PRIO_VISION) mPrio[x][y]=PRIO_VISION;
            }
        }
    }
}

void Navigator::run()
{
    //////////////////////////
    // GET COMMANDS
    for (yarp::os::Bottle* bot; bot=mUserPortI.read(false);)
    {
        yarp::os::ConstString cmd=bot->get(0).asString();

        if (cmd=="target" || cmd=="t")
        {   
            if (bot->size()>=4)
            {
                mHaveTargetH=true;
                mTargetH=mOdoH-bot->get(3).asDouble();                
            }
            else
            {
                mHaveTargetH=false;
            }

            setUserTarget(-bot->get(1).asDouble(),bot->get(2).asDouble());

            if (mHaveTargetH)
            {
                printf("TARGET HEADING=%.1f\n",mTargetH);
                fflush(stdout);
            }
        }
        else if (cmd=="gotoRel")
        {   
            if (bot->size()>=4)
            {
                mTargetH=mOdoH-bot->get(3).asDouble();
                printf("NEW TARGET X=%.3f Y=%.3f H=%.1f\n",mTarget.x,mTarget.y,mTargetH);
                fflush(stdout);
                mHaveTargetH=true;
                mPaused=false;
            }
            else
            {
                printf("NEW TARGET X=%.3f Y=%.3f\n",mTarget.x,mTarget.y);
                fflush(stdout);
                mHaveTargetH=false;
                mPaused=false;
            }

            setUserTarget(-bot->get(1).asDouble(),bot->get(2).asDouble());
        }
        else if (cmd=="gotoAbs")
        {   
            printf("Not yet implemented\n");
            fflush(stdout);
        }
        else if (cmd=="event" || cmd=="e")
        {
            addEvent(-bot->get(1).asDouble(),bot->get(2).asDouble(),bot->get(3).asDouble());
        }
        else if (cmd=="stop" || cmd=="s")
        {
            printf("STOP\n");
            fflush(stdout);
            mHaveTarget=false;
        }
        else if (cmd=="pause" || cmd=="p")
        {
            printf("PAUSE\n");
            fflush(stdout);
            mPaused=true;
        }
        else if (cmd=="go" || cmd=="g" || cmd=="resume")
        {
            printf("GO\n");
            fflush(stdout);
            mPaused=false;
        }
        else if (cmd=="help")
        {
            printf("available commands:\n");
            printf("- go\n");
            printf("- pause\n");
            printf("- resume\n");
            printf("- stop\n");
            printf("- event <1> <2> <3> \n");
            printf("- target <x> <y> <h> \n");
            printf("- gotoAbs <x> <y> <h> \n");
            printf("- gotoRel <x> <y> <h> \n");
        }
    } 
    
    for (yarp::os::Bottle* bot; bot=mVisionPortI.read(false);)
    {
        mHaveTargetH=false;
        setVisionTarget(-bot->get(0).asDouble());        
    }

    for (yarp::os::Bottle* bot; bot=mEventPortI.read(false);)
    {
        addEvent(-bot->get(0).asDouble(),bot->get(1).asDouble(),bot->get(2).asDouble());
    }
    
    // GET COMMANDS
    //////////////////////////

    //////////////////////////
    // GET ODOMETRY
    yarp::os::Bottle *odometryLast=NULL;

    static int odometryResetCycle=0;

    if (!odometryResetCycle)
    {
        if (mResetOdometryPortO.getOutputCount()>0)
        {
            yarp::os::Bottle msg,rsp;
            msg.addString("reset_odometry");
            mResetOdometryPortO.write(msg,rsp);
            if (rsp.get(0).asString()=="Odometry reset done.")
            {
                mResetOdometryPortO.write(msg,rsp);
                if (rsp.get(0).asString()=="Odometry reset done.")
                {
                    odometryResetCycle=1;
                }
            }
        }

        return;
    }
    
    if (odometryResetCycle && odometryResetCycle<200)
    {
        ++odometryResetCycle;
        
        return;
    }
    
    for (yarp::os::Bottle *odometry; odometry=mOdometryPortI.read(false);)
    {
        odometryLast=odometry;
    }

    if (odometryLast)
    {
        mOdoP.x= odometryLast->get(1).asDouble();
        mOdoP.y=-odometryLast->get(0).asDouble();
        mOdoH=mod180(-odometryLast->get(2).asDouble());
    }
    // GET ODOMETRY
    //////////////////////////

    double timeOdoNew=yarp::os::Time::now();
    static double timeOdoOld=0.0;
    if (timeOdoNew-timeOdoOld>10.0)
    {
        timeOdoOld=timeOdoNew;
        printf("X=%.3f Y=%.3f H=%.1f\n",mOdoP.x,mOdoP.y,mOdoH);
    }

    //////////////////////////
    // GET LASER
    yarp::sig::Vector *rangeDataLast=NULL;
    for (yarp::sig::Vector *rangeData; rangeData=mLaserPortI.read(false);)
    {    
        rangeDataLast=rangeData;
    }
    if (rangeDataLast) updateMap(*rangeDataLast);
    // GET LASER
    //////////////////////////
    
    //////////////////////////
    // COMPUTE GNF
    static int cycle=9;
    if (++cycle==10)
    {
        updateZeta();
        if (mHaveTarget) updateGNF();
        cycle=0;
    }
    // COMPUTE GNF
    //////////////////////////

    if (!mHaveTarget)
    {
        if (!mPaused && mTargetFilter.numSamples()==1)
        {
            setOmega(0.0);
            setVel(Vec2D(0.0,0.1));
            sendCtrlRef();

            return;   
        }
     
        sendBrake();

        return;
    }

    //////////////////////////
    // FOLLOW GNF
    double curvature,zeta;
    Vec2D direction,gradient;
    
    Vec2D  deltaP=mTarget-mOdoP;
    double distance=deltaP.mod();

    // have target
       
    int reachable=followGNF(mOdoP,direction,curvature,zeta,gradient);
    
    if (distance<0.05)
    {
        setVel(Vec2D::zero);
       
        if (mHaveTargetH)
        {
            double deltaH=0.5*mod180(mTargetH-mOdoH);
            if (deltaH<-mMaxOmega) deltaH=-mMaxOmega;
            if (deltaH> mMaxOmega) deltaH= mMaxOmega;
            setOmega(deltaH);

            if (fabs(deltaH)<1.5)
            {
                mHaveTargetH=false;
            }
        }
        else
        {
            setOmega(0.0);
            printf("Target reached.\n");
            mHaveTarget=false;            
        }
    }
    else if (distance<0.2)
    {
        setVel((1.0-zeta)*0.5*distance*deltaP.norm().rot(-mOdoH));
        setOmega(0.0);
    }
    else
    {
        if (reachable==GNF_TARGET_UNREACHABLE)
        {
            static int t=0;
            if (++t==100)
            {
                printf("TARGET UNREACHABLE\n");
                t=0;
            }
        }
        
        if (zeta>THR)
        {
            if (direction*gradient<0.0)
            {
                direction-=1.05*(direction*gradient)*gradient;
            }
        }
        
        double deltaH=mod180(direction.arg()-mOdoH);

        //////////////////////////////////////////////////
        double deltaT=mod180((mTarget-mOdoP).arg()-mOdoH);
        double errH=mod180(deltaT-deltaH);

        if (fabs(errH)<90.0)
        {
            setVel((zeta*mMinSpeed+(1.0-zeta)*mMaxSpeed)*Vec2D(deltaH));

            double omega=0.2*deltaT;
            if (omega<-mMaxOmega) omega=-mMaxOmega;
            if (omega> mMaxOmega) omega= mMaxOmega;
            setOmega(omega);
        }
        else if (fabs(errH)<135.0)
        {
            deltaT+=errH>0.0?(90.0-errH):(-90.0-errH);
            
            double omega=0.2*deltaT;
            if (omega<-mMaxOmega) omega=-mMaxOmega;
            if (omega> mMaxOmega) omega= mMaxOmega;
            setOmega(omega);
        }
        else if (fabs(deltaH)<=90.0)
        {
            setVel((zeta*mMinSpeed+(1.0-zeta)*mMaxSpeed)*Vec2D(deltaH));

            setOmega((mMaxOmega/90.0)*deltaH/*+mVel.mod()*curvature*/);
        }
        else
        {
            setVel(Vec2D::zero);
            setOmega(deltaH>0.0?mMaxOmega:-mMaxOmega);
        }
        //////////////////////////////////////////////////

        /*
        if (fabs(deltaH)<=45.0)
        {
            setVel((zeta*mMinSpeed+(1.0-zeta)*mMaxSpeed)*Vec2D(deltaH));

            // must be from odometry
            setOmega((mMaxOmega/45.0)*deltaH+Vec2D::RAD2DEG*mVel.mod()*curvature);
        }
        else
        {
            setVel(Vec2D::zero);
            setOmega(deltaH>0.0?mMaxOmega:-mMaxOmega);
        }
        */
    }

    // SEND COMMANDS
    if (mPaused)
    {
        sendBrake();
        return;
    }
    
    sendCtrlRef(); 
}

