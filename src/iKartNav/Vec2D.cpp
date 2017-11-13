#include "Vec2D.h"

const Vec2D Vec2D::zero(0.0,0.0);
const double Vec2D::DEG2RAD=M_PI/180.0;
const double Vec2D::RAD2DEG=180.0/M_PI;

double Vec2D::mod2()const
{
    return x*x+y*y;
}

double Vec2D::mod()const
{
    return sqrt(x*x+y*y);
}

Vec2D Vec2D::rot(double alfa)
{
    alfa*=DEG2RAD;
    double cs=cos(alfa);
    double sn=sin(alfa);
    return Vec2D(cs*x-sn*y,sn*x+cs*y);
}

Vec2D Vec2D::rotLeft()const
{
    return Vec2D(-y,x);
}

Vec2D Vec2D::rotRight()const
{
    return Vec2D(y,-x);
}

Vec2D Vec2D::norm(double dl) const
{ 
    double dm=mod();
        
    if (dm!=0.0)
        return *this*(dl/dm);
    else
        return Vec2D(0.0,0.0);
}

double Vec2D::normalize(double dl)
{ 
    double dm=mod();
        
    if (dm!=0.0)
    {
        x*=dl/=dm;
        y*=dl;
    }
    else
        x=y=0.0;

    return dm;
}

double Vec2D::arg() const
{
    return RAD2DEG*atan2(y,x);
}