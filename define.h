#ifndef DEFINE_H
#define DEFINE_H

#ifndef M_PI
#define	M_PI				3.141592653589793
#endif

//#define	nint(x)				(int)((x>0)?(x+0.5):(x-0.5))
#define BOUND(x,min,max)	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
//#define	sqr(x)				((x)*(x))
#define deg2rad(x)          (x)/180.0*M_PI

struct point3d{
    double		x;
    double		y;
    double		z;
    point3d(){
        x=y=z=0;
    }
    point3d(double x_, double y_, double z_){
        x = x_; y= y_; z=z_;
    }

} ;

struct point4d{
    double x,y,z,info;
};

struct linefeat{
    point4d pt;
    point4d linehead;
    point4d linetail;
};

struct planefeat{
    point4d pt;
    point4d plane_pt1;
    point4d plane_pt2;
    point4d plane_pt3;


};

struct point3fi{
    float			x, y, z;
    unsigned char			i;
    point3fi(){
        x=y=z=0;
        i=0;
    }
    point3fi(float x_, float y_, float z_){
        x = x_; y= y_; z=z_;
        i=0;
    }
} ;

struct color3b
{
    unsigned char r,g,b;
};

struct rgbpoint3d
{
    double x,y,z;
    int r,g,b;
};

#endif // DEFINE_H

