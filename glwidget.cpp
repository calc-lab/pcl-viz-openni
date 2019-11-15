#include "glwidget.h"
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <qdebug.h>
#include <QPixmap>
#include <QtMath>

static inline double pt2PtDist(double x0,double y0,double z0,double x1,double y1,double z1){
    return qSqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)+(z0-z1)*(z0-z1));
}

static inline double normlize(point3fi p){
    return qSqrt(p.x*p.x+p.y*p.y+p.z*p.z);
}

static inline double ptMultiPt(point3fi p1, point3fi p2){
    return p1.x*p2.x+p1.y*p2.y+p1.z*p2.z;
}

static inline point3fi ptPlusPt(point3fi p1, point3fi p2){
    return point3fi(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
}

static inline point3fi ptMinusPt(point3fi p1, point3fi p2){
    return point3fi(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
}

double pt2LineDist(point3fi p, point3fi p1, point3fi p2)
{
    point3fi part1 = ptMinusPt(p, p1);
    point3fi part2 = ptMinusPt(p2,p1);
    double ratio = ptMultiPt(part1, part2)/(normlize(part2)*normlize(part2));

    point3fi part3(part2.x*ratio, part2.y*ratio, part2.z*ratio);
    return normlize(ptMinusPt(part1, part3));
}

void fitPlane(point3fi pt1, point3fi pt2, point3fi pt3, double &A, double &B, double &C, double &D)
{
    A = pt1.y*(pt2.z-pt3.z)+pt2.y*(pt3.z-pt1.z)+pt3.y*(pt1.z-pt2.z);
    B = pt1.z*(pt2.x-pt3.x)+pt2.z*(pt3.x-pt1.x)+pt3.z*(pt1.x-pt2.x);
    C = pt1.x*(pt2.y-pt3.y)+pt2.x*(pt3.y-pt1.y)+pt3.x*(pt1.y-pt2.y);
    D = -pt1.x*(pt2.y*pt3.z-pt3.y*pt2.z)-pt2.x*(pt3.y*pt1.z-pt1.y*pt3.z)-pt3.x*(pt1.y*pt2.z-pt2.y*pt1.z);
}

double pt2PlaneDist(point3fi pt, double A, double B, double C, double D)
{
    double n = normlize(point3fi(A,B,C))+1e-7;
    return fabs((A*pt.x+B*pt.y+C*pt.z+D)/n);
}

GLWidget::GLWidget(QWidget *parent) : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;
    zEye = 60;

    xEye = yEye = 0;
    setFocusPolicy(Qt::StrongFocus);
}

GLWidget::~GLWidget()
{
}

void GLWidget::initializeGL()
{
    //qglClearColor(Qt::black);
    glClearColor(0.0, 0.0, 0.0, 0);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_FLAT);
    glDisable(GL_LIGHTING);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    gluLookAt(xEye,yEye,zEye, //eye
              0,0,0, //center
              0,1,0 //up
              );

    //glTranslatef(0.0, 0.0, -5.0);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    glClearColor(1, 1, 1, 1.0);
    //glClearColor(0.87, 0.89, 0.90, 1.0);
    //glClearColor(0.78, 0.78, 0.78, 1.0);

    draw();

}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //fov, aspect, zNear, zFar
    gluPerspective(30, 1, 1, 400);

    gluLookAt(0,0,zEye, //eye
              0,0,0, //center
              0,1,0 //up
              );
    glMatrixMode(GL_MODELVIEW);
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}


void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRot(xRot + 8 * dy);
        setYRot(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRot(xRot + 8 * dy);
        setZRot(zRot + 8 * dx);
    }

    lastPos = event->pos();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    float rolldistance=-event->delta();
    //qDebug()<<rolldistance;
    zEye += rolldistance/50;
    zEye = std::min<int>(140,zEye);
    zEye = std::max<int>(zEye, 1);
    setScale(zEye);
}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_S)
    {
        //        QPixmap grab_img = this->grab();
        //        grab_img.save("1.png");
        //        qDebug()<<"save ok";
        yEye += 3;

        updateGL();

    }
    else if(event->key() == Qt::Key_W){
        yEye -= 3;

        updateGL();
    }
}
void GLWidget::setScale(int value)
{
    zEye = value;
    updateGL();
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360)
        angle -= 360 * 16;
}

void GLWidget::setXRot(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        //emit xRotationChanged(value);
        updateGL();
    }
}

void GLWidget::setYRot(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        //emit yRotationChanged(angle);
        updateGL();
    }
}

void GLWidget::setZRot(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        //emit zRotationChanged(angle);
        updateGL();
    }
}



void GLWidget::drawPointCloud()
{
    for (auto elem:drawingelements){

        if (elem.type == POINTS){
            glPointSize(elem.pointsize);
            glBegin(GL_POINTS);
            if (elem.colors.size() == elem.pts.size()){
                for (int i=0; i<elem.pts.size(); i++){
                    glColor3f(elem.colors[i].r/255.0, elem.colors[i].g/255.0, elem.colors[i].b/255.0);
                    glVertex3f(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z);

                }
            }
            glEnd();
        }
        else if(elem.type == LINES) {
            if (elem.pts.size()%3 != 0){
                return;
            }
            std::vector<double> ptdistbuf;

            //画直线
            glLineWidth(3);
            glBegin(GL_LINES);
            for (int i=0; i<(elem.pts.size()-2); i+=3){
                glColor3f(elem.colors[i].r/255.0, elem.colors[i].g/255.0, elem.colors[i].b/255.0);
                glVertex3f(elem.pts[i+1].x, elem.pts[i+1].y, elem.pts[i+1].z);
                glVertex3f(elem.pts[i+2].x, elem.pts[i+2].y, elem.pts[i+2].z);
                ptdistbuf.push_back(pt2LineDist(elem.pts[i], elem.pts[i+1], elem.pts[i+2]));
            }
            glEnd();

            //画特征点
            glPointSize(elem.pointsize);
            glBegin(GL_POINTS);
            for (int i=0; i<(elem.pts.size()-2); i+=3){
                glColor3f(elem.colors[i].r/255.0, elem.colors[i].g/255.0, elem.colors[i].b/255.0);
                //glColor3f(1.0, 0.0, 0.0);
                glVertex3f(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z);
            }
            glEnd();

            //显示距离
//            for (int i=0; i<(elem.pts.size()-2); i+=3){
//                if (ptdistbuf.size()==0)
//                    break;
//                qglColor(Qt::black);
//                renderText(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z, QString::number(ptdistbuf[i/3],'g', 3), QFont("Times New Roman", 10, QFont::Normal, false));
//            }
            //显示编号
//            for (int i=0; i<(elem.pts.size()-2); i+=3){
//                qglColor(Qt::black);
//                renderText(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z, QString("%1").arg(i/4), QFont("Times New Roman", 10, QFont::Normal, false));
//            }
        }
        else if (elem.type == PLANES) {
            if (elem.pts.size()%4 != 0){
                return;
            }

            std::vector<double> ptdistbuf;
            //画三角面
//            glBegin(GL_TRIANGLES);
//            for (int i=0; i<(elem.pts.size()-3); i+=4){
//                glColor3f(elem.colors[i].r/255.0, elem.colors[i].g/255.0, elem.colors[i].b/255.0);
//                glVertex3f(elem.pts[i+1].x, elem.pts[i+1].y, elem.pts[i+1].z);
//                glVertex3f(elem.pts[i+2].x, elem.pts[i+2].y, elem.pts[i+2].z);
//                glVertex3f(elem.pts[i+3].x, elem.pts[i+3].y, elem.pts[i+3].z);

//                double A, B, C, D;
//                fitPlane(elem.pts[i+1], elem.pts[i+2], elem.pts[i+3], A, B, C, D);
//                ptdistbuf.push_back(pt2PlaneDist(elem.pts[i], A, B, C, D));
//            }
//            glEnd();

            //画特征点
//            glPointSize(elem.pointsize);
//            glBegin(GL_POINTS);
//            for (int i=0; i<(elem.pts.size()-3); i+=4){
//                //glColor3f(elem.colors[i].r/255.0, elem.colors[i].g/255.0, elem.colors[i].b/255.0);
//                glColor3f(0.0, 0.0, 1.0);
//                glVertex3f(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z);
//            }
//            glEnd();

            //显示距离
//            for (int i=0; i<(elem.pts.size()-3); i+=4){
//                if (ptdistbuf.size()==0)
//                    break;
//                qglColor(Qt::black);
//                renderText(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z, QString::number(ptdistbuf[i/4],'g', 3), QFont("Times New Roman", 10, QFont::Normal, false));
//            }

            //显示编号
//            for (int i=0; i<(elem.pts.size()-3); i+=4){
//                qglColor(Qt::black);
//                renderText(elem.pts[i].x, elem.pts[i].y, elem.pts[i].z, QString("%1").arg(i/4), QFont("Times New Roman", 10, QFont::Normal, false));
//            }
        }
    }
}

void GLWidget::setPointCloudFrame(std::vector<DrawingElem> elems)
{
    drawingelements = elems;
    draw();
    updateGL();
}

void GLWidget::draw()
{
    //drawAxis(0.6);
    drawPointCloud();
}

void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
{
    double x=x2-x1;
    double y=y2-y1;
    double z=z2-z1;
    double L=sqrt(x*x+y*y+z*z);
    const double RADPERDEG = 0.0174533;

    GLUquadricObj *quadObj;

    glPushMatrix ();

    glTranslated(x1,y1,z1);

    if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
    } else if (z<0){
        glRotated(180,1.,0.,0.);
    }

    glTranslatef(0,0,L-4*D);

    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
    gluDeleteQuadric(quadObj);

    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, 2*D, 32, 1);
    gluDeleteQuadric(quadObj);

    glTranslatef(0,0,-L+4*D);

    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, D, D, L-4*D, 32, 1);
    gluDeleteQuadric(quadObj);

    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, D, 32, 1);
    gluDeleteQuadric(quadObj);

    glPopMatrix ();

}
void GLWidget::drawAxis(double length)
{
    glPushMatrix();
    glColor3f(1,0,0);
    glTranslatef(-length,0,0);
    Arrow(length,0,0, 2*length,0,0, 0.04);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0,1,0);
    glTranslatef(0,-length,0);
    Arrow(0,length,0, 0,2*length,0, 0.04);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0,0,1);
    glTranslatef(0,0,-length);
    Arrow(0,0,length, 0,0,2*length, 0.04);
    glPopMatrix();
}


