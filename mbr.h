#ifndef MBR_H
#define MBR_H

#include <vector>
#include <cmath>
#include <GL/glut.h>
using namespace std;

#include "point.h"
#include "point.cpp"

///-------------------------------clase Mbr--------------------------------///
template<class T>
class CMbr
{
  public:
    CPoint<T> cmbr_esq[2];

    CMbr(CPoint<T> , CPoint<T>); //constructor
    CMbr(){}
    CMbr(CPoint<T>);

    bool contiene(CPoint<T>); //verifica si el rectaqngulo contiene a un punto
    double calc_area();
    double calc_enlargement(CPoint<T>); //calcula cuanto se alarga un rectangulo con un punto dado
    double calc_enlargement(CMbr<T>); //calcula cuanto se alarga con un rectangulo dado
    double calc_margin();
    bool calc_overlap(CMbr<T>);

    bool enlargement(CPoint<T>); //alarga el rectangulo a un punto dado
    bool enlargement(CMbr<T>);

    bool operator ==(const CMbr<T>);

    void print();
    void draw(); //dibuja el rectangulo
    void draw(double,double,double); //dibuja el rectangulo con colores definidps


    template<class U>friend ostream& operator<<(ostream &, const CMbr<U>&);
};

#endif
