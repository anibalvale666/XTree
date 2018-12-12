#ifndef POINT_H
#define POINT_H

#include <vector>
#include <iostream>
using namespace std;

template < class T>
class CPoint
{
  public:
    CPoint(vector<T> ); //constructor para  dimensiones pasando el objeto con n dimensiones como un vector
    CPoint(T , T ); //constructor para dos dimensiones
    CPoint(){} //constructor que o recibe nada
    vector<T> coord; //vector que guarda los datos
    double calc_distancia(CPoint<T>);


    template<class U>friend ostream& operator<<(ostream &, const CPoint<U>&);


  private:
};

#endif
