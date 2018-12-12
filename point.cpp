#include "point.h"

template<class T>
CPoint<T>::CPoint(vector<T> a)
{
  coord = a;
}

template<class T>
CPoint<T>::CPoint(T x, T y)
{
  coord.push_back(x);
  coord.push_back(y);
}

template<class T>
double CPoint<T>::calc_distancia(CPoint<T> der)
{
  double distancia = 0;
  for(int i=0;i<der.coord.size();i++)
      distancia += pow(der.coord[i] - this->coord[i], 2) ;
  return sqrt(distancia);
}


template<class T>
ostream& operator<<(ostream& os, const CPoint<T>& cp)
{
    os << " ( ";
    for(int i=0;i<cp.coord.size();i++)
      os << cp.coord[i] << ",";
    os << ")";
    return os;
}
