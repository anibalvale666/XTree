#include "mbr.h"


//implementaciones
template<class T>
CMbr<T>::CMbr(CPoint<T> e1, CPoint<T> e2)
{
  cmbr_esq[0] = e1;
  cmbr_esq[1] = e2;
}


//cuando solo tenemos un punto
template<class T>
CMbr<T>::CMbr(CPoint<T> e1)
{
  cmbr_esq[0] = cmbr_esq[1] = e1;
}


template<class T>
double CMbr<T>::calc_area()
{
  double result = 1;
  for(int i=0;i < cmbr_esq[0].coord.size();i++)
  {
    result *= abs(cmbr_esq[0].coord[i] - cmbr_esq[1].coord[i]);
  }
  return result;
}

template<class T>
bool CMbr<T>::contiene(CPoint<T> x)
{
  T min,max;
  for(int i=0;i < cmbr_esq[0].coord.size();i++)
  {
    if(cmbr_esq[0].coord[i] < cmbr_esq[1].coord[i])
    {
      min = cmbr_esq[0].coord[i];
      max = cmbr_esq[1].coord[i];
    }
    else
    {
      min = cmbr_esq[1].coord[i];
      max = cmbr_esq[0].coord[i];
    }
    if(x.coord[i] < min or x.coord[i] > max) return false;
  }
  return true;
}

template<class T>
double CMbr<T>::calc_enlargement(CPoint<T> x)
{
  CMbr<T> rect_enlargement = *this;
  T * min,*max;
  if(contiene(x)) return 0;
  for(int i=0;i < cmbr_esq[0].coord.size();i++) //min.coord.size() esto lleva el numero de coordenadas que es la dimension
  {
    //primero identifico el minimo y el maximo de la dimension i
    if(rect_enlargement.cmbr_esq[0].coord[i] < rect_enlargement.cmbr_esq[1].coord[i])
    {
      min = &rect_enlargement.cmbr_esq[0].coord[i];
      max = &rect_enlargement.cmbr_esq[1].coord[i];
    }
    else
    {
      min = &rect_enlargement.cmbr_esq[1].coord[i];
      max = &rect_enlargement.cmbr_esq[0].coord[i];
    }

    //despues comparo las coordenadas del punto con cada coordenada del rectangulo_enlargement
    if(x.coord[i] < *min)
    {
      *min = x.coord[i];
    }
    else if(x.coord[i] > *max)
      *max = x.coord[i];
  }

  return rect_enlargement.calc_area() - this->calc_area();
}

template<class T>
double CMbr<T>::calc_enlargement(CMbr<T> x)
{
  CMbr<T> rect_enlargement = *this;

  if(contiene(x.cmbr_esq[0]) and contiene(x.cmbr_esq[1])) return 0;
  rect_enlargement.enlargement(x.cmbr_esq[0]);
  rect_enlargement.enlargement(x.cmbr_esq[1]);

  return rect_enlargement.calc_area() - this->calc_area();
}

template<class T>
double CMbr<T>::calc_margin()
{
  double res=0;
  for(int i=0;i < cmbr_esq[0].coord.size();i++)
    res += 2*abs(cmbr_esq[0].coord[i] - cmbr_esq[1].coord[i]);
  return res;
}


template<class T>
bool CMbr<T>::enlargement(CPoint<T> x)
{
  if(contiene(x)) return 0;
  T * min,*max;
  for(int i=0;i < cmbr_esq[0].coord.size();i++)
  {
    if(cmbr_esq[0].coord[i] < cmbr_esq[1].coord[i])
    {
      min = &cmbr_esq[0].coord[i];
      max = &cmbr_esq[1].coord[i];
    }
    else
    {
      min = &cmbr_esq[1].coord[i];
      max = &cmbr_esq[0].coord[i];
    }

    //despues comparo las coordenadas del punto con cada coordenada del rectangulo_enlargement
    if(x.coord[i] < *min)
    {
      *min = x.coord[i];
    }
    else if(x.coord[i] > *max)
      *max = x.coord[i];
  }
  return 1;
}

template<class T>

bool CMbr<T>::enlargement(CMbr<T> e)
{
  this->enlargement(e.cmbr_esq[0]);
  this->enlargement(e.cmbr_esq[1]);

  return 1;
}



template<class T>
bool CMbr<T>::calc_overlap(CMbr<T> other)
{
  //evaluamos el cuadrado propietario
  if(this->contiene(other.cmbr_esq[0]) or this->contiene(other.cmbr_esq[1])) return true;
  if(other.contiene(this->cmbr_esq[0]) or other.contiene(this->cmbr_esq[1])) return true;
  return false;
}

template<class T>
bool CMbr<T>::operator ==(const CMbr<T> der)
{
  if(this->cmbr_esq[0].coord == der.cmbr_esq[0].coord)
    if(this->cmbr_esq[1].coord == der.cmbr_esq[1].coord)
      return 1;
  return 0;
}

template<class T>
void CMbr<T>::print()
{
  cout <<"[ ";
  cmbr_esq[0].print();
  cout << "   ";
  cmbr_esq[1].print();
  cout << " ]";
}


template<class T>
void CMbr<T>::draw()
{


  glBegin(GL_LINES);
  //lineavertical
  glColor3d(255,0,0);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[1].coord[1]);
  glColor3d(255,0,0);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[1].coord[1]);

  //linea horizontal
  glColor3d(255,0, 0);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[0].coord[1]);
  glColor3d(255,0,0);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[1].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[1].coord[1]);

  glEnd();
}


template<class T>
void CMbr<T>::draw(double r,double g,double b)
{
  //r = r%255;
  //g = g%255;
  //b = b%255;
  glBegin(GL_LINES);
  //lineavertical
  glColor3d(r,g,b);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[1].coord[1]);
  glColor3d(r,g,b);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[1].coord[1]);

  //linea horizontal
  glColor3d(r,g,b);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[0].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[0].coord[1]);
  glColor3d(r,g,b);
  glVertex2d(cmbr_esq[0].coord[0], cmbr_esq[1].coord[1]);
  glVertex2d(cmbr_esq[1].coord[0], cmbr_esq[1].coord[1]);
  glEnd();

}


template<class T>
ostream& operator<<(ostream& os, const CMbr<T>& cm)
{
  os << "[ " << cm.cmbr_esq[0] << "   " << cm.cmbr_esq[1] << " ]";
  return os;
}
