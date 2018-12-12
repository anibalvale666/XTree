#include "xtree.h"
#include <algorithm>
#include <stdlib.h>

//implementacion
template<class T>
CXTree<T>::CXTree(int max,int x_min, int y_min, int x_max, int y_max )
  :cxt_M(max), cxt_x_min(x_min), cxt_y_min(y_min), cxt_x_max(x_max), cxt_y_max(y_max)
{
  cxt_root = new CXTreeNode<T>(cxt_M);
}



//---------------------------funcion insertar------------------------------//

template<class T>
bool CXTree<T>::insert_point(CPoint<T> x)
{
  //I1 Encontramos la posicion para el nuevo record
  vector<pair< CXTreeNode<T> *,int > > path;
  choose_leaf(x,path);
  //verificamos que p no contenga el mismo punto
  for(int i=0;i<path[path.size()-1].first->cxtn_points.size();i++)
    if(x.coord == path[path.size()-1].first->cxtn_points[i].coord)
      return 0;

  //I2 agregamos registro
  path[path.size()-1].first->cxtn_points.push_back(x);

  //hacemos la verificacione ocn la funcion adjustTree

  adjust_tree(path);
  //cout << "finisj" << endl;
  return 1;
}

template<class T>
void CXTree<T>::choose_leaf(CPoint<T> x,vector<pair< CXTreeNode<T> *,int > > &path)
{
  //cout << "cf" <<endl;
  T min_distance,actual_distance;
  int pivot;
  CXTreeNode<T> *p;
  for(p = cxt_root;!p->is_leaf(); p = p->cxtn_children_pointers[pivot])
  {
    pivot =0;
    min_distance = p->cxtn_mbr[0].calc_enlargement(x);
    for(int i=1;i <p->cxtn_mbr.size();i++)
    {
      actual_distance = p->cxtn_mbr[i].calc_enlargement(x);
      if(actual_distance == min_distance)
      {
        if(p->cxtn_mbr[pivot].calc_area() > p->cxtn_mbr[i].calc_area()) pivot = i;
      }
      else if(actual_distance < min_distance)
      {
        pivot = i;
        min_distance = actual_distance;
      }
    }
    path.push_back(make_pair(p,pivot));
  }
  path.push_back(make_pair(p,p->cxtn_points.size()));

}

//-------------------------------funcion Split--------------------------------//
template<class T>
void CXTree<T>::split(CXTreeNode<T> *s,CXTreeNode<T> * p,int pc)
{
  //cout << "sp" << endl;

  CMbr<T> seed_rect1,seed_rect2;
  CXTreeNode<T> in(cxt_M),out1(cxt_M),out2(cxt_M);
  int d_split;

  in = *s;
  //cout << " root" << endl;
  //split del root
  if(s == p )
  {
    CXTreeNode<T> *new_root = new CXTreeNode<T>(cxt_M);
    new_root->cxtn_leaf = 0;
    p = cxt_root = new_root;
    p->insert_mbr(seed_rect1,s,0);
  }

  CXTreeNode<T> *nn = new CXTreeNode<T>(cxt_M);
  p->cxtn_mbr[pc] =seed_rect1; ///////////////////////////////////////////
  p->insert_mbr(seed_rect2,nn,pc+1);
  if(s->is_leaf())
  {
    s->cxtn_points.clear();
    topological_split(in,out1,out2);
    seed_rect1 = CMbr<T>(out1.cxtn_points[0],out1.cxtn_points[0]);
    seed_rect2 = CMbr<T>(out2.cxtn_points[0],out2.cxtn_points[0]);

    s->cxtn_points = out1.cxtn_points;
    nn->cxtn_points = out2.cxtn_points;
    for(int i=0;i<out1.cxtn_points.size();i++)
      seed_rect1.enlargement(out1.cxtn_points[i]);

    for(int i=0;i<out2.cxtn_points.size();i++)
      seed_rect2.enlargement(out2.cxtn_points[i]);

    p->cxtn_mbr[pc] = seed_rect1;
    p->cxtn_mbr[pc+1] = seed_rect2;
  }
  else
  {
    CXTreeNode<T> node_aux(cxt_M) ;
    node_aux=*s;

    nn->cxtn_leaf = 0;

      //cout << " j"<< endl;

    s->cxtn_points.clear();
    s->cxtn_children_pointers.clear();


    topological_split(in,out1,out2);
    seed_rect1 = out1.cxtn_mbr[0];
    seed_rect2 = out2.cxtn_mbr[0];

    s->cxtn_mbr = out1.cxtn_mbr;
    s->cxtn_children_pointers = out1.cxtn_children_pointers;
    nn->cxtn_mbr = out2.cxtn_mbr;
    nn->cxtn_children_pointers = out2.cxtn_children_pointers;

    for(int i=0;i<out1.cxtn_mbr.size();i++)
      seed_rect1.enlargement(out1.cxtn_mbr[i]);

    for(int i=0;i<out2.cxtn_mbr.size();i++)
      seed_rect2.enlargement(out2.cxtn_mbr[i]);

    p->cxtn_mbr[pc] = seed_rect1;
    p->cxtn_mbr[pc+1] = seed_rect2;

  }
  //cout << "gg" <<endl;
  //cout << "finsplit" << endl;
  //cout <<seed_rect1 << p->cxtn_mbr[1] << " "<< cxt_root <<" s:" << &s << " nn:" << &nn << endl;
}


template<class T>
void CXTree<T>::topological_split(CXTreeNode<T> in, CXTreeNode<T>  &out1, CXTreeNode<T> &out2)
{
  CMbr<T> r0,r1;
  int m = ceil(in.cxtn_M/2);
  int pivotes[2];
  double area1,area2,enlargement1,enlargement2;
  bool option; //0 es ou0t1 1 es ouut2
  int pos;


  if(in.is_leaf())
  {
    pick_seeds(&in,r0,r1,pivotes);
    out1.insert_point(r0.cmbr_esq[0]);
    out2.insert_point(r1.cmbr_esq[1]);
    in.cxtn_points.erase(in.cxtn_points.begin() + pivotes[0]);
    in.cxtn_points.erase(in.cxtn_points.begin() + pivotes[1]);

    while(out1.cxtn_points.size() <= m and out2.cxtn_points.size() <= m)
    {
      pick_next(&in,pos,r0,r1);
      enlargement1 = r0.calc_enlargement(in.cxtn_points[pos]);
      enlargement2 = r1.calc_enlargement(in.cxtn_points[pos]);
      if( enlargement1 == enlargement2)
      {
        area1 = r0.calc_area();
        area2 = r1.calc_area();
        if(area1 == area2)
        {
          if(out1.cxtn_points.size() <=  out2.cxtn_points.size())
            option = 0;
          else
            option = 1;
        }
        else
        {
          if(area1 < area2)
            option = 0;
          else
            option = 1;
        }
      }
      else if(enlargement1 < enlargement2)
        option = 0;
      else
        option = 1;

      if(option)
      {
        out2.insert_point(in.cxtn_points[pos]);
        r1.enlargement(in.cxtn_points[pos]);
      }
      else
      {
        out1.insert_point(in.cxtn_points[pos]);
        r0.enlargement(in.cxtn_points[pos]);
      }
      in.cxtn_points.erase(in.cxtn_points.begin()+pos);

    }
    if(out1.cxtn_points.size() >= m)
    {
      for(int i =0;i<in.cxtn_points.size();i++)
        r1.enlargement(in.cxtn_points[i]);
        out2.cxtn_points.insert(out2.cxtn_points.end(),in.cxtn_points.begin(),in.cxtn_points.end());
    }
    else
    {
      for(int i=0;i<in.cxtn_points.size();i++)
        r0.enlargement(in.cxtn_points[i]);
      out1.cxtn_points.insert(out1.cxtn_points.end(),in.cxtn_points.begin(),in.cxtn_points.end());
    }

  }
  else //else medio----------------------------//
  {
       int pos;
       pick_seeds(&in,r0,r1,pivotes);

       out1.insert_mbr(r0,in.cxtn_children_pointers[pivotes[0]]);
       out2.insert_mbr(r1,in.cxtn_children_pointers[pivotes[1]]);

       in.cxtn_mbr.erase(in.cxtn_mbr.begin() + pivotes[0]);
       in.cxtn_children_pointers.erase(in.cxtn_children_pointers.begin() + pivotes[0]);

       if(pivotes[0]<pivotes[1]) pivotes[1]--;

       in.cxtn_mbr.erase(in.cxtn_mbr.begin() + pivotes[1]);
       in.cxtn_children_pointers.erase(in.cxtn_children_pointers.begin() + pivotes[1]);


       while(out1.cxtn_mbr.size() <= m and out2.cxtn_mbr.size() <= m)
       {
         pick_next(&in,pos,r0,r1);
         enlargement1 = r0.calc_enlargement(in.cxtn_mbr[pos]);
         enlargement2 = r1.calc_enlargement(in.cxtn_mbr[pos]);
         if( enlargement1 == enlargement2 )
         {
           area1 = r0.calc_area();
           area2 = r1.calc_area();
           if(area1 == area2)
           {
             if(out1.cxtn_mbr.size() <=  out2.cxtn_mbr.size())
               option = 0;
             else
               option = 1;
           }
           else if(area1 < area2)
             option = 0;
           else
             option = 1;
           }
           else if(enlargement1 < enlargement2)
             option = 0;
           else
             option = 1;

           if(option)
           {
             out2.insert_mbr(in.cxtn_mbr[pos],in.cxtn_children_pointers[pos]);
             r1.enlargement(in.cxtn_mbr[pos]);
           }
           else
           {
             out1.insert_mbr(in.cxtn_mbr[pos],in.cxtn_children_pointers[pos]);
             r0.enlargement(in.cxtn_mbr[pos]);
           }
           in.cxtn_mbr.erase(in.cxtn_mbr.begin()+pos);
           in.cxtn_children_pointers.erase(in.cxtn_children_pointers.begin()+pos);
         }

       if(out1.cxtn_mbr.size() >= m)
       {
         for(int i =0;i<in.cxtn_mbr.size();i++)
           r1.enlargement(in.cxtn_mbr[i]);

         out2.cxtn_mbr.insert(out2.cxtn_mbr.end(),in.cxtn_mbr.begin(),in.cxtn_mbr.end());
         out2.cxtn_children_pointers.insert(out2.cxtn_children_pointers.end(),in.cxtn_children_pointers.begin(),in.cxtn_children_pointers.end());
       }
       else
       {
         for(int i=0;i<in.cxtn_mbr.size();i++)
          r0.enlargement(in.cxtn_mbr[i]);
          out1.cxtn_mbr.insert(out1.cxtn_mbr.end(),in.cxtn_mbr.begin(),in.cxtn_mbr.end());
          out1.cxtn_children_pointers.insert(out1.cxtn_children_pointers.end(),in.cxtn_children_pointers.begin(),in.cxtn_children_pointers.end());
      }
  }

}

template<class T>
void CXTree<T>::pick_seeds(CXTreeNode<T> *p ,CMbr<T> &rect1, CMbr<T> &rect2,int (&pivots)[2])
{
  double area=0,aux_area;
  CMbr<T> rect_compare;
  int pivot1,pivot2;

  if(p->is_leaf())
  {
    //evaluamos con todos los puntos
    for(int i=0;i<p->cxtn_points.size()-1;i++)
    {
      rect_compare.cmbr_esq[0] = p->cxtn_points[i];

      for(int j=i+1;j<p->cxtn_points.size();j++)
      {
        rect_compare.cmbr_esq[1] = p->cxtn_points[j];
        aux_area = rect_compare.calc_area();

        if(aux_area > area)
        {
          pivot1 = i;
          pivot2 = j;
          area = aux_area;
        }
      }
    }
    //aqui creamos los dos rectangulos con los dos pivotes que son los puntos que forman el area mas grande
    rect1.cmbr_esq[0] = p->cxtn_points[pivot1];
    rect1.cmbr_esq[1] = p->cxtn_points[pivot1];
    rect2.cmbr_esq[0] = p->cxtn_points[pivot2];
    rect2.cmbr_esq[1] = p->cxtn_points[pivot2];
  }
  else
  {

    for(int i=0;i<p->cxtn_mbr.size()-1;i++)
    {
      for(int j=i+1;j<p->cxtn_mbr.size();j++)
      {
        rect_compare = p->cxtn_mbr[i];
        rect_compare.enlargement(p->cxtn_mbr[j].cmbr_esq[0]);
        rect_compare.enlargement(p->cxtn_mbr[j].cmbr_esq[1]);
        aux_area = abs(rect_compare.calc_area() - p->cxtn_mbr[i].calc_area() - p->cxtn_mbr[j].calc_area());
        if(aux_area > area)
        {
          pivot1 = i;
          pivot2 = j;
          area = aux_area;
        }
      }
    }

    rect1 = p->cxtn_mbr[pivot1];
    rect2 = p->cxtn_mbr[pivot2];
  }

  pivots[0] = pivot1;
  pivots[1] = pivot2;
  //eliminamos los puntos evaluados
  //p->cxtn_points.erase(p->cxtn_points.begin()+pivot1);
  //p->cxtn_points.erase(p->cxtn_points.begin()+pivot2);

}

template<class T>
void CXTree<T>::pick_next(CXTreeNode<T> *p,int &pivot,CMbr<T> rect1, CMbr<T> rect2)
{
  double area1,area2,max;
  max = 0;
  pivot = 0;

  if(p->is_leaf())
  {
    for(int i=0;i<p->cxtn_points.size();i++)
    {
      area1 =rect1.calc_enlargement(p->cxtn_points[i]);
      area2 =rect2.calc_enlargement(p->cxtn_points[i]);
      if(max < abs(area1 - area2 ))
      {
        pivot =i;
        max = abs(area1 - area2);
      }
    }
  }
  else
  {
    CMbr<T> mbr_aux;
    for(int i=0;i<p->cxtn_mbr.size();i++)
    {
      mbr_aux = rect1;
      mbr_aux.enlargement(p->cxtn_mbr[i].cmbr_esq[0]);
      mbr_aux.enlargement(p->cxtn_mbr[i].cmbr_esq[1]);
      area1 = mbr_aux.calc_area() - rect1.calc_area();

      mbr_aux = rect2;
      mbr_aux.enlargement(p->cxtn_mbr[i].cmbr_esq[0]);
      mbr_aux.enlargement(p->cxtn_mbr[i].cmbr_esq[1]);
      area2 = mbr_aux.calc_area() - rect2.calc_area();

      if( abs(area1 - area2 ) > max)
      {
        pivot =i;
        max = abs(area1 - area2);
      }
    }
  }
}


//----------------------funciones ASterisco-----------------------------------///
template<class T>
void CXTree<T>::overlap_minimal_split(CXTreeNode<T> in ,CXTreeNode<T> &out1, CXTreeNode<T> &out2)
{

}

template<class T>
void CXTree<T>::split_ast(CXTreeNode<T> *s,CXTreeNode<T> * p,int pc)
{
  //cout << "sp" << endl;

  CMbr<T> seed_rect1,seed_rect2;
  CXTreeNode<T> in(cxt_M),out1(cxt_M),out2(cxt_M);
  int d_split;

  in = *s;
  //cout << " root" << endl;
  //split del root
  if(s == p )
  {
    CXTreeNode<T> *new_root = new CXTreeNode<T>(cxt_M);
    new_root->cxtn_leaf = 0;
    p = cxt_root = new_root;
    p->insert_mbr(seed_rect1,s,0);
  }

  CXTreeNode<T> *nn = new CXTreeNode<T>(cxt_M);
  p->cxtn_mbr[pc] =seed_rect1; ///////////////////////////////////////////
  p->insert_mbr(seed_rect2,nn,pc+1);
  if(s->is_leaf())
  {
    s->cxtn_points.clear();

    for(int i=0;i<in.cxtn_points.size();i++)
      in.cxtn_mbr.push_back(CMbr<T> (in.cxtn_points[i]));


    //d_split = choose_split_axis(in.cxtn_mbr);
    d_split = rand()%91;

    choose_split_index(in,out1,out2,d_split);
    seed_rect1 = CMbr<T>(out1.cxtn_points[0],out1.cxtn_points[0]);
    seed_rect2 = CMbr<T>(out2.cxtn_points[0],out2.cxtn_points[0]);

    s->cxtn_points = out1.cxtn_points;
    nn->cxtn_points = out2.cxtn_points;
    for(int i=0;i<out1.cxtn_points.size();i++)
      seed_rect1.enlargement(out1.cxtn_points[i]);

    for(int i=0;i<out2.cxtn_points.size();i++)
      seed_rect2.enlargement(out2.cxtn_points[i]);

    p->cxtn_mbr[pc] = seed_rect1;
    p->cxtn_mbr[pc+1] = seed_rect2;

    //guardo el cxtn_split_history
    p->cxtn_split_history = d_split;
  }
  else
  {
    CXTreeNode<T> node_aux(cxt_M);
    node_aux =*s;

    nn->cxtn_leaf = 0;

    d_split = p->cxtn_split_history;

    s->cxtn_mbr.clear();
    s->cxtn_children_pointers.clear();


    choose_split_index(in,out1,out2,d_split);
    seed_rect1 = out1.cxtn_mbr[0];
    seed_rect2 = out2.cxtn_mbr[0];

    s->cxtn_mbr = out1.cxtn_mbr;
    s->cxtn_children_pointers = out1.cxtn_children_pointers;
    nn->cxtn_mbr = out2.cxtn_mbr;
    nn->cxtn_children_pointers = out2.cxtn_children_pointers;

    for(int i=1;i<out1.cxtn_mbr.size();i++)
      seed_rect1.enlargement(out1.cxtn_mbr[i]);

    for(int i=1;i<out2.cxtn_mbr.size();i++)
      seed_rect2.enlargement(out2.cxtn_mbr[i]);

    p->cxtn_mbr[pc] = seed_rect1;
    p->cxtn_mbr[pc+1] = seed_rect2;
  }

  //cout << "gg" <<endl;
  //cout << "finsplit" << endl;
  //cout <<seed_rect1 << p->cxtn_mbr[1] << " "<< cxt_root <<" s:" << &s << " nn:" << &nn << endl;
}



int csa_dimension;
/*template<class T>
static bool sort_node_min(const CMbr<T> &a, const CMbr<T> &b)
{
  int l_aux,d_aux;
  if(a.cmbr_esq[0].coord[csa_dimension] < a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
  if(b.cmbr_esq[0].coord[csa_dimension] < b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;

  return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
}

template<class T>
static bool sort_node_max(const CMbr<T> & a,   const CMbr<T> &b)
{
  int l_aux,d_aux;
  if(a.cmbr_esq[0].coord[csa_dimension] > a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
  if(b.cmbr_esq[0].coord[csa_dimension] > b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
  return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
}
*/



template<class T>
int CXTree<T>::choose_split_axis(vector<CMbr<T> > in)
{
  vector<CMbr<T> > aux =in, left,right;

  int axis_return =0;
  int k = 1;
  int m = ceil((in.size()-1)*0.4);
  CMbr<T> bb1,bb2;
  double sum=1000000000,sum_aux;
  for(int i=0;i<in[0].cmbr_esq[0].coord.size();i++)
  {
    csa_dimension = i; //colocamos la dimension
    sum_aux =0;

    std::sort(aux.begin(), aux.end(), [](const CMbr<T> &a, const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] < a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] < b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;

      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });
    std::sort(aux.begin(), aux.end(), [](const CMbr<T> & a,   const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] > a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] > b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });

    sum_aux =0;
    for(int k =1;k<(in.size()-1)-2*m+2;k++)
    {
      left.clear();
      right.clear();

      left.insert(left.end(), aux.begin(),aux.begin()+ m - 1 + k);
      right.insert(right.end(),aux.begin()+ m - 1 + k+1,aux.end());

      bb1 = left[0];
      bb2 = right[0];
      for(int j=0;j<left.size();j++)
        bb1.enlargement(left[j]);
      for(int j=0;j<right.size();j++)
        bb2.enlargement(right[j]);
      sum_aux += bb1.calc_margin() +  bb2.calc_margin();
    }
    if(sum_aux < sum)
    {
      sum =sum_aux;
      axis_return = i;
    }
  }
  return axis_return;
}

template<class T>
void CXTree<T>::choose_split_index(CXTreeNode<T> in, CXTreeNode<T> &out1, CXTreeNode<T> &out2, int d)
{
  CXTreeNode<T> aux(cxt_M),left(cxt_M),right(cxt_M);
  aux=in;
  CMbr<T> bb1,bb2;
  int pivot = 0;
  int m = ceil(cxt_M*0.4);
  double min_area=10000000000;
  csa_dimension = d;
  double area_aux;


  if(in.is_leaf())
  {
    /*for(int i=0;i<aux.cxtn_points.size();i++)
      aux.cxtn_mbr.push_back(CMbr<T> (aux.cxtn_points[i]));*/

    sort(aux.cxtn_mbr.begin(), aux.cxtn_mbr.end(), [](const CMbr<T> & a,   const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] < a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] < b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });
    sort(aux.cxtn_mbr.begin(), aux.cxtn_mbr.end(),[](const CMbr<T> & a,   const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] > a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] > b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });


    for(int i=0;i<aux.cxtn_mbr.size();i++)
      aux.cxtn_points[i] =aux.cxtn_mbr[i].cmbr_esq[0];

    for(int k = 1;k<in.cxtn_M-2*m+2;k++)
    {
      left.cxtn_points.insert(left.cxtn_points.end(), aux.cxtn_points.begin(), aux.cxtn_points.begin()+ m - 1 + k);
      right.cxtn_points.insert(right.cxtn_points.end(), aux.cxtn_points.begin()+ m - 1 + k+1, aux.cxtn_points.end());

      for(int j=0;j<left.cxtn_points.size();j++)
        bb1.enlargement(left.cxtn_points[j]);
      for(int j=0;j<right.cxtn_points.size();j++)
        bb2.enlargement(right.cxtn_points[j]);
      if(!bb1.calc_overlap(bb2))
      {
        area_aux = bb1.calc_area() + bb2.calc_area();
        if(area_aux < min_area)
        {
          min_area = area_aux;
          pivot =k;
        }
      }
      left.cxtn_points.clear();
      right.cxtn_points.clear();
    }

    left.cxtn_points.insert(left.cxtn_points.end(), aux.cxtn_points.begin(), aux.cxtn_points.begin()+ m - 1 + pivot);
    right.cxtn_points.insert(right.cxtn_points.end(), aux.cxtn_points.begin()+ m - 1 + pivot +1, aux.cxtn_points.end());

  }//else mendio--------------------------------//
  else
  {
    sort(aux.cxtn_mbr.begin(), aux.cxtn_mbr.end(),[](const CMbr<T> & a,   const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] < a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] < b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });
    sort(aux.cxtn_mbr.begin(), aux.cxtn_mbr.end(), [](const CMbr<T> & a,   const CMbr<T> &b)
    {
      int l_aux,d_aux;
      if(a.cmbr_esq[0].coord[csa_dimension] > a.cmbr_esq[1].coord[csa_dimension]) l_aux=0; else l_aux=1;
      if(b.cmbr_esq[0].coord[csa_dimension] > b.cmbr_esq[1].coord[csa_dimension]) d_aux=0; else d_aux=1;
      return (a.cmbr_esq[l_aux].coord[csa_dimension] < b.cmbr_esq[d_aux].coord[csa_dimension]);
    });

    //poner los punteros en posicion
    for(int i=0;i<aux.cxtn_mbr.size() ;i++)
    {
      for(int j=0;j<in.cxtn_mbr.size();j++)
      {
        if(aux.cxtn_mbr[i] == in.cxtn_mbr[j])
          aux.cxtn_children_pointers[i] = in.cxtn_children_pointers[j];
      }
    }


    for(int k = 1;k<in.cxtn_M-2*m+2;k++)
    {
      left.cxtn_mbr.insert(left.cxtn_mbr.end(), aux.cxtn_mbr.begin(), aux.cxtn_mbr.begin()+ m - 1 + k);
      left.cxtn_children_pointers.insert(left.cxtn_children_pointers.end(), aux.cxtn_children_pointers.begin(), aux.cxtn_children_pointers.begin()+ m - 1 + k);
      right.cxtn_mbr.insert(right.cxtn_mbr.end(), aux.cxtn_mbr.begin()+ m - 1 + k+1, aux.cxtn_mbr.end());
      right.cxtn_children_pointers.insert(right.cxtn_children_pointers.end(), aux.cxtn_children_pointers.begin()+ m - 1 + k+1, aux.cxtn_children_pointers.end());

      for(int j=0;j<left.cxtn_mbr.size();j++)
        bb1.enlargement(left.cxtn_mbr[j]);
      for(int j=0;j<right.cxtn_mbr.size();j++)
        bb2.enlargement(right.cxtn_mbr[j]);
      //if(!bb1.calc_overlap(bb2))
      //{
        area_aux = bb1.calc_area() + bb2.calc_area();
        if(area_aux < min_area)
        {
          min_area = area_aux;
          pivot =k;
        }
      //}
      left.cxtn_mbr.clear();
      right.cxtn_mbr.clear();
      left.cxtn_children_pointers.clear();
      right.cxtn_children_pointers.clear();
    }

    left.cxtn_mbr.insert(left.cxtn_mbr.end(), in.cxtn_mbr.begin(), in.cxtn_mbr.begin()+ m - 1 + pivot);
    left.cxtn_children_pointers.insert(left.cxtn_children_pointers.end(), aux.cxtn_children_pointers.begin(), aux.cxtn_children_pointers.begin()+ m - 1 + pivot);
    right.cxtn_mbr.insert(right.cxtn_mbr.end(), in.cxtn_mbr.begin()+ m - 1 + pivot +1, in.cxtn_mbr.end());
    right.cxtn_children_pointers.insert(right.cxtn_children_pointers.end(), aux.cxtn_children_pointers.begin()+ m - 1 + pivot+1, aux.cxtn_children_pointers.end());

  }
  out1 = left; out2 = right;
}


//------------------------------------split xtree-----------------------------//

template<class T>
void CXTree<T>::split_x(CXTreeNode<T> *s,CXTreeNode<T> * p,int pc)
{
  CXTreeNode<T> in(cxt_M),out1(cxt_M),out2(cxt_M);
  CMbr<T> bb1,bb2;
  int axis;

  in = *s;

  if(s->is_leaf())
  {
    split_ast(s,p,pc);
  }
   else
   {
    axis = rand()%91;
    choose_split_index(in,out1,out2,axis);

    topological_split(in,out1,out2);
    bb1 = out1.cxtn_mbr[0];
    bb2 = out2.cxtn_mbr[0];
    for(int j=1;j<out1.cxtn_mbr.size();j++)
      bb1.enlargement(out1.cxtn_mbr[j]);
    for(int j=1;j<out2.cxtn_mbr.size();j++)
      bb2.enlargement(out2.cxtn_mbr[j]);

    if(!bb1.calc_overlap(bb2))
    {
      split_ast(s,p,pc);
    }
    else
    {
      s->cxtn_M += cxt_M;
      s->cxtn_supernode = true;
    }
  }
}



//----------------------------funcion adjust_tree---------------------------//
template<class T>
void CXTree<T>::adjust_tree(vector<pair< CXTreeNode<T> *,int > > &path)
{
  //cout << "iniat" << endl;
  for(int i=path.size()-1;i>0;i--)
  {
    if(path[i].first->cxtn_mbr.size() > path[i].first->cxtn_M or path[i].first->cxtn_points.size() > path[i].first->cxtn_M)
    {

      split_x(path[i].first,path[i-1].first,path[i-1].second);
    }
    else
    {
      if( i == path.size()-1)
      {
        path[i-1].first->cxtn_mbr[path[i-1].second].enlargement(path[i].first->cxtn_points[path[i].second]);
      }
      else
      {
        path[i-1].first->cxtn_mbr[path[i-1].second].enlargement(path[i].first->cxtn_mbr[path[i].second].cmbr_esq[0]);
        path[i-1].first->cxtn_mbr[path[i-1].second].enlargement(path[i].first->cxtn_mbr[path[i].second].cmbr_esq[1]);
      }
    }
  }
  if(path[0].first->cxtn_mbr.size() > path[0].first->cxtn_M or path[0].first->cxtn_points.size() > path[0].first->cxtn_M)
  {
    split_x(path[0].first,path[0].first,0);
  }

}

//-----------------------------knn------------------------------------------//
template<class T>
void search_k_points(int k,CPoint<T> p,vector<CPoint<T> > &res)
{
    vector<pair< CXTreeNode<T> *,int > > path;
    vector<pair< CPoint<T> ,double > > compare_path;
    choose_leaf(k,path);
    path[path.size()-1].first->cxtn_points.push_back(p);

    while(compare_path.size() < k )
    {
      for(int i=0;i < path[path.size()-1].first->cxtn_points.size();i++)
        compare_path.push_back(make_pair(path[path.size()-1].first->cxtn_points[i]),0);
    }

    for(int i =0;i <compare_path.size();i++)
    {
      compare_path[i].second = p.calc_distancia(compare_path[i].first);
    }

    Sort(compare_path.begin(),compare_path.end(),[](const pair< CPoint<T> ,double > & a, pair< CPoint<T> ,double > &b)
    {
      return (a.second < b.first);
    });

}

///////-----------------------funcion imprimir------------------------------///
template<class T>
void CXTree<T>::print()
{
  vector<CXTreeNode<T> *> tail;
  tail.push_back(cxt_root);
  int j = 0;
  int rects =0;
  while(j <tail.size())
  {
    for(int i=0;i<tail[j]->cxtn_mbr.size();i++)
      tail.push_back(tail[j]->cxtn_children_pointers[i]);
    j++;
  }

  for(int i=0;i < tail.size();i++)
  {
    if(tail[i]->is_leaf())
    {
      for(int n=0;n<tail[i]->cxtn_points.size();n++)
      {
        //cout << tail[i]->cxtn_points[n] << endl;
      }
    }
    else
    {
      for(int n=0;n<tail[i]->cxtn_mbr.size();n++)
      {
        //cout << tail[i]->cxtn_mbr[n] << endl;
        rects++;
      }
    }
  }
  cout << rects << endl;
}


template<class T>
void CXTree<T>::drawTree()
{
  vector<CXTreeNode<T> *> tail;
  tail.push_back(cxt_root);
  int j = 0;
  while(j <tail.size())
  {
    for(int i=0;i<tail[j]->cxtn_mbr.size();i++)
      tail.push_back(tail[j]->cxtn_children_pointers[i]);
    j++;
  }

  for(int i=0;i < tail.size();i++)
  {
    if(tail[i]->is_leaf())
    {
      for(int n=0;n<tail[i]->cxtn_points.size();n++)
      {
        			glPointSize(1.0);
        			glBegin(GL_POINTS);
        			glColor3f(0.0,1.0,0.0);
        			glVertex2f(tail[i]->cxtn_points[n].coord[0],tail[i]->cxtn_points[n].coord[1]);
        			glEnd();
      }
    }
    else
    {
      for(int n=0;n<tail[i]->cxtn_mbr.size();n++)
      {
        //if(/*tail[i] == cxt_root or*/ !!tail[i]->cxtn_children_pointers[0]->cxtn_mbr.size())
          tail[i]->cxtn_mbr[n].draw();
      }
    }
  }
}


//-------------------------------funciones Xtree--------------------------------//
/*
bool split(vector<CMbr<T> > in,vector<CMbr<T> > &out1,vector<CMbr<T> >  &out2)
{
    vector<CMbr<T> > t1,t2;
    CMbr<T> r1,r2;
    quadratic_split(in,t1,t2);

    //en r1 y r2 hago un enlargement de los vectores en t1y t2

    //test overlap
    if(t1.calc_overlap(t2))
    {
      minimal_split(in,t1,t2);
      //test for unbalance nodes
      if( (t1.size() < MIN_FANOUT) or (t2.size() <MIN_FANOUT) ) return false;
    }

    out1 =t1; out2 = t2;
    return true;
}*/
