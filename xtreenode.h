#ifndef RTREENODE_H
#define RTREENODE_H

#include "mbr.h"
#include "mbr.cpp"

template<class T>
class CXTreeNode
{
  public:
    CXTreeNode(int);
    bool is_leaf();
    bool is_supernode();
    bool insert_point(CPoint<T> x);
    bool insert_mbr(CMbr<T> x,CXTreeNode<T> *p,int pos);
    bool insert_mbr(CMbr<T> x,CXTreeNode<T> *p);
    bool cxtn_leaf;  //si es un nodo hoja o no
    bool cxtn_supernode;
    double cxtn_split_history;
    int cxtn_M;

     template<class U> friend class CXTree;
  private:
    //vector<pair<Cmbr<T> ,CXTreeNode *> > cxtn_mbr_and_pointers; //rectangulos con los punteros a los hijos
    vector<CMbr<T> > cxtn_mbr; // hyperrectangulos
    vector<CXTreeNode<T> * > cxtn_children_pointers; //esto crece si no es nodo hoja

    vector<CPoint<T> > cxtn_points; //data espacial

};



#endif
