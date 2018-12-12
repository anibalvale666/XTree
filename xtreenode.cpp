#include "xtreenode.h"

template<class T>
CXTreeNode<T>::CXTreeNode(int M)
{
  cxtn_leaf = true;
  cxtn_supernode =false;
  cxtn_split_history = -1;
  cxtn_M = M;
}

template<class T>
bool CXTreeNode<T>::is_leaf()
{
    return cxtn_leaf;
}

template<class T>
bool CXTreeNode<T>::is_supernode()
{
    return cxtn_supernode;
}

template<class T>
bool CXTreeNode<T>::insert_point(CPoint<T> x)
{
  cxtn_points.push_back(x);
  return 1;
}

template<class T>
bool CXTreeNode<T>::insert_mbr(CMbr<T> x,CXTreeNode<T> *p,int pos)
{
  cxtn_mbr.insert(cxtn_mbr.begin()+pos,x);
  cxtn_children_pointers.insert(cxtn_children_pointers.begin()+pos,p);
  return 1;
}

template<class T>
bool CXTreeNode<T>::insert_mbr(CMbr<T> x,CXTreeNode<T> *p)
{
  cxtn_mbr.push_back(x);
  cxtn_children_pointers.push_back(p);
  return 1;
}
