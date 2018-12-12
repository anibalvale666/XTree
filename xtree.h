#ifndef XTREE_H
#define XTREE_H

#include "xtreenode.h"
#include "xtreenode.cpp"

template<class T>
class CXTree
{
  public:
    CXTree(){} // constructor vacio;
    CXTree(int,int,int,int,int);

    bool find_point(T x,CXTreeNode<T> * &p);

    //-funciones usadas en la insercion y auxiliares
    bool insert_point(CPoint<T> );
    void choose_leaf(CPoint<T> ,vector<pair< CXTreeNode<T> *,int > > & );
    void pick_seeds(CXTreeNode<T> * ,CMbr<T> &, CMbr<T> &,int (&)[2]);
    void pick_next(CXTreeNode<T> *,int &,CMbr<T> , CMbr<T> );
    void adjust_tree(vector<pair< CXTreeNode<T> *,int > > & );
    void split(CXTreeNode<T> *,CXTreeNode<T> * ,int );

    //funciones asterisco
    void split_ast(CXTreeNode<T> *,CXTreeNode<T> * ,int ); //306
    int choose_split_axis(vector<CMbr<T> > );
    void choose_split_index(CXTreeNode<T>, CXTreeNode<T> &, CXTreeNode<T> &, int );

    //static bool sort_node_min(const CMbr<T> &, const CMbr<T> &);
    //static bool sort_node_max(const CMbr<T> &,   const CMbr<T> &);


    //funciones xTREe
    int insert(CPoint<T>,CXTreeNode<T> **);
    void split_x(CXTreeNode<T> *,CXTreeNode<T> * ,int );
    void topological_split(CXTreeNode<T> ,CXTreeNode<T> &, CXTreeNode<T> &); //147
    void overlap_minimal_split(CXTreeNode<T> ,CXTreeNode<T> &, CXTreeNode<T> &); //


    //knn
    void search_k_points(int ,CPoint<T>,vector<CPoint<T> >&);

    //funcion print
    void print();
    void drawTree();

    CXTreeNode<T>  *cxt_root;
  private:
    int cxt_M;
    int cxt_x_min, cxt_y_min, cxt_x_max, cxt_y_max;
    int MAX_Overlap=0.2; //20%


};


#endif
