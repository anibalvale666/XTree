#define GLUT_DISABLE_ATEXIT_HACK
#include <iostream>
#include <math.h>
#include<GL/glut.h>
#include<vector>
#include "xtree.h"
#include "xtree.cpp"
#include <fstream>


#define KEY_ESC 27
using namespace std;

//Crear quad tree
//CXTree<double> a(50,-300,300, 300,-300);
CXTree <double> *a;
CMbr<int> rect(CPoint<int>(0,0),CPoint<int>(0,0));
int cont;
int cx =0,cy=0,check=0,x2,x3,y2,y3;
int w_window = 600,h_window = 600;
vector<CPoint<double> > searched_points;


int conv_coord_x(int x)
{
	return x-(w_window/2);
}
int conv_coord_y(int y)
{
	return (h_window/2)-y;
}

vector<CPoint<double> > puntosCapitales;
void gg1()
{
	double maxx =0,minx =10000000; 	double maxy =0,miny =10000000;
	ifstream fe("crimesll.txt");
	double longitude,latitude;
	fe >> cont;
	vector<double> xvect;
	vector<double> yvect;
	vector<double> ll_point;
	for(int i=0;i<cont;i++)
	{

      fe >> longitude >> latitude ;
      //cout << country_name << " " << capital_name << " " << latitude << " " << longitude << " " << country_code << " " << continent_name << endl;
			//cout  << latitude << " " << longitude << endl;
			if(maxx < longitude)
			{
				maxx = longitude;
			}
			if(minx > longitude)
			{
				minx = longitude;
			}
			if(maxy < latitude)
			{
				maxy = latitude;
			}
			if(miny > latitude)
			{
				miny = latitude;
			}

			//normalizacion
			ll_point.push_back(longitude);
			ll_point.push_back(latitude);

			puntosCapitales.push_back(CPoint<double>(ll_point));
			ll_point.clear();
	}
	int inserted_elements =0;
	for(int i = 0;i < cont;i++)
	{
		puntosCapitales[i].coord[0] = ( (puntosCapitales[i].coord[0] - minx)/(maxx-minx) )* 600;
		puntosCapitales[i].coord[1] = ( (puntosCapitales[i].coord[1] - miny)/(maxy-miny) )* 600;

		puntosCapitales[i].coord[0] = puntosCapitales[i].coord[0] - 300;
		puntosCapitales[i].coord[1] = puntosCapitales[i].coord[1] - 300;

		a->insert_point(puntosCapitales[i]);
		//cout << i << endl;
	}

	fe.close();

}

void gg2()
{
	double maxx =0,minx =10000000; 	double maxy =0,miny =10000000;
	ifstream fe("YearPredictionMSD.txt");
	fe >> cont;
	vector<double> d91(91,0);
	vector<double> xvect;
	vector<double> yvect;
	vector<double> ll_point;
	for(int i=0;i<cont;i++)
	{
			for(int k=0;k<91;k++)
      	fe >> d91[k] ;



		a->insert_point(CPoint<double>(d91));

		//cout << i << endl;
	}

	fe.close();

}

//dibuja un simple gizmo
void displayGizmo()
{
	if(check = 1)
	{
		check = 0;
		glPointSize(5.0);
		glBegin(GL_POINTS);
		glColor3f(1.0,1.0,1.0);
		glVertex2f(cx,cy);
		glEnd();
	}
	rect.draw(20,20,20);


	for(int i=0;i < searched_points.size() ;i++)
	{
		double aux1,aux2;
		aux1 = searched_points[i].coord[0];
		aux2 = searched_points[i].coord[1];


		glPointSize(3.0);
		glBegin(GL_POINTS);
		glColor3f(0.0,0.0,1.0);
		glVertex2f(aux1,aux2);
		glEnd();
		//cout << regionPoints[i].coord[0] << " " << regionPoints[i].coord[1] << endl;
	}
	searched_points.clear();

}


void OnMouseClick(int button, int state, int x, int y)
{
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
  {
    //convertir x,y
		//insertar un nuevo punto en el quadtree
		check = 1;
		//cout << x << " " << y << endl;
		//cin >> x >> y;
		cx = x-(w_window/2);
		cy = (h_window/2)-y;
		a->insert_point(CPoint<double>(cx,cy));
  }

}

void OnMouseMotion(int x, int y)
{
     //opcional
	 //hacer algo x,z cuando se mueve el mouse
	 int lado = 50;
	 x2 = conv_coord_x(x-lado);
	 x3 = conv_coord_x(x+lado);
	 y2 = conv_coord_y(y-lado);
	 y3 = conv_coord_y(y+lado);

	 rect.cmbr_esq[0] = CPoint<int>(x2,y2);
	  rect.cmbr_esq[1] = CPoint<int>(x3,y3);

	 //CMbr<double> rect_search(CPoint<double>(conv_coord_x(x)-lado,conv_coord_y(y)-lado),CPoint<double>(conv_coord_x(x)+lado,conv_coord_y(y)+lado));

	 //a->find_by_region(a->cxt_root,rect_search,searched_points);

}

//

void idle(){ // AGREGAR ESTA FUNCION
	glutPostRedisplay();
}

//funcion llamada a cada imagen
void glPaint(void) {

	//El fondo de la escena al color initial
	glClear(GL_COLOR_BUFFER_BIT); //CAMBIO
	glLoadIdentity();
	glOrtho(-300.0f,  300.0f,-300.0f, 300.0f, -1.0f, 1.0f);

	//dibujar quadTree (qt->draw())
		a->drawTree();
	//dibuja el gizmo
	displayGizmo();
	//doble buffer, mantener esta instruccion al fin de la funcion
	glutSwapBuffers();
}

//
//inicializacion de OpenGL
//
void init_GL(void) {
	//Color del fondo de la escena
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f); //(R, G, B, transparencia) en este caso un fondo negro

	//modo projeccion
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}

//en el caso que la ventana cambie de tamaño
GLvoid window_redraw(GLsizei width, GLsizei height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}

GLvoid window_key(unsigned char key, int x, int y) {
	switch (key) {
	case KEY_ESC:
		exit(0);
		break;

	default:
		break;
	}

}
//
//el programa principal
//

int main(int argc, char** argv) {

	int M;
	cout << " ingresa el M: ";
	cin >> M;
	a = new CXTree<double>(M,-300,300, 300,-300);
	gg2();
	cout << "f";
	cin>>M;
	//a->print();


	//Inicializacion de la GLUT
	/*glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(600, 600); //tamaño de la ventana
	glutInitWindowPosition(100, 100); //posicion de la ventana
	glutCreateWindow("Equistree"); //titulo de la ventana

	init_GL(); //funcion de inicializacion de OpenGL

	glutDisplayFunc(glPaint);
	glutReshapeFunc(&window_redraw);
	// Callback del teclado
	glutKeyboardFunc(&window_key);
	glutMouseFunc(&OnMouseClick);
	glutPassiveMotionFunc(&OnMouseMotion);
	glutIdleFunc(&idle);


	glutMainLoop(); //bucle de rendering
	//no escribir nada abajo de mainloop

*/

}
