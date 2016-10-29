#ifndef CONVEXHULLCREATOR_H
#define CONVEXHULLCREATOR_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <GUI/managers/dcelmanager.h>
#include <lib/dcel/dcel_vertex_iterators.h>
#include <vector>
#include <string>

class convexhullCreator
{
public:
    convexhullCreator(DrawableDcel* dcel);
    void hullcreator(MainWindow *mainWindow);
    void hullcreator();
    ~convexhullCreator();

private:
    void findVertices();
    void randomfour();
    bool circleControll() const;
    void tethraCreation();
    std::list<Dcel::HalfEdge *> horizonSort(std::list<Dcel::HalfEdge *>)const;
    std::list<Dcel::HalfEdge*> setHorizon(std::set<Dcel::Face *> *)const;
    void setTwins(std::vector<Dcel::HalfEdge*>,std::vector<Dcel::HalfEdge*>,int);
    std::deque<Dcel::Face*> addNewFace(std::list<Dcel::HalfEdge*>,Dcel::Vertex*);
    void delFacesVisib(std::set<Dcel::Face*>*);




    //variables
    int npoints;
    std::vector<Dcel::Vertex*> vectorPoint;
    DrawableDcel *dcel;
};


#endif // CONVEXHULLCREATOR_H
