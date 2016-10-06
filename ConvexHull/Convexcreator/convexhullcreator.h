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
    void hullcreator();

private:
    void findVertices();
    void randomfour();
    bool circleControll() const;
    void tethraCreation();
    std::vector<Dcel::Face*> addNewFace(std::list<Dcel::HalfEdge*>,Dcel::Vertex*);


    //variables
    int npoints;
    std::vector<Pointd> vectorPoint;
    DrawableDcel *dcel;
};


#endif // CONVEXHULLCREATOR_H
