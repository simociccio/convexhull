#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <GUI/managers/dcelmanager.h>
#include <lib/dcel/dcel_vertex_iterators.h>
#include <vector>
#include <string>


class conflictgraph
{
public:
    conflictgraph(DrawableDcel* dcel, std::vector<Dcel::Vertex*> &vectorPoint);
    void addFaceToVertexList(Dcel::Vertex*, Dcel::Face*);
    void addVertexToFaceList(Dcel::Face*, Dcel::Vertex*);
    std::set<Dcel::Face*>* isVisibleByV(Dcel::Vertex*);
    std::set<Dcel::Vertex*>* isVisibleByF(Dcel::Face*);
    void removeVertex(Dcel::Vertex*);
    void removeFaces(std::set<Dcel::Face*>*setFaces);
    void updateCg(std::set<Dcel::Vertex*> *,Dcel::Face* );
    std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> visibleVertexToTest(std::list<Dcel::HalfEdge *>);


    std::map<Dcel::Vertex*,std::set<Dcel::Face*>*> maptoface;
    std::map<Dcel::Face*,std::set<Dcel::Vertex*>*> maptover;



    void createGraph();

private:
    //variables
    std::vector<Dcel::Vertex*> vectorPoint;
    DrawableDcel *dcel;
    int npoints;


};

#endif // CONFLICTGRAPH_H
