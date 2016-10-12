#ifndef CONFLICTLIST_H
#define CONFLICTLIST_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <GUI/managers/dcelmanager.h>
#include <lib/dcel/dcel_vertex_iterators.h>
#include <vector>
#include <string>


class conflictlist
{
public:
    conflictlist(DrawableDcel* dcel, const std::vector<Pointd> &vectorPoint);
    //std::unordered_map<Dcel::Vertex*,std::set<Dcel::Face*>*> maptoface;
    //std::unordered_map<Dcel::Face*,std::set<Dcel::Vertex*>*> maptovertex;

private:
    //variables
    std::vector<Pointd> vectorPoint;
    DrawableDcel *dcel;
};

#endif // CONFLICTLIST_H
