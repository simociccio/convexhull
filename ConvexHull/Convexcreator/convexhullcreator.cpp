#include "convexhullcreator.h"
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>

convexhullCreator::convexhullCreator(DrawableDcel* dcel){
   this->dcel=dcel;
   this->npoints=dcel->getNumberVertices();
   this->vectorPoint=std::vector<Pointd>(npoints);

}
void convexhullCreator::hullcreator(){
    findVertices();
    randomfour();
    this->dcel->reset();
    tethraCreation();
    std::cout<<"debug";

}

void convexhullCreator::randomfour(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

      shuffle (this->vectorPoint.begin(), this->vectorPoint.end(), std::default_random_engine(seed));
      std::cout<<this->vectorPoint.size();
      bool coplanar=true;

      do{
          //check if the first point are coplanar
          Eigen::Matrix4d matrix;
          for(int i=0; i<4; i++){
              matrix(i, 0) = vectorPoint[i].x();
              matrix(i, 1) = vectorPoint[i].y();
              matrix(i, 2) = vectorPoint[i].z();
              matrix(i, 3) = 1;
          }

          double det = matrix.determinant();

          //check of the determinant
          coplanar = det > -std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon();
          //if we take 4 not coplanar vertices
          if(coplanar){
             randomfour();
          }
      } while(coplanar);

  }

bool convexhullCreator::circleControll() const{
    Eigen::Matrix4d matrix;
    for(int i=0; i<4; i++){
        matrix(i, 0) = vectorPoint[i].x();
        matrix(i, 1) = vectorPoint[i].y();
        matrix(i, 2) = vectorPoint[i].z();
        matrix(i, 3) = 1;
    }
    double det = matrix.determinant();

    //check of the positivity of the determinant
    bool pos = det < -std::numeric_limits<double>::epsilon();
    if(pos){
        return false;
    }else{
        return true;
    }


}
void convexhullCreator::tethraCreation(){

    //create list of pointer of halfedges
    std::list<Dcel::HalfEdge*> listHe;

    //add first 4 points to the dcel
    Dcel::Vertex* v1 = dcel->addVertex(vectorPoint[0]);
    Dcel::Vertex* v2 = dcel->addVertex(vectorPoint[1]);
    Dcel::Vertex* v3 = dcel->addVertex(vectorPoint[2]);
    Dcel::Vertex* v4 = dcel->addVertex(vectorPoint[3]);

    //create the 3 triangle halfedges
    Dcel::HalfEdge* he1 = dcel->addHalfEdge();
    Dcel::HalfEdge* he2 = dcel->addHalfEdge();
    Dcel::HalfEdge* he3 = dcel->addHalfEdge();

    //crete face triangle
    Dcel::Face* f1 = dcel->addFace();
    f1->setOuterHalfEdge(he1);

    if(circleControll()){
        Dcel::Vertex* var = v3;
        v3 = v1;
        v1 = var;
    }
        he1->setFromVertex(v1);
        he1->setToVertex(v2);
        he1->setNext(he2);
        he1->setPrev(he3);
        v1->setIncidentHalfEdge(he1);
        v1->incrementCardinality();
        v2->incrementCardinality();
        he1->setFace(f1);


        he2->setFromVertex(v2);
        he2->setToVertex(v3);
        he2->setNext(he3);
        he2->setPrev(he1);
        v2->setIncidentHalfEdge(he2);
        v2->incrementCardinality();
        v3->incrementCardinality();
        he2->setFace(f1);



        he3->setFromVertex(v3);
        he3->setToVertex(v1);
        he3->setNext(he1);
        he3->setPrev(he2);
        v3->setIncidentHalfEdge(he3);
        v3->incrementCardinality();
        v1->incrementCardinality();
        he3->setFace(f1);

        listHe.push_back(he1);
        listHe.push_back(he2);
        listHe.push_back(he3);

        addNewFace(listHe,v4);

}

std::vector<Dcel::Face*> convexhullCreator::addNewFace(std::list<Dcel::HalfEdge*>listHe,Dcel::Vertex* v3){

    std::vector<Dcel::Face*> newFace=std::vector<Dcel::Face* >(listHe.size());

    int i=0;
    for(auto iter = listHe.begin(); iter != listHe.end(); ++iter,i++){
        Dcel::HalfEdge* listHalfedge = *iter;

        Dcel::HalfEdge* he1 = dcel->addHalfEdge();
        Dcel::HalfEdge* he2 = dcel->addHalfEdge();
        Dcel::HalfEdge* he3 = dcel->addHalfEdge();

        Dcel::Vertex* v1 = listHalfedge->getToVertex();
        Dcel::Vertex* v2 = listHalfedge->getFromVertex();

        //current face creation
        Dcel::Face* cf = dcel->addFace();
        cf->setOuterHalfEdge(he1);
        newFace[i]=cf;


        he1->setFromVertex(v1);
        he1->setToVertex(v2);
        he1->setNext(he2);
        he1->setPrev(he3);
        v1->setIncidentHalfEdge(he1);
        v1->incrementCardinality();
        v2->incrementCardinality();
        he1->setFace(cf);


        he2->setFromVertex(v2);
        he2->setToVertex(v3);
        he2->setNext(he3);
        he2->setPrev(he1);
        v2->setIncidentHalfEdge(he2);
        v2->incrementCardinality();
        v3->incrementCardinality();
        he2->setFace(cf);



        he3->setFromVertex(v3);
        he3->setToVertex(v1);
        he3->setNext(he1);
        he3->setPrev(he2);
        v3->setIncidentHalfEdge(he3);
        v3->incrementCardinality();
        v1->incrementCardinality();
        he3->setFace(cf);

    }
    return newFace;
}


void convexhullCreator::findVertices(){
    Dcel::VertexIterator iter;
    int i=0;
    for(iter = dcel->vertexBegin(); iter != dcel-> vertexEnd(); ++iter){
        this->vectorPoint[i]=(*iter)->getCoordinate();
        i++;
    }
}
