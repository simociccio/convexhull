#include "convexhullcreator.h"
#include "conflictgraph.h"
#include <stdlib.h>
#include <iostream>
#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>

convexhullCreator::convexhullCreator(DrawableDcel* dcel){
   this->dcel=dcel;
   this->npoints=dcel->getNumberVertices();
   this->vectorPoint=std::vector<Dcel::Vertex*>(npoints);

}
void convexhullCreator::hullcreator(){
    findVertices();
    randomfour();
    this->dcel->reset();
    tethraCreation();
    std::cout<<"debug";
    conflictgraph confg = conflictgraph(this->dcel,this->vectorPoint);
    confg.createGraph();
    std::cout<<"debug";
    for(int i=4; i<npoints; i++){
        std::list<Dcel::HalfEdge*> listHorizon;
        Dcel::Vertex* point = vectorPoint[i];

        std::set<Dcel::Face*>* isVisiblebyF=confg.isVisibleByF(point);

        if(isVisiblebyF->size()>0){

        }


    }

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
              matrix(i, 0) = vectorPoint[i]->getCoordinate().x();
              matrix(i, 1) = vectorPoint[i]->getCoordinate().y();
              matrix(i, 2) = vectorPoint[i]->getCoordinate().z();
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
        matrix(i, 0) = this->vectorPoint[i]->getCoordinate().x();
        matrix(i, 1) = this->vectorPoint[i]->getCoordinate().y();
        matrix(i, 2) = this->vectorPoint[i]->getCoordinate().z();
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
    std::list<Dcel::Face*> face;

    //add first 4 points to the dcel
    Dcel::Vertex* v1 = this->dcel->addVertex(this->vectorPoint[0]->getCoordinate());
    Dcel::Vertex* v2 = this->dcel->addVertex(this->vectorPoint[1]->getCoordinate());
    Dcel::Vertex* v3 = this->dcel->addVertex(this->vectorPoint[2]->getCoordinate());
    Dcel::Vertex* v4 = this->dcel->addVertex(this->vectorPoint[3]->getCoordinate());

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

std::list<Dcel::Face*> convexhullCreator::addNewFace(std::list<Dcel::HalfEdge*>listHe,Dcel::Vertex* vi){

    std::vector<Dcel::HalfEdge*> hen = std::vector<Dcel::HalfEdge*>(listHe.size());
    std::vector<Dcel::HalfEdge*> hex = std::vector<Dcel::HalfEdge*>(listHe.size());
    std::list<Dcel::Face*> Face;
    int i=0;
    for(auto iter = listHe.begin(); iter != listHe.end(); ++iter,i++){

        Dcel::HalfEdge* he1 = dcel->addHalfEdge();
        Dcel::HalfEdge* he2 = dcel->addHalfEdge();
        Dcel::HalfEdge* he3 = dcel->addHalfEdge();

        //uso l'he puntato da iter per salvarmi il vertice di partenza e arrivo
        Dcel::Vertex* v1 = (*iter)->getToVertex();
        Dcel::Vertex* v2 = (*iter)->getFromVertex();

        //creazione faccia
        Dcel::Face* cf = dcel->addFace();
        Face.push_back(cf);
        cf->setOuterHalfEdge(he1);
        he1->setFromVertex(v1);
        he1->setToVertex(v2);
        he1->setNext(he2);
        he1->setPrev(he3);
        he1->setTwin(*iter);
        (*iter)->setTwin(he1);
        v1->setIncidentHalfEdge(he1);
        v1->incrementCardinality();
        v2->incrementCardinality();
        he1->setFace(cf);

        he2->setFromVertex(v2);
        he2->setToVertex(vi);
        he2->setNext(he3);
        he2->setPrev(he1);
        v2->setIncidentHalfEdge(he2);
        v2->incrementCardinality();
        vi->incrementCardinality();
        he2->setFace(cf);

        he3->setFromVertex(vi);
        he3->setToVertex(v1);
        he3->setNext(he1);
        he3->setPrev(he2);
        vi->setIncidentHalfEdge(he3);
        vi->incrementCardinality();
        v1->incrementCardinality();
        he3->setFace(cf);
        hen[i]=he3;
        hex[i]=he2;
        }

    setTwins(hen,hex,i);
    return Face;

}
std::set<Dcel::HalfEdge*> convexhullCreator::setHorizon(std::set<Dcel::Face*> *face){
    Dcel::HalfEdge* twin;
    std::list<Dcel::HalfEdge*> hev;
    std::list<Dcel::HalfEdge*> heSorted;
    for(auto iter=face->begin();iter!=face->end();++iter){
        twin=(*iter)->getOuterHalfEdge()->getTwin();
        for(int i =0 ; i<3 ; i++){
            if(face->count(twin->getFace())){
                hev.push_back(twin);
                break;
             }
            twin=(*iter)->getOuterHalfEdge()->getNext()->getTwin();
        }

    }
    return horizonSort(hev);


}
std::set<Dcel::HalfEdge*> convexhullCreator::horizonSort(std::list<Dcel::HalfEdge *> he){
    Dcel::Vertex* fromCurrentHe;

    std::set<Dcel::HalfEdge*> sortedSet;

    for(auto iter=he.begin(); iter!=he.end(); ++iter){

        if(iter==he.begin()){
            fromCurrentHe=(*iter)->getToVertex();
            sortedSet.insert(*iter);
            he.remove(*iter);
        }
        else{
            auto lastEl = sortedSet.rbegin();
            fromCurrentHe=(*lastEl)->getToVertex();;
            he.remove(*lastEl);
            if(he.size()==1){
                sortedSet.insert(*lastEl);
                return sortedSet;
            }
        }
        for(auto iter1=he.begin(); iter1!=he.end(); ++iter1){
          if(fromCurrentHe==(*iter1)->getFromVertex()){
              sortedSet.insert(*iter1);
          }

        }
    }
    return sortedSet;
}
//questa funzione setta i twin in maniera tale che scorrendo il for i twin seguano il ciclo e uscendo dal for concludo il ciclo in
//modo che l'ultimo he punti al primo
void convexhullCreator::setTwins(std::vector<Dcel::HalfEdge*> hen,std::vector<Dcel::HalfEdge*> hex,int i){
    int j;
    for(j=0;j<i-1;j++){
        hen[j]->setTwin(hex[j+1]);
        hex[j+1]->setTwin(hen[j]);
       }
    hen[i-1]->setTwin(hex[0]);
    hex[0]->setTwin(hen[i-1]);

}

void convexhullCreator::findVertices(){
    Dcel::VertexIterator iter;
    int i=0;
    for(iter = dcel->vertexBegin(); iter != dcel-> vertexEnd(); ++iter){
        this->vectorPoint[i]=(*iter);
        i++;
    }
}
