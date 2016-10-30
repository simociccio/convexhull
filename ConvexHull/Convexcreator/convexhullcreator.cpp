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
/**
 * @brief convexhullcreator::~convexhullCreator()
 * distuttore classe convexhullCreator
 */
convexhullCreator::~convexhullCreator(){

}
/**
 * @brief convexhullcreator::hullcreator()
 * core del programma. qui viene eseguito tuttol l'algoritmo del convexhull.
 */
void convexhullCreator::hullcreator(){
    findVertices();

    randomfour();

    this->dcel->reset();

    tethraCreation();


    conflictgraph confg = conflictgraph(this->dcel,this->vectorPoint);

    confg.createGraph();

    for(int i=4; i<npoints; i++){
        std::list<Dcel::HalfEdge*> sortHorizon;
        std::set<Dcel::Face*>* setFace=confg.isVisibleByV(vectorPoint[i]);

        if(setFace->size()>0){

            sortHorizon=setHorizon(setFace);
            std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> visibleVertex=confg.visibleVertexToTest(sortHorizon);
            Dcel::Vertex* currentV = dcel->addVertex(vectorPoint[i]->getCoordinate());
            std::deque<Dcel::Face*> newFace = addNewFace(sortHorizon,currentV);
            confg.removeFaces(setFace);
            delFacesVisib(setFace);
            //aggiorno il CG
            uint j=0;
            for(auto iter=sortHorizon.begin();iter!=sortHorizon.end();++iter,j++){
                std::set<Dcel::Vertex*>* vertexToIns=visibleVertex[*iter];
                confg.updateCg(vertexToIns,newFace[j]);
                delete visibleVertex[*iter];

            }

       }

        confg.removeVertex(vectorPoint[i]);
        delete vectorPoint[i];


    }
}

void convexhullCreator::hullcreator(MainWindow* mainWindow){
    findVertices();

    randomfour();

    this->dcel->reset();

    tethraCreation();

    conflictgraph confg = conflictgraph(this->dcel,this->vectorPoint);

    confg.createGraph();

    for(int i=4; i<npoints; i++){
        std::list<Dcel::HalfEdge*> sortHorizon;
        std::set<Dcel::Face*>* setFace=confg.isVisibleByV(vectorPoint[i]);

        if(setFace->size()>0){

            sortHorizon=setHorizon(setFace);

            std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> visibleVertex=confg.visibleVertexToTest(sortHorizon);

            Dcel::Vertex* currentV = dcel->addVertex(vectorPoint[i]->getCoordinate());

            std::deque<Dcel::Face*> newFace = addNewFace(sortHorizon,currentV);
            confg.removeFaces(setFace);
            delFacesVisib(setFace);
            //aggiorno il CG
            uint j=0;
            for(auto iter=sortHorizon.begin();iter!=sortHorizon.end();++iter,j++){
                std::set<Dcel::Vertex*>* vertexToIns=visibleVertex[*iter];
                confg.updateCg(vertexToIns,newFace[j]);
                delete visibleVertex[*iter];

            }

            this->dcel->update();
            mainWindow->updateGlCanvas();


       }

        confg.removeVertex(vectorPoint[i]);
        delete vectorPoint[i];

    }

}

/**
 * @brief convexhullcreator::circleControll()
 * controllo la direzione della norma in maniera tale da poter determinare che senso dovrà avere il ciclo degli halfedge
 */

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
    bool pos = det > std::numeric_limits<double>::epsilon();
    if(pos){
        return false;
    }else{
        return true;
    }

}
/**
 * @brief convexhullcreator::randomfour()
 * prende random 4 vertici controllandone la coplanarità.se i vertici non sono soplanari la funzione è ricorsiva e ripete da capo la generazione dei vettori
 */

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
/**
 * @brief convexhullcreator::tethraCreation()
 * creazione del tetraedro
 */
void convexhullCreator::tethraCreation(){

    //aggiungo i primi 4 vertici
    Dcel::Vertex* v1 = this->dcel->addVertex(this->vectorPoint[0]->getCoordinate());
    Dcel::Vertex* v2 = this->dcel->addVertex(this->vectorPoint[1]->getCoordinate());
    Dcel::Vertex* v3 = this->dcel->addVertex(this->vectorPoint[2]->getCoordinate());
    Dcel::Vertex* v4 = this->dcel->addVertex(this->vectorPoint[3]->getCoordinate());

    //genero i primi 3 halfedge
    Dcel::HalfEdge* he1 = dcel->addHalfEdge();
    Dcel::HalfEdge* he2 = dcel->addHalfEdge();
    Dcel::HalfEdge* he3 = dcel->addHalfEdge();



    //creo la prima faccia
    Dcel::Face* f1 = dcel->addFace();
    f1->setOuterHalfEdge(he1);

    //controllo la norma per settare il senso di percorrenza della faccia
    if(circleControll()){
        he1->setFromVertex(v3);
        he1->setToVertex(v2);
        he1->setNext(he2);
        he1->setPrev(he3);
        v3->setIncidentHalfEdge(he1);
        v3->incrementCardinality();
        v2->incrementCardinality();
        he1->setFace(f1);


        he2->setFromVertex(v2);
        he2->setToVertex(v1);
        he2->setNext(he3);
        he2->setPrev(he1);
        v2->setIncidentHalfEdge(he2);
        v2->incrementCardinality();
        v1->incrementCardinality();
        he2->setFace(f1);

        he3->setFromVertex(v1);
        he3->setToVertex(v3);
        he3->setNext(he1);
        he3->setPrev(he2);
        v1->setIncidentHalfEdge(he3);
        v1->incrementCardinality();
        v3->incrementCardinality();
        he3->setFace(f1);
    }else{
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
        }
        std::list<Dcel::HalfEdge*> listHe;

        //aggiungo gli he alla lista
        listHe.push_back(he1);
        listHe.push_back(he2);
        listHe.push_back(he3);

        //creo la faccia
        addNewFace(listHe,v4);

}
/**
 * @brief convexhullcreator::addNewFace()
 * prende in ingresso la prima volta gli he del triangolo in seguito la lista degli he dell'orizzonte, il secondo parametro
 * il nuovo vettore. Con questa funzione si creano le nuove facce tra il vettore pasato in ingresso e gli he dell'orizzonte
 * @param listhe,vi
 * @return deque delle facce create
 */

std::deque<Dcel::Face*> convexhullCreator::addNewFace(std::list<Dcel::HalfEdge*>listHe,Dcel::Vertex* vi){

    std::deque<Dcel::HalfEdge*> hen = std::deque<Dcel::HalfEdge*>(listHe.size());
    std::deque<Dcel::HalfEdge*> hex = std::deque<Dcel::HalfEdge*>(listHe.size());
    std::deque<Dcel::Face*> faceDeq;
    int i=0;
    for(auto iter = listHe.begin(); iter != listHe.end(); ++iter,i++){

        //uso l'he puntato da iter per salvarmi il vertice di partenza e arrivo
        Dcel::Vertex* v1 = (*iter)->getFromVertex();
        Dcel::Vertex* v2 = (*iter)->getToVertex();


        Dcel::HalfEdge* he1 = dcel->addHalfEdge();
        Dcel::HalfEdge* he2 = dcel->addHalfEdge();
        Dcel::HalfEdge* he3 = dcel->addHalfEdge();

        //creazione faccia
        Dcel::Face* cf = dcel->addFace();
        faceDeq.push_back(cf);
        cf->setOuterHalfEdge(he1);
        he1->setFromVertex(v2);
        he1->setToVertex(v1);
        he1->setNext(he2);
        he1->setPrev(he3);
        he1->setTwin(*iter);
        (*iter)->setTwin(he1);
        v2->setIncidentHalfEdge(he1);
        v2->incrementCardinality();
        v1->incrementCardinality();
        he1->setFace(cf);

        he2->setFromVertex(v1);
        he2->setToVertex(vi);
        he2->setNext(he3);
        he2->setPrev(he1);
        v1->setIncidentHalfEdge(he2);
        v1->incrementCardinality();
        vi->incrementCardinality();
        he2->setFace(cf);

        he3->setFromVertex(vi);
        he3->setToVertex(v2);
        he3->setNext(he1);
        he3->setPrev(he2);
        vi->setIncidentHalfEdge(he3);
        vi->incrementCardinality();
        v2->incrementCardinality();
        he3->setFace(cf);
        hen[i]=he3;
        hex[i]=he2;
        }

    setTwins(hen,hex,i);
    return faceDeq;

}
/**
 * @brief convexhullcreator::setHorizon()
 * prende in ingresso un set di facce e scorrendo gli he della faccia scopre se il twin di tae halfedge si trova o meno in una faccia presente nel set.
 * Nel caso non è presente in una faccia del set vuol dire che l'he fa parte dell'orizzonte
 * @param faceList
 * @return lista di halfedge
 */

std::list<Dcel::HalfEdge*> convexhullCreator::setHorizon(std::set<Dcel::Face*> *faceList)const{
    Dcel::HalfEdge* twin,*outerHe;
    std::list<Dcel::HalfEdge*> hev;
    for(auto iter=faceList->begin();iter!=faceList->end();++iter){
        outerHe=(*iter)->getOuterHalfEdge();
        twin=outerHe->getTwin();
        auto iter1=outerHe;
        do{
            if(faceList->count(twin->getFace())<1){
                hev.push_back(twin);
             }
            outerHe = outerHe->getNext();
            twin=outerHe->getTwin();

        }while(iter1!=outerHe);

    }
    return horizonSort(hev);


}
/**
 * @brief convexhullcreator::horizonSort()
 * utilizzando 'orizzonte non hordinato della funzione qui sopra lo ordino in maniera tale che il tovertex del primo coincida con il fromvertex del secondo e così via
 * @param heList
 * @return lista di halfedge
 */
std::list<Dcel::HalfEdge*> convexhullCreator::horizonSort(std::list<Dcel::HalfEdge *> he) const{
    Dcel::Vertex* fromCurrentHe;

    std::list<Dcel::HalfEdge*> sortedList;
    int i =1;
    for(auto iter=he.begin(); iter!=he.end(); ++iter,i++){

        if(i==1){
            fromCurrentHe=(*iter)->getToVertex();
            sortedList.push_back(*iter);
            he.remove(*iter);
        }
        else{
            auto lastEl = sortedList.rbegin();
            fromCurrentHe=(*lastEl)->getToVertex();;
            he.remove(*lastEl);
            if(he.size()==0){
                return sortedList;
            }
        }
        for(auto iter1=he.begin(); iter1!=he.end(); ++iter1){
          if(fromCurrentHe==(*iter1)->getFromVertex()){
              sortedList.push_back(*iter1);
          }

        }
    }
    return sortedList;
}
/**
 * @brief convexhullcreator::setTwins()
 * questa funzione setta i twin in maniera tale che scorrendo il for i twin seguano il ciclo e uscendo dal for concludo il ciclo in
 * modo che l'ultimo he punti al primo
 * @param vector di he in ingresso, vector di he in uscita, intero
 */
void convexhullCreator::setTwins(std::deque<Dcel::HalfEdge*> hen,std::deque<Dcel::HalfEdge*> hex,int i){
    int j;
    for(j=0;j<i-1;j++){
        hen[j]->setTwin(hex[j+1]);
        hex[j+1]->setTwin(hen[j]);
       }
    hen[i-1]->setTwin(hex[0]);
    hex[0]->setTwin(hen[i-1]);

}
/**
 * @brief convexhullcreator::findVertices()
 * prendo tutti i vertici dalla dcel
 * copio i punti in un altro vettore per evitare che i puntatori prendano vertici che non gli spettano
 */
void convexhullCreator::findVertices(){
    auto vectIt = vectorPoint.begin();
        for(auto vit = dcel->vertexBegin(); vit != dcel->vertexEnd(); ++vit,++vectIt){
            *vectIt = new Dcel::Vertex(**vit);
        }
}

void convexhullCreator::delFacesVisib(std::set<Dcel::Face*>* setFace){
    for(auto iter=setFace->begin();iter!=setFace->end();++iter){
        for(auto iterh=(*iter)->incidentHalfEdgeBegin(); iterh!=(*iter)->incidentHalfEdgeEnd();++iterh){

            //devo prendere i vertici in modo da poterne controllare la cardinalità per ogni singolo HE della faccia

            Dcel::Vertex* getFrom = (*iterh)->getFromVertex();
            Dcel::Vertex* getTo = (*iterh)->getToVertex();

            this->dcel->deleteHalfEdge(*iterh);
            getFrom->decrementCardinality();
            getTo->decrementCardinality();

            if(getFrom->getCardinality()<1){
                this->dcel->deleteVertex(getFrom);
            }
            if(getTo->getCardinality()<1){
                this->dcel->deleteVertex(getTo);
            }
        }
         //elimino la faccia
         this -> dcel -> deleteFace(*iter);
    }
}

