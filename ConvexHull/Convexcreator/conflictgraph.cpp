#include "conflictgraph.h"
#include "convexhullcreator.h"

conflictgraph::conflictgraph(DrawableDcel *dcel, std::vector<Dcel::Vertex*> &vectorPoint)
{

    this->dcel=dcel;
    this->vectorPoint = vectorPoint;
    this->npoints=vectorPoint.size();

}
/**
 * @brief conflictgraph::~conflictgraph()
 * questo è il distruttore ella classe che viene
 * richiamato una volta che la classe non viene
 * piu' utilizzata
 */
conflictgraph::~conflictgraph(){

}
/**
 * @brief conflictgraph::createGraph()
 * questo metodo permette la creazione del
 * grafo inserendo i vertici e le facce nelle 2 mappe
 */

void conflictgraph::createGraph(){
    Eigen::Matrix4d matrix;


    for(auto iter=dcel->faceBegin(); iter!=dcel->faceEnd();++iter){

        int i=0;

        for(auto vertexit = (*iter)->incidentVertexBegin();vertexit!=(*iter)->incidentVertexEnd();++vertexit,i++){

            matrix(i, 0) = (*vertexit)->getCoordinate().x();
            matrix(i, 1) = (*vertexit)->getCoordinate().y();
            matrix(i, 2) = (*vertexit)->getCoordinate().z();
            matrix(i, 3) = 1;


        }

        for(int i=4;i<npoints;i++){

            matrix(3, 0) = vectorPoint[i]->getCoordinate().x();
            matrix(3, 1) = vectorPoint[i]->getCoordinate().y();
            matrix(3, 2) = vectorPoint[i]->getCoordinate().z();
            matrix(3, 3) = 1;
            double det = matrix.determinant();
            if(det < -std::numeric_limits<double>::epsilon()){

                addFaceToVertexList(vectorPoint[i],(*iter));
                addVertexToFaceList((*iter),vectorPoint[i]);
            }


        }
    }


}
/**
 * @brief conflictgraph::addFaceToVertexList()
 * aggiunge la faccia in corrispondenza di un determinato vertice
 * nella mappa
 * @param vi, vertice da usare come chiave nella mappa
 *        face, faccia da aggiungere
 */
void conflictgraph::addFaceToVertexList(Dcel::Vertex* vi, Dcel::Face* face){

    std::set<Dcel::Face*>* setf;

    if(maptoface.find(vi) != maptoface.end()){

        setf= maptoface[vi];
        setf->insert(face);
    }
    else{
        setf= new std::set<Dcel::Face*>();
        setf->insert(face);
        maptoface[vi]=setf;

    }

}
/**
 * @brief conflictgraph::addVertexToFaceList()
 * aggiunge il vertice in corrispondenza di un determinata faccia
 * nella mappa
 * @param vi, vertice da inserire
 *        face, faccia da usare come chiave nella mappa
 */

void conflictgraph::addVertexToFaceList(Dcel::Face* face, Dcel::Vertex* vi){

    std::set<Dcel::Vertex*>* setv;
    if(maptover.find(face) != maptover.end()){
        setv= maptover[face];
        setv->insert(vi);
    }
    else{
        setv= new std::set<Dcel::Vertex*>();
        setv->insert(vi);
        maptover[face]=setv;
    }

}
/**
 * @brief conflictgraph::isVisibleByF()
 * mi restituisce un set con i vertici visibili dalla faccia che gli passo in input
 * @param face
 */

std::set<Dcel::Vertex*>* conflictgraph::isVisibleByF(Dcel::Face *face)const{


    if(maptover.find(face)!=maptover.end()){
         auto ver=  new std::set<Dcel::Vertex*>(*maptover.at(face));
            return ver;
        }
        else{
         auto ver=  new std::set<Dcel::Vertex*>();
            return ver;
        }
}
/**
 * @brief conflictgraph::isVisibleByV()
 * mi restituisce un set con le facce visibili dal vertice che gli passo in input
 * @param vertex
 */

std::set<Dcel::Face*>* conflictgraph::isVisibleByV(Dcel::Vertex *vertex)const{

        if(maptoface.find(vertex)!=maptoface.end()){
            return new std::set<Dcel::Face*>(*maptoface.at(vertex));
        }
        else{
            return new std::set<Dcel::Face*>();
        }
}

/**
 * @brief conflictgraph::removeVertex()
 * rimuove il vertice dalla mappa dei vertici e elimina tutte le occorrenze di quel vertice nella mappa delle facce
 * @param v
 */

void conflictgraph::removeVertex(Dcel::Vertex* v){
    std::set<Dcel::Face*> *faceToR = isVisibleByV(v);
    if(faceToR!=nullptr){
        maptoface.erase(v);
        for(auto iter= faceToR->begin();iter!=faceToR->end();++iter){
            if(isVisibleByF(*iter) != nullptr){
               isVisibleByF(*iter)->erase(v);
            }
        }
    }

}

/**
 * @brief conflictgraph::removeFaces()
 * rimuove le facce dalla mappa e tutte le occorrenze della facce nella mappa dei vertici
 * @param setFaces
 */

void conflictgraph::removeFaces(std::set<Dcel::Face*> *setFaces){

    for(auto vit1 = setFaces->begin(); vit1 != setFaces->end(); ++vit1){
            std::set<Dcel::Vertex*> *visibleV = isVisibleByF(*vit1);
            if(visibleV!=nullptr){
                maptover.erase(*vit1);

                for(auto vit2 = visibleV->begin(); vit2 != visibleV->end(); ++vit2){
                    auto mapOfElem = maptoface[*vit2];
                    if(mapOfElem != nullptr){
                        mapOfElem->erase(*vit1);
                    }
                }
            }
        }
}
/**
 * @brief conflictgraph::visibleVertexToTest()
 * scorrendo l'orizzonte posso vedere tutti i vertici visibili si dalla faccia
 * dell'halfedge che dalla faccia del twin. Dopo aver fatto questo li inserisco in una mappa
 * che mi permette di avere per ogni halfedge il set di vertci visibili da esso
 * @param listHoriz, la lista di halfedge dell'orizzonte
 * @return mapHV: mappa halfedge set di vertici
 */

std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> conflictgraph::visibleVertexToTest(std::list<Dcel::HalfEdge*> listHoriz){
    std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> mapHV;
    for(auto iter = listHoriz.begin();iter!=listHoriz.end();++iter){
        std::set<Dcel::Vertex*> *confv=isVisibleByF((*iter)->getFace());
        std::set<Dcel::Vertex*> *confvt=isVisibleByF((*iter)->getTwin()->getFace());
        confv->insert(confvt->begin(),confvt->end());
        mapHV[*iter]=confv;
    }
    return mapHV;

}
/**
 * @brief conflictgraph::visibleVertexToTest()
 * scopro i vertici visibili in modo da poterli aggiungere nel conflictgraph
 * @param vertex, set di vertici
 *        face
 */


void conflictgraph::updateCg(std::set<Dcel::Vertex*> *vertex,Dcel::Face* face){

Eigen::Matrix4d matrix;
    for(auto iter=vertex->begin();iter!=vertex->end();++iter){
        int i=0;
        for(auto vertexit = face->incidentVertexBegin();vertexit!=face->incidentVertexEnd();++vertexit,i++){
            matrix(i, 0) = (*vertexit)->getCoordinate().x();
            matrix(i, 1) = (*vertexit)->getCoordinate().y();
            matrix(i, 2) = (*vertexit)->getCoordinate().z();
            matrix(i, 3) = 1;
        }
        matrix(3, 0) = (*iter)->getCoordinate().x();
        matrix(3, 1) = (*iter)->getCoordinate().y();
        matrix(3, 2) = (*iter)->getCoordinate().z();
        matrix(3, 3) = 1;
        double det = matrix.determinant();
        if(det < -std::numeric_limits<double>::epsilon()){
            addFaceToVertexList((*iter),face);
            addVertexToFaceList(face,(*iter));
        }

        }
}





