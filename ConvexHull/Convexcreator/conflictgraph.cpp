#include "conflictgraph.h"
#include "convexhullcreator.h"

conflictgraph::conflictgraph(DrawableDcel *dcel, std::vector<Dcel::Vertex*> &vectorPoint)
{

    this->dcel=dcel;
    this->vectorPoint = vectorPoint;
    this->npoints=vectorPoint.size();

}
void conflictgraph::createGraph(){
    Eigen::Matrix4d matrix;


    for(auto iter=dcel->faceBegin(); iter!=dcel->faceEnd();++iter){

        int i=0;

        for(auto vertexit = (*iter)->incidentVertexBegin();i<3;++vertexit,i++){

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
    std::cout<<"debug";


}
void conflictgraph::addFaceToVertexList(Dcel::Vertex* vi, Dcel::Face* face){


    if(maptoface.find(vi) != maptoface.end()){

        std::set<Dcel::Face*>* setf= maptoface[vi];
        setf->insert(face);
    }
    else{
        std::set<Dcel::Face*>* setf= new std::set<Dcel::Face*>();
        setf->insert(face);
        maptoface[vi]=setf;
    }

}
void conflictgraph::addVertexToFaceList(Dcel::Face* face, Dcel::Vertex* vi){

    if(maptover.find(face) != maptover.end()){
        std::set<Dcel::Vertex*>* setv= maptover[face];
        setv->insert(vi);
    }
    else{
        std::set<Dcel::Vertex*>* setv= new std::set<Dcel::Vertex*>();
        setv->insert(vi);
        maptover[face]=setv;
    }

}

std::set<Dcel::Vertex*>* conflictgraph::isVisibleByF(Dcel::Face *face){

    if(maptover.find(face)!=maptover.end()){
            return new std::set<Dcel::Vertex*>(*maptover.at(face));
        }
        else{
            return new std::set<Dcel::Vertex*>();
        }
}

std::set<Dcel::Face*>* conflictgraph::isVisibleByV(Dcel::Vertex *vertex){

        if(maptoface.find(vertex)!=maptoface.end()){
            return new std::set<Dcel::Face*>(*maptoface.at(vertex));
        }
        else{
            return new std::set<Dcel::Face*>();
        }
}

void conflictgraph::removeVertex(Dcel::Vertex* v){

    std::set<Dcel::Face*> *faceToR = maptoface[v];
    if(faceToR!=nullptr){
        maptoface.erase(v);
        for(auto iter= faceToR->begin();iter!=faceToR->end();++iter){
            auto associatedSet = this->maptover[*iter];
            if(associatedSet != nullptr){
               associatedSet->erase(v);
            }
        }
    }

}

void conflictgraph::removeFaces(std::set<Dcel::Face*> *setFaces){

    for(auto vit1 = setFaces->begin(); vit1 != setFaces->end(); ++vit1){
            std::set<Dcel::Vertex*> *visiblePoints = maptover[*vit1];

            //if the conflict list of the face exist,
            //delete it from the map
            if(visiblePoints != nullptr){
                maptover.erase(*vit1);

                //we have to delete the face from the conflict list of each vertex
                //we iterate only on the vertices that see the face
                for(auto vit2 = visiblePoints->begin(); vit2 != visiblePoints->end(); ++vit2){

                    auto associatedSet = maptoface[*vit2];
                    if(associatedSet != nullptr){
                        associatedSet->erase(*vit1);
                    }
                }
            }
        }
}

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





