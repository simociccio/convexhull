#include "conflictgraph.h"
#include "convexhullcreator.h"

conflictgraph::conflictgraph(DrawableDcel *dcel, std::vector<Dcel::Vertex*> &vectorPoint)
{
    this->dcel=dcel;
    this->vectorPoint = vectorPoint;


}
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

        for(unsigned int i=4;i<vectorPoint.size();i++){

            matrix(3, 0) = vectorPoint[i]->getCoordinate().x();
            matrix(3, 1) = vectorPoint[i]->getCoordinate().y();
            matrix(3, 2) = vectorPoint[i]->getCoordinate().z();
            matrix(3, 3) = 1;
            double det = matrix.determinant();
            if(det < -std::numeric_limits<double>::epsilon()){

                addFaceToVertexList(this->vectorPoint[i],(*iter));
                addVertexToFaceList((*iter),this->vectorPoint[i]);
            }


        }
    }

}
void conflictgraph::addFaceToVertexList(Dcel::Vertex* vi, Dcel::Face* face){

    bool is_in = maptoface.find(vi) != maptoface.end();
    if(is_in){
        std::set<Dcel::Face*>* listf= maptoface[vi];
        listf->insert(face);
    }
    else{
        std::set<Dcel::Face*>* listf= new std::set<Dcel::Face*>();
        listf->insert(face);
        this->maptoface[vi]=listf;
    }

}
void conflictgraph::addVertexToFaceList(Dcel::Face* face, Dcel::Vertex* vi){

    bool is_in = maptover.find(face) != maptover.end();
    if(is_in){
        std::set<Dcel::Vertex*>* listv= maptover[face];
        listv->insert(vi);
    }
    else{
        std::set<Dcel::Vertex*>* listv= new std::set<Dcel::Vertex*>();
        listv->insert(vi);
        maptover[face]=listv;
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

void conflictgraph::removeVertex(std::set<Dcel::Vertex*> *vertex){

    for(auto iter = vertex->begin(); iter!= vertex->end(); ++iter){
        std::set<Dcel::Face*> *faceToRemove = isVisibleByV(*iter);
        for(auto iter2 = faceToRemove->begin(); iter2!= faceToRemove->end(); ++iter2){
            maptover[*iter2]->erase(*iter);
        }
        maptoface.erase(*iter);
    }

}

void conflictgraph::removeFaces(Dcel::Face* face){
    std::set<Dcel::Vertex*>* verticesToRemove = isVisibleByF(face);
    for(auto iter2 = verticesToRemove->begin(); iter2!= verticesToRemove->end(); ++iter2){
        maptoface[*iter2]->erase(face);
    }
    maptover.erase(face);
}
void conflictgraph::updateCg(std::set<Dcel::Vertex*> *vertex,Dcel::Face* face){

Eigen::Matrix4d matrix;
    for(auto iter=vertex->begin();iter!=vertex->end();++iter){
        int i;
        for(auto vertexit = *(face->incidentVertexBegin());vertexit!=*(face->incidentVertexEnd());++vertexit,i++){

            matrix(i, 0) = (*vertexit).getCoordinate().x();
            matrix(i, 1) = (*vertexit).getCoordinate().y();
            matrix(i, 2) = (*vertexit).getCoordinate().z();
            matrix(i, 3) = 1;


        }

        for(unsigned int i=4;i<vectorPoint.size();i++){

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
}




