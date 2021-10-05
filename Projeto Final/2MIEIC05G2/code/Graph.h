/*
 * Graph.h
 */
#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <queue>
#include <list>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <map>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "MutablePriorityQueue.h"


template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

template <class T>
class Vertex {
    T info;						// content of the vertex
    std::vector<Edge<T> > adj;		// outgoing edges

    double dist = 0;
    Vertex<T> *path = NULL;
    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

    Vertex<T>* parent = NULL;
    int low = 0;
    int num = 0;

    void addEdge(int id,Vertex<T> *dest, double w);

public:
    Vertex(T in);
    T getInfo() const;
    double getDist() const;
    Vertex *getPath() const;
    double getEdgeDistance(Vertex<T>* dest);
    Edge<T> getEdge(Vertex<T>* dest);

    bool operator<(Vertex<T> & vertex) const; // // required by MutablePriorityQueue
    friend class Graph<T>;
    friend class MutablePriorityQueue<Vertex<T>>;
};


template <class T>
Vertex<T>::Vertex(T in): info(in) {}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
void Vertex<T>::addEdge(int id, Vertex<T> *d, double w) {
    adj.push_back(Edge<T>(id, d, w));
}

template <class T>
double Vertex<T>::getEdgeDistance(Vertex<T>* dest) {
    for(Edge<T> edge: this->adj){
        if(edge.getDest() == dest){
            return edge.getWeight();
        }
    }
    return 0;
}

template <class T>
Edge<T> Vertex<T>::getEdge(Vertex<T>* dest) {
    for(Edge<T> edge: this->adj){
        if(edge.getDest() == dest){
            return edge;
        }
    }
    return Edge<T>(0, nullptr,0);
}


template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template <class T>
Vertex<T> *Vertex<T>::getPath() const {
    return this->path;
}

/********************** Edge  ****************************/

template <class T>
class Edge {
    int id;
    Vertex<T> * dest;      // destination vertex
    double weight;         // edge weight
public:
    Edge(int id,Vertex<T> *d, double w);

    Vertex<T> *getDest() const;

    double getWeight() const;

    int getId() const;

    void setId(int id);

    friend class Graph<T>;
    friend class Vertex<T>;

};

template <class T>
Edge<T>::Edge(int id,Vertex<T> *d, double w): id(id),dest(d), weight(w) {}


/*************************** Graph  **************************/

template <class T>
class Graph {
    std::vector<Vertex<T> *> vertexSet;    // vertex set
    std::vector<std::vector<double>> distMin;
    std::vector<std::vector<Vertex<T>*>> predecessores;
    std::map< std::pair<std::string , std::string>, std::pair<Vertex<T>*, double> > dijkstraMemoization;
    int originNode = 0;
    int counter = 1;
public:
    Graph();
    Vertex<T> *findVertex(const T &in) const;
    bool addVertex(const T &in);
    bool addEdge(int id,const T &sourc, const T &dest, double w);
    int getNumVertex() const;
    std::vector<Vertex<T> *> getVertexSet() const;
    void setOriginNode(int originNode);
    T getOriginNode();

    void findArt(Vertex<T> *v);
    // Fp06 - single source
    void unweightedShortestPath(const T &s);    //TODO...
    void dijkstraShortestPath(const T &s);      //TODO...
    void bellmanFordShortestPath(const T &s);   //TODO...
    std::vector<T> getPath(const T &origin, const T &dest) const;   //TODO...

    std::pair<std::vector<T>, double> getDijsktraPath(const T& origin, const T& dest);

    // Fp06 - all pairs
    void floydWarshallShortestPath();   //TODO...
    std::pair<std::vector<T>, double> getfloydWarshallPath(const T &origin, const T &dest) const;   //TODO...

};

template <class T>
Graph<T>::Graph() {
    dijkstraMemoization = {};
}

template<class T>
Vertex<T> *Edge<T>::getDest() const {
    return dest;
}

template<class T>
double Edge<T>::getWeight() const {
    return weight;
}

template<class T>
int Edge<T>::getId() const {
    return id;
}

template<class T>
void Edge<T>::setId(int id) {
    Edge::id = id;
}

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet)
        if (v->info == in)
            return v;
    return NULL;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if ( findVertex(in) != NULL)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(int id,const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    v1->addEdge(id,v2,w);
    return true;
}


/**************** Single Source Shortest Path algorithms ************/

template<class T>
void Graph<T>::unweightedShortestPath(const T &orig) {
    for(Vertex<T>* vertex: vertexSet){
        vertex->visited = false;
    }

    std::queue<Vertex<T> *> vertex_queue;
    Vertex<T>* vertexSource = findVertex(orig);
    vertex_queue.push(vertexSource);
    vertexSource->visited = true;
    Vertex<T>* vertexCurrent;

    while(!vertex_queue.empty()){
        vertexCurrent = vertex_queue.front();
        vertex_queue.pop();

        for(Edge<T> edge: vertexCurrent->adj){
            if(!edge.dest->visited){
                vertex_queue.push(edge.dest);
                edge.dest->visited = true;
                edge.dest->dist = vertexCurrent->dist++;
                edge.dest->path = vertexCurrent;
            }
        }
    }

}

template <class T>
void Graph<T>::setOriginNode(int originNode) {
    this->originNode = originNode;
}

template <class T>
T Graph<T>::getOriginNode() {
    return this->vertexSet[originNode]->info;
}

template<class T>
void Graph<T>::dijkstraShortestPath(const T &origin) {
    for(Vertex<T>* vertex: getVertexSet()){
        vertex->dist = 9999;
        vertex->path = NULL;
    }
    Vertex<T>* vertexSource = findVertex(origin);
    vertexSource->dist = 0;
    vertexSource->path = NULL;

    MutablePriorityQueue<Vertex<T> > q;
    q.insert(vertexSource);
    Vertex<T>* vertexCurrent;
    while(!q.empty()){
        vertexCurrent = q.extractMin();
        for(Edge<T> edge: vertexCurrent->adj){
            if(edge.dest->dist > vertexCurrent->dist + edge.weight){
                edge.dest->dist = vertexCurrent->dist + edge.weight;
                edge.dest->path = vertexCurrent;
                auto const result = dijkstraMemoization.insert(std::make_pair(std::make_pair(origin.getId(), edge.dest->info.getId()), std::make_pair(vertexCurrent,edge.dest->dist)));
                if(not result.second) result.first->second = std::make_pair(vertexCurrent,edge.dest->dist);
                q.insert(edge.dest);
            }
        }
    }

}


template<class T>
void Graph<T>::bellmanFordShortestPath(const T &orig) {
    for(Vertex<T>* vertex: getVertexSet()){
        vertex->dist = 9999;
        vertex->path = NULL;
    }

    int temp_distance = 0;
    Vertex<T>* vertexSource = findVertex(orig);
    vertexSource->dist = 0;
    for(Vertex<T>* vertex: getVertexSet()){
        //iterate through every edge V times
        for(Vertex<T>* vertex1: getVertexSet()){
            for(Edge<T> edge: vertex1->adj){
                temp_distance = vertex1->dist + edge.weight;
                if(temp_distance < edge.dest->dist){
                    edge.dest->dist = temp_distance;
                    edge.dest->path = vertex1;
                }
            }
        }

    }

    //iterate through every edge again
    for(Vertex<T>* vertex1: getVertexSet()){
        for(Edge<T> edge: vertex1->adj){
            if(vertex1->dist + edge.weight < edge.dest->dist){
                std::cout << "Negative cycle exists!" << std::endl;
            }
        }
    }

}

template <class T>
void Graph<T>::findArt(Vertex<T> *v) {
    v->visited = true;
    v->low = counter++;
    v->num = v->low;
    Vertex<T>* nextVertex;
    for(Edge<T> edge: v->adj){
        nextVertex = edge.dest;
        if(!nextVertex->visited){
            nextVertex->parent = v;
            findArt(nextVertex);
            v->low = std::min(v->low,nextVertex->low);
            if(v->low >= v->num){
                std::cout << "Ponto de articulação vertex: " << v->info.getId() << std::endl;
            }
        }else{ //aresta de retorno
            if(v->parent != nextVertex){
                v->low = std::min(v->low,nextVertex->num);
            }
        }
    }
}


template<class T>
std::vector<T> Graph<T>::getPath(const T &origin, const T &dest) const{
    std::vector<T> res;
    T currInfo = dest;
    while(currInfo != origin){
        res.push_back(currInfo);
        currInfo = findVertex(currInfo)->path->info;
    }
    res.push_back(origin);
    std::reverse(res.begin(),res.end());
    return res;
}


template <class T>
std::pair<std::vector<T>, double> Graph<T>::getDijsktraPath(const T &origin, const T &dest)  {

    std::vector<T> res;
    std::pair<std::string, std::string> key = std::make_pair(origin.getId(), dest.getId());
    double dist = 0;
    if(dijkstraMemoization.find(key) == dijkstraMemoization.end() ){
        dijkstraShortestPath(origin);
    }
    int i = 0;
    Vertex<T>* originVert = findVertex(origin);
    while (dijkstraMemoization[key].first != originVert ) {
        res.emplace(res.begin(), dijkstraMemoization[key].first->info);
        if(i > 0){
            dist = dist + findVertex(res[i-1])->getEdgeDistance(dijkstraMemoization[key].first);
        }
        i++;
        for (int j = 0 ; j < getNumVertex() ; j++) {
            if (vertexSet[j]->info == dijkstraMemoization[key].first->info) {
                key.second = vertexSet[j]->info.getId();
                break;
            }
        }
    }

    res.push_back(dest);
    dist = dist + findVertex(res[i - 1])->getEdgeDistance(findVertex(dest));
    res.insert(res.begin(), origin);
    return std::make_pair(res,dist);
}


/**************** All Pairs Shortest Path  ***************/

template<class T>
void Graph<T>::floydWarshallShortestPath() {
    distMin.clear();
    predecessores.clear();                  // Aloca memória para as Matrizes
    distMin = std::vector<std::vector<double>>(getNumVertex(), std::vector<double>(getNumVertex(), INF));
    predecessores = std::vector<std::vector<Vertex<T>*>>(getNumVertex(), std::vector<Vertex<T>*>(getNumVertex(), NULL));

    for (int i = 0 ; i < getNumVertex() ; i++) {
        for (int j = 0 ; j < getNumVertex() ; j++) {
            if (i == j) // Elementos da diagonal da Matriz a 0
                distMin[i][j] = 0;
            else {
                for (auto edge : vertexSet[i]->adj) {
                    if (edge.dest->getInfo() == vertexSet[j]->info) {
                        distMin[i][j] = edge.weight;
                        predecessores[i][j] = vertexSet[i];
                    }
                }
            }
        }
    }


    for (int k = 0 ; k < getNumVertex() ; k++) {
        for (int l = 0 ; l < getNumVertex() ; l++) {
            for (int m = 0 ; m < getNumVertex() ; m++) {
                if (distMin[l][m] > distMin[l][k] + distMin[k][m]) {
                    distMin[l][m] = distMin[l][k] + distMin[k][m];
                    predecessores[l][m] = predecessores[k][m];
                }
            }

        }
    }

}


template<class T>
std::pair<std::vector<T>, double> Graph<T>::getfloydWarshallPath(const T &orig, const T &dest) const{
    std::vector<T> res;
    double dist;
    int indexOrigem, indexDestino;
    for (int i = 0 ; i < getNumVertex() ; i++) {
        if (vertexSet[i]->info == orig)
            indexOrigem = i;
        else if (vertexSet[i]-> info == dest)
            indexDestino = i;
    }
    int i = 0;
    while (predecessores[indexOrigem][indexDestino] != vertexSet[indexOrigem]) {
        res.emplace(res.begin(), predecessores[indexOrigem][indexDestino]->info);
        if(i > 0){
            dist = dist + findVertex(res[i-1])->getEdgeDistance(predecessores[indexOrigem][indexDestino]);
        }
        i++;
        for (int j = 0 ; j < getNumVertex() ; j++) {
            if (vertexSet[j]->info == predecessores[indexOrigem][indexDestino]->info) {
                indexDestino = j;
                break;
            }
        }
    }

    res.push_back(dest);

    dist = dist + findVertex(res[i-1])->getEdgeDistance(findVertex(dest));
    res.insert(res.begin(), orig);
    return std::make_pair(res,dist);

}



#endif /* GRAPH_H_ */
