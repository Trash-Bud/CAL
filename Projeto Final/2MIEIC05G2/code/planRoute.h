#include <iostream>
#include <algorithm>
#include <chrono>
#include <sstream>
#include "productprovider.h"
#include <Windows.h>
#include "Graph.h"
#include "deliveryCar.h"
#include "graphviewer.h"

#ifndef CAL_MP1_ALGO_PLANROUTE_H
#define CAL_MP1_ALGO_PLANROUTE_H

void planRouteForCarsGVBruteForce(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, GraphViewer& gv, PATH_FINDING_ALGO algo);
void planRouteForCarsNoGVNearestNeighbour(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, PATH_FINDING_ALGO algo);
void planRouteForCarsNoGVRecursiveBackTracking(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph);
void planRouteForCarsNoGVBruteForce(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, PATH_FINDING_ALGO algo);

#endif //CAL_MP1_ALGO_PLANROUTE_H
