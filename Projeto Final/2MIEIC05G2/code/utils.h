//
// Created by radio on 07/05/2021.
//

#ifndef CAL_MP1_ALGO_UTILS_H
#define CAL_MP1_ALGO_UTILS_H

#include <vector>
#include "productprovider.h"
#include "Graph.h"
#include "graphviewer.h"
#include "client.h"

enum PATH_FINDING_ALGO{
    DIJSKTRA,
    FLOYD_WARSHALL
};

using namespace std;

//BUILD GRAPH
void buildGraphFromTxtNoGV(Graph<Node>& graph,const string& edgesTxt,const string& nodesTxt,const string& nodesXY, vector<Client* >& clients, vector<Provider*>& providers);
void buildGraphFromTxt(GraphViewer& gv, Graph<Node>& graph,const string& edgesTxt,const string& nodesTxt,const string& nodesXY, vector<Client* >& clients, vector<Provider*>& providers);
std::vector<int> getNodeIds(string& nodesTxt);
double calculate_distance_between_two_nodes(const Node& node1, const Node& node2);

//CHECK TYPE
Provider* checkIfProvider(std::vector<Provider *> &providers, int id);
Client* checkIfClient(std::vector<Client*> &clients, int id);

//CREATE CLIENTS AND PROVIDERS FOR THE GRAPH
void fill_client_and_provider_rand(std::vector<int> nodeIds, std::vector<Client* >& clients, std::vector<Provider *>& providers, int number_clients, int number_providers, std::vector<string> products);
void fix_client_and_provider_color(std::vector<Client*>& clients, GraphViewer::Node& node, std::vector<Provider*>& providers,const std::string& id);
void fill_client_and_provider_rand_providers_guaranteed(vector<int> nodeIds, vector<Client* >& clients, vector<Provider *>& providers, int number_clients,vector<string> products);

//MAP OF PRODUCTS RELATED
bool checkIfProviderIsRelevant(Provider* provider, std::unordered_map<std::string, int> shoppingList);
bool check_if_available(unordered_map<string,int> availableStock,vector<int> clientId, vector<Client*>& clients);
void removeFromStock(unordered_map<string,int>& availableStock, vector<int> clientId, vector<Client*>& clients);
void unloadMerch(std::unordered_map<std::string, int> &carStock,Client* client);
void removeFromProviders(vector<Provider*>& providers, vector<Node>& path, unordered_map<std::string,int> shoppingList);

//CLIENT CHOICE RELATED
void handleNewClientCombo(vector<vector<int>>& bestClientCombo, vector<int>& clientCombo, double & bestWeight, double currWeight, unordered_map<string,int>& availableStock, vector<Client* >& clients);

//OTHERS
void findPermutations(vector<int> a, vector<vector<int>>& allPerms);
#endif //CAL_MP1_ALGO_UTILS_H
