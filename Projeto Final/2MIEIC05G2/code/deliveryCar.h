#ifndef CAL_MP1_ALGO_CAR_H
#define CAL_MP1_ALGO_CAR_H

#include "client.h"
#include "utils.h"
#include "Graph.h"
#include "productprovider.h"
#include "graphviewer.h"
#include <string>
#include <vector>
#include <unordered_map>


class DeliveryCar{
private:
    std::string id;
    int capacity;
    std::unordered_map<std::string, int> shoppingList;
    std::vector<Node> nodesTravelled;
    std::vector<int> clientsToDeliverTo;
public:
    DeliveryCar(std::string id, int capacity);

    //get
    int getCapacity() const;
    const std::string &getId() const;
    std::unordered_map<std::string, int> getShoppingList() const;
    const std::vector<Node> &getNodesTravelled() const;


    //set
    void setNodesTravelled(const std::vector<Node> &nodesTravelled);

    //Shopping List related
    bool addToShoppingList(std::string ProductName, int amount);
    void fillShoppingList(std::vector<int> clientIds, std::vector<Client*> clients);
    void loadCar(std::unordered_map<std::string, int>& carStock, std::unordered_map<std::string, int>& stockToLoad );
    bool unloadCar(std::unordered_map<std::string, int> &carStock,Client* client);


    //client related
    void removeFromClients(int clientToRemove);
    void setClientsToDeliverTo(std::vector<int> clients);
    bool checkIfClientIsDeliverable(Client* client, std::unordered_map<std::string, int> currCarryingCar);

    //provider related
    bool checkIfMeetsRequirement(std::unordered_map<std::string, int>& shoppingList);
    void checkProviderCombinations(int set_size, std::vector<std::vector<int>>& providerId, std::vector<Provider *>& providers);
    bool check_if_perm_works(std::vector<int> a, std::vector<Provider *> providers,std::vector<std::vector<int>> &viableRoute);

    //Algorithms

    std::pair<std::vector<Node>, double> getBestPossiblePathBruteForce(std::vector<Provider *>& providers, std::vector<Client*>& clients, Graph<Node>& graph,
                                                             GraphViewer& gv, PATH_FINDING_ALGO algo);

    std::pair<std::vector<Node>, double> getBestPossiblePathNearestNeighbourNoGV(std::vector<Provider *>& providers, std::vector<Client*>& clients, Graph<Node>& graph,
                                                                        PATH_FINDING_ALGO algo);

    pair<vector<Node>, double> getBestPossiblePathNoGVBruteForce(std::vector<Provider *> &providers,
                                                                 std::vector<Client*> &clients, Graph<Node> &graph, PATH_FINDING_ALGO algo);

    pair<vector<Node>, double> getBestPossiblePathNoGVBackTrackingRecursive(std::vector<Provider *> &providers,
                                                                            std::vector<Client*> &clients, Graph<Node> &graph);

    void findBestByBackTracking(vector<int> &visitedNodes, Graph<Node>& graph, vector<int> ids, vector<int>& bestPath, double &bestDistance, double curDistance, vector<Provider*>& providers, vector<Client*>& clients,
                                std::unordered_map<std::string, int>& carStock, int& numClients, int& maxClients);

    //debug
    void printShoppingList();
};

#endif //CAL_MP1_ALGO_CAR_H