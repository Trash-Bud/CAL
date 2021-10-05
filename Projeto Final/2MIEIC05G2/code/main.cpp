#include <iostream>
#include <algorithm>
#include <chrono>
#include <sstream>
#include "productprovider.h"
#include "Graph.h"
#include "deliveryCar.h"
#include "graphviewer.h"
#include "planRoute.h"

using namespace std;

void testConnectivity(){
    Graph<Node> graph;
    Graph<Node> graph_strong;
    //distribute clients to cars
    vector<Client *> clients;
    vector<Provider *> providers;
    std::string city = "penafiel";
    std::string nodesTxt = "penafiel_full_nodes_latlng.txt";
    std::string edgestxt = "penafiel_full_edges.txt";
    std::string nodesXY = "penafiel_full_nodes_xy.txt";

    std::string nodesTxt_strong = "penafiel_strong_nodes_latlng.txt";
    std::string edgestxt_strong = "penafiel_strong_edges.txt";
    std::string nodesXY_strong = "penafiel_strong_nodes_xy.txt";

    buildGraphFromTxtNoGV(graph_strong,edgestxt_strong, nodesTxt_strong, nodesXY_strong,clients,providers);

    buildGraphFromTxtNoGV(graph,edgestxt, nodesTxt, nodesXY,clients,providers);
    cout << "Analysing " << city << " FULL: " << endl;
    for(auto& vertex: graph.getVertexSet()){
        graph.findArt(vertex);
    }

    cout << "Analysing " << city << " STRONG: " << endl;
    for(auto& vertex: graph_strong.getVertexSet()){
        graph.findArt(vertex);
    }
}

void run_with_GV(GraphViewer &gv, Graph<Node> graph, vector<Client*>&clients, vector<Provider*> &providers, vector<DeliveryCar *> deliveryCars, PATH_FINDING_ALGO algo, string city){
    auto start1 = chrono::steady_clock::now();

    sort(deliveryCars.begin(), deliveryCars.end(), [](const DeliveryCar* d1, const DeliveryCar* d2){
        return d1->getCapacity() < d2->getCapacity();
    });

    sort(clients.begin(), clients.end(), [](const Client* c1, const Client* c2){
        return c1->getNumOfProducts() < c2->getNumOfProducts();
    });

    if(algo == FLOYD_WARSHALL){
        graph.floydWarshallShortestPath();
    }

    planRouteForCarsGVBruteForce(deliveryCars, clients, providers, graph, gv, algo);

    auto end1 = chrono::steady_clock::now();

    for(DeliveryCar* deliveryCar1: deliveryCars){
        cout << "best path for car " << deliveryCar1->getId() << " is: " << "\n";
        for(Node node: deliveryCar1->getNodesTravelled()){
            cout << node.getId() << "->";
        }
        cout << "\n";
    }

    cout << "ran for: " << chrono::duration_cast<chrono::milliseconds >(end1 - start1).count() << " milli seconds \n";
    gv.join();
}

void create_graph_with_gv(GraphViewer &gv, Graph<Node> &graph, string city, string nodesTxt, string edgesTxt, string nodesXY, vector<string> products, vector<Client*>&clients, vector<Provider*> &providers, int client_num, int provider_num ){

    fill_client_and_provider_rand(getNodeIds(nodesTxt),clients,providers,client_num,provider_num,products);
    buildGraphFromTxt(gv,graph,edgesTxt, nodesTxt, nodesXY,clients,providers);
    gv.setCenter(sf::Vector2f(0, 0));
    gv.createWindow(1800, 1050);

}

void create_graph_without_gv( Graph<Node> &graph, string city, string nodesTxt, string edgesTxt, string nodesXY, vector<string> products, vector<Client*>&clients, vector<Provider*> &providers, int client_num, int provider_num ){

    fill_client_and_provider_rand(getNodeIds(nodesTxt),clients,providers,client_num,provider_num,products);
    buildGraphFromTxtNoGV(graph,edgesTxt, nodesTxt, nodesXY,clients,providers);
}

void create_example_graph_2( Graph<Node> &graph, string city, string nodesTxt, string edgesTxt, string nodesXY,vector<Client*>&clients, vector<Provider*> &providers) {
    Client * c1 = new Client("7649");
    c1->addOrder("C",2);
    c1->addOrder("B",1);

    Client * c2 = new Client("891");
    c2->addOrder("A",4);
    c2->addOrder("D",5);

    clients = {c1,c2};

    Provider* p1 = new Provider("1333");
    p1->addProduct("B",4);

    Provider* p2 = new Provider("1962");
    p2->addProduct("D",6);

    Provider* p3 = new Provider("9968");
    p3->addProduct("A",9);

    Provider* p4 = new Provider("4843");
    p4->addProduct("C",3);
    p4->addProduct("B",3);

    Provider* p5 = new Provider("5950");
    p5->addProduct("A",16);
    p5->addProduct("E",2);

    providers = {p1,p2,p3,p4,p5};

    buildGraphFromTxtNoGV(graph,edgesTxt, nodesTxt, nodesXY,clients,providers);

}



void create_example_graph_10( Graph<Node> &graph, string city, string nodesTxt, string edgesTxt, string nodesXY,vector<Client*>&clients, vector<Provider*> &providers) {
    Client * c1 = new Client("7649");
    c1->addOrder("C",15);
    c1->addOrder("B",16);

    Client * c2 = new Client("891");
    c2->addOrder("A",13);
    c2->addOrder("D",9);

    Client * c3 = new Client("9109");
    c3->addOrder("C",2);

    Client* c4 = new Client("9684");
    c4->addOrder("E",11);
    c4->addOrder("A",9);
    c4->addOrder("C",4);

    Client* c5 = new Client("849");
    c5->addOrder("D",1);
    c5->addOrder("B",8);
    c5->addOrder("A",15);

    Client* c6 = new Client("2423");
    c6->addOrder("D",4);

    Client* c7 = new Client("537");
    c7->addOrder("E",7);
    c7->addOrder("B",9);

    Client* c8 = new Client("7469");
    c8->addOrder("A",8);

    Client* c9 = new Client("9579");
    c9->addOrder("D",2);

    Client* c10 = new Client("9401");
    c10->addOrder("D",4);
    c10->addOrder("B",6);
    c10->addOrder("C",3);

    clients = {c1,c2,c3,c4,c5,c6,c7,c8,c9,c10};

    Provider* p1 = new Provider("1333");
    p1->addProduct("B",4);

    Provider* p2 = new Provider("1962");
    p2->addProduct("D",6);

    Provider* p3 = new Provider("9968");
    p3->addProduct("A",9);

    Provider* p4 = new Provider("4843");
    p4->addProduct("C",3);
    p4->addProduct("B",3);

    Provider* p5 = new Provider("5950");
    p5->addProduct("A",16);
    p5->addProduct("E",2);

    providers = {p1,p2,p3,p4,p5};

    buildGraphFromTxtNoGV(graph,edgesTxt, nodesTxt, nodesXY,clients,providers);

}

void create_example_graph_5( Graph<Node> &graph, string city, string nodesTxt, string edgesTxt, string nodesXY,vector<Client*>&clients, vector<Provider*> &providers) {
    Client * c1 = new Client("7649");
    c1->addOrder("C",2);
    c1->addOrder("B",16);

    Client * c2 = new Client("891");
    c2->addOrder("D",9);

    Client * c3 = new Client("9109");
    c3->addOrder("C",2);

    Client* c4 = new Client("9684");
    c4->addOrder("E",11);
    c4->addOrder("C",4);

    Client* c5 = new Client("849");
    c5->addOrder("D",1);
    c5->addOrder("B",2);
    c5->addOrder("A",15);


    clients = {c1,c2,c3,c4,c5};

    Provider* p1 = new Provider("1333");
    p1->addProduct("B",15);

    Provider* p2 = new Provider("1962");
    p2->addProduct("D",6);

    Provider* p3 = new Provider("9968");
    p3->addProduct("A",9);

    Provider* p4 = new Provider("4843");
    p4->addProduct("C",3);
    p4->addProduct("B",10);

    Provider* p5 = new Provider("5950");
    p5->addProduct("A",16);
    p5->addProduct("E",2);

    providers = {p1,p2,p3,p4,p5};

    buildGraphFromTxtNoGV(graph,edgesTxt, nodesTxt, nodesXY,clients,providers);

}


void run_without_gv_Nearest_Neighbour(Graph<Node> graph, vector<Client*>&clients, vector<Provider*> &providers, vector<DeliveryCar *> deliveryCars, PATH_FINDING_ALGO algo, string city){
    auto start1 = chrono::steady_clock::now();

    sort(deliveryCars.begin(), deliveryCars.end(), [](const DeliveryCar* d1, const DeliveryCar* d2){
        return d1->getCapacity() < d2->getCapacity();
    });

    sort(clients.begin(), clients.end(), [](const Client* c1, const Client* c2){
        return c1->getNumOfProducts() < c2->getNumOfProducts();
    });

    //pre processing for floyd warshall algorithm
    if(algo == FLOYD_WARSHALL){
        graph.floydWarshallShortestPath();
    }

    planRouteForCarsNoGVNearestNeighbour(deliveryCars, clients, providers, graph, algo);

    auto end1 = chrono::steady_clock::now();

    for(DeliveryCar* deliveryCar1: deliveryCars){
        cout << "best path for car " << deliveryCar1->getId() << " is: " << "\n";
        for(Node node: deliveryCar1->getNodesTravelled()){
            cout << node.getId() << "->";
        }
        cout << "\n";
    }

    cout << "ran for: " << chrono::duration_cast<chrono::milliseconds >(end1 - start1).count() << " milli seconds \n";

}

void run_without_gv_recursive(Graph<Node> graph, vector<Client*>&clients, vector<Provider*> &providers, vector<DeliveryCar *> deliveryCars){
    auto start1 = chrono::steady_clock::now();

    sort(deliveryCars.begin(), deliveryCars.end(), [](const DeliveryCar* d1, const DeliveryCar* d2){
        return d1->getCapacity() < d2->getCapacity();
    });

    sort(clients.begin(), clients.end(), [](const Client* c1, const Client* c2){
        return c1->getNumOfProducts() < c2->getNumOfProducts();
    });

    planRouteForCarsNoGVRecursiveBackTracking(deliveryCars, clients, providers, graph);

    auto end1 = chrono::steady_clock::now();

    for(DeliveryCar* deliveryCar1: deliveryCars){
        cout << "best path for car " << deliveryCar1->getId() << " is: " << "\n";
        for(Node node: deliveryCar1->getNodesTravelled()){
            cout << node.getId() << "->";
        }
        cout << "\n";
    }

    cout << "ran for: " << chrono::duration_cast<chrono::milliseconds >(end1 - start1).count() << " milli seconds \n";
}

void run_without_gv_permutations(Graph<Node> graph, vector<Client*>&clients, vector<Provider*> &providers, vector<DeliveryCar *> deliveryCars, PATH_FINDING_ALGO algo, string city){

    auto start1 = chrono::steady_clock::now();

    sort(deliveryCars.begin(), deliveryCars.end(), [](const DeliveryCar* d1, const DeliveryCar* d2){
        return d1->getCapacity() < d2->getCapacity();
    });

    sort(clients.begin(), clients.end(), [](const Client* c1, const Client* c2){
        return c1->getNumOfProducts() < c2->getNumOfProducts();
    });

    //pre processing for floyd warshall algorithm
    if(algo == FLOYD_WARSHALL){
        graph.floydWarshallShortestPath();
    }

    planRouteForCarsNoGVBruteForce(deliveryCars, clients, providers, graph, algo);

    auto end1 = chrono::steady_clock::now();

    for(DeliveryCar* deliveryCar1: deliveryCars){
        cout << "best path for car " << deliveryCar1->getId() << " is: " << "\n";
        for(Node node: deliveryCar1->getNodesTravelled()){
            cout << node.getId() << "->";
        }
        cout << "\n";
    }

    cout << "ran for: " << chrono::duration_cast<chrono::milliseconds >(end1 - start1).count() << " milli seconds \n";

}

int main() {
    Graph<Node> graph;
    GraphViewer gv;
    DeliveryCar deliveryCar("1",10);
    DeliveryCar deliveryCar1("2",10);
    //vector<DeliveryCar *> deliveryCars = {&deliveryCar, &deliveryCar1};
    vector<DeliveryCar *> deliveryCars = {&deliveryCar};
    vector<Client *> clients;
    vector<Provider *> providers;

    vector<string> products = {"A", "B", "C", "D", "E"};
    std::string city = "penafiel";
    std::string nodesTxt = "penafiel_strong_nodes_latlng.txt";
    std::string edgesTxt = "penafiel_strong_edges.txt";
    std::string nodesXY = "penafiel_strong_nodes_xy.txt";

    int num_clients = 20;
    int num_providers = 5;

    //uncomment to create a graph with GV
    //create_graph_with_gv(gv,graph,city,nodesTxt,edgesTxt,nodesXY,products,clients,providers,num_clients,num_providers);

    //uncomment to run permutations with FLOYD_WARSHALL
    //run_with_GV(gv,graph,clients,providers,deliveryCars,FLOYD_WARSHALL,city);

    //uncomment to run permutations with DIJSKTRA
    //run_with_GV(gv,graph,clients,providers,deliveryCars,DIJSKTRA,city);

    //uncomment to create a random graph with no GV
    //create_graph_without_gv(graph,city,nodesTxt,edgesTxt,nodesXY,products,clients,providers,num_clients,num_providers);

    //uncomment for example graph no GV
    //create_example_graph_5(graph,city,nodesTxt,edgesTxt,nodesXY,clients,providers);
    //create_example_graph_2(graph,city,nodesTxt,edgesTxt,nodesXY,clients,providers);
    //create_example_graph_10(graph,city,nodesTxt,edgesTxt,nodesXY,clients,providers);

    //uncomment to run permutations with FLOYD_WARSHALL (no GV)
    //run_without_gv_permutations(graph,clients,providers,deliveryCars,FLOYD_WARSHALL,city);

    //uncomment to run permutations with DIJSKTRA (no GV)
    //run_without_gv_permutations(graph,clients,providers,deliveryCars,DIJSKTRA,city);

    //uncomment to run recursive (no GV)
    //run_without_gv_recursive(graph,clients,providers,deliveryCars);

    //uncomment to run nearest neighbour search with DIJSKTRA (no GV)
    //run_without_gv_Nearest_Neighbour(graph,clients,providers,deliveryCars,DIJSKTRA,city);

    //uncomment to run nearest neighbour search with FLOYD_WARSHALL (no GV)
    //run_without_gv_Nearest_Neighbour(graph,clients,providers,deliveryCars,FLOYD_WARSHALL,city);


    return 0;
}
