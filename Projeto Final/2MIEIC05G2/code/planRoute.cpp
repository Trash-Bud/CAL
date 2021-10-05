#include "planRoute.h"

void planRouteForCarsGVBruteForce(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, GraphViewer& gv, PATH_FINDING_ALGO algo){
    vector<vector<int>> bestClientCombo = {};
    vector<int> clientIds;
    for(int i = 0; i < clients.size();i++){
        clientIds.push_back(i);
    }
    //get the total stock of products the providers have
    unordered_map<string,int> availableStock;
    for(auto& provider: providers){
        for(auto& product: provider->getStock()){
            if(availableStock.find(product.first) != availableStock.end()){
                availableStock[product.first] += product.second;
            }else{
                availableStock[product.first] = product.second;
            }
        }
    }

    vector<int> clientCombo;

    double bestWeight = -1;
    double currWeight = 0;

    for(DeliveryCar* deliveryCar: deliveryCars){
        bestClientCombo = {};
        bestWeight = -1;
        cout << "Now evaluating the best route for delivery car: " << deliveryCar->getId() << "\n";
        Sleep(3000);
        for(int i = 0; i < clientIds.size();i++) {
            clientCombo = {};
            currWeight = 0;
            clientCombo.push_back(clientIds[i]);
            currWeight += clients[clientIds[i]]->getNumOfProducts();
            if(currWeight > deliveryCar->getCapacity()){
                break;
            }
            handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);
            for(int j = i + 1; j < clientIds.size();j++) {
                clientCombo.push_back(clientIds[j]);
                currWeight += clients[clientIds[j]]->getNumOfProducts();
                if(currWeight > deliveryCar->getCapacity()){
                    break;
                }
                handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);

            }

        }
        double bestDistance = 999999;
        vector<int> trueBestClientCombo;
        pair<vector<Node>, double> bestPath;
        //now we have the best combinations possible for this car, lets test them and see which one ends up with our car travelling the least!
        if(bestClientCombo.size() == 0){
            cout << "No possible delieveries for delivery car number " << deliveryCar->getId() << "\n";
        }else{
            for(auto& combo: bestClientCombo){
                deliveryCar->setClientsToDeliverTo(combo);
                bestPath = deliveryCar->getBestPossiblePathBruteForce(providers,clients,graph,gv,algo);
                if(bestPath.second < bestDistance){
                    trueBestClientCombo = combo;
                    bestDistance = bestPath.second;
                }
            }
            cout << bestDistance<<endl;

        }

        //after the best combination is found for one delivery car we need to remove the clients already being serviced
        for(auto iter = clientIds.begin(); iter != clientIds.end();){
            auto toRemove = find(trueBestClientCombo.begin(),trueBestClientCombo.end(),*iter);
            if(toRemove != trueBestClientCombo.end()){
                iter = clientIds.erase(iter);
            }else{
                iter++;
            }
        }

        //before going to the next deliveryCar, remove the stock from this delivery as if it was already done!
        removeFromStock(availableStock,trueBestClientCombo,clients);
        removeFromProviders(providers,bestPath.first,deliveryCar->getShoppingList());
        deliveryCar->setNodesTravelled(bestPath.first);
    }

}



void planRouteForCarsNoGVNearestNeighbour(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, PATH_FINDING_ALGO algo){
    vector<vector<int>> bestClientCombo = {};
    vector<int> clientIds;
    for(int i = 0; i < clients.size();i++){
        clientIds.push_back(i);
    }
    //get the total stock of products the providers have
    unordered_map<string,int> availableStock;
    for(auto& provider: providers){
        for(auto& product: provider->getStock()){
            if(availableStock.find(product.first) != availableStock.end()){
                availableStock[product.first] += product.second;
            }else{
                availableStock[product.first] = product.second;
            }
        }
    }

    vector<int> clientCombo;

    double bestWeight = -1;
    double currWeight = 0;

    for(DeliveryCar* deliveryCar: deliveryCars){
        bestClientCombo = {};
        bestWeight = -1;
        cout << "Now evaluating the best route for delivery car: " << deliveryCar->getId() << "\n";
        Sleep(3000);
        for(int i = 0; i < clientIds.size();i++) {
            clientCombo = {};
            currWeight = 0;
            clientCombo.push_back(clientIds[i]);
            currWeight += clients[clientIds[i]]->getNumOfProducts();
            if(currWeight > deliveryCar->getCapacity()){
                break;
            }
            handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);
            for(int j = i + 1; j < clientIds.size();j++) {
                clientCombo.push_back(clientIds[j]);
                currWeight += clients[clientIds[j]]->getNumOfProducts();
                if(currWeight > deliveryCar->getCapacity()){
                    break;
                }
                handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);

            }

        }
        double bestDistance = 999999;
        vector<int> trueBestClientCombo;
        pair<vector<Node>, double> bestPath;
        //now we have the best combinations possible for this car, lets test them and see which one ends up with our car travelling the least!
        if(bestClientCombo.size() == 0){
            cout << "No possible deliveries for delivery car number " << deliveryCar->getId() << "\n";
        }else{
            for(auto& combo: bestClientCombo){
                deliveryCar->setClientsToDeliverTo(combo);
                bestPath = deliveryCar->getBestPossiblePathNearestNeighbourNoGV(providers,clients,graph, algo);
                if(bestPath.second < bestDistance){
                    trueBestClientCombo = combo;
                    bestDistance = bestPath.second;
                }
            }
        }

        //after the best combination is found for one delivery car we need to remove the clients already being serviced
        for(auto iter = clientIds.begin(); iter != clientIds.end();){
            auto toRemove = find(trueBestClientCombo.begin(),trueBestClientCombo.end(),*iter);
            if(toRemove != trueBestClientCombo.end()){
                iter = clientIds.erase(iter);
            }else{
                iter++;
            }
        }

        //before going to the next deliveryCar, remove the stock from this delivery as if it was already done!
        removeFromStock(availableStock,trueBestClientCombo,clients);
        removeFromProviders(providers,bestPath.first,deliveryCar->getShoppingList());
        deliveryCar->setNodesTravelled(bestPath.first);

    }

}

void planRouteForCarsNoGVRecursiveBackTracking(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph){
    vector<vector<int>> bestClientCombo = {};
    vector<int> clientIds;
    for(int i = 0; i < clients.size();i++){
        clientIds.push_back(i);
    }
    //get the total stock of products the providers have
    unordered_map<string,int> availableStock;
    for(auto& provider: providers){
        for(auto& product: provider->getStock()){
            if(availableStock.find(product.first) != availableStock.end()){
                availableStock[product.first] += product.second;
            }else{
                availableStock[product.first] = product.second;
            }
        }
    }

    vector<int> clientCombo;

    double bestWeight = -1;
    double currWeight = 0;

    for(DeliveryCar* deliveryCar: deliveryCars){
        bestClientCombo = {};
        bestWeight = -1;
        cout << "Now evaluating the best route for delivery car: " << deliveryCar->getId() << "\n";
        Sleep(3000);
        for(int i = 0; i < clientIds.size();i++) {
            clientCombo = {};
            currWeight = 0;
            clientCombo.push_back(clientIds[i]);
            currWeight += clients[clientIds[i]]->getNumOfProducts();
            if(currWeight > deliveryCar->getCapacity()){
                break;
            }
            handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);
            for(int j = i + 1; j < clientIds.size();j++) {
                clientCombo.push_back(clientIds[j]);
                currWeight += clients[clientIds[j]]->getNumOfProducts();
                if(currWeight > deliveryCar->getCapacity()){
                    break;
                }
                handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);

            }

        }
        double bestDistance = 999999;
        vector<int> trueBestClientCombo;
        pair<vector<Node>, double> bestPath;
        //now we have the best combinations possible for this car, lets test them and see which one ends up with our car travelling the least!
        if(bestClientCombo.size() == 0){
            cout << "No possible deliveries for delivery car number " << deliveryCar->getId() << "\n";
        }else{
            for(auto& combo: bestClientCombo){
                deliveryCar->setClientsToDeliverTo(combo);
                bestPath = deliveryCar->getBestPossiblePathNoGVBackTrackingRecursive(providers, clients, graph);
                if(bestPath.second < bestDistance){
                    trueBestClientCombo = combo;
                    bestDistance = bestPath.second;
                }
            }
            cout << bestDistance<<endl;
        }

        //after the best combination is found for one delivery car we need to remove the clients already being serviced
        for(auto iter = clientIds.begin(); iter != clientIds.end();){
            auto toRemove = find(trueBestClientCombo.begin(),trueBestClientCombo.end(),*iter);
            if(toRemove != trueBestClientCombo.end()){
                iter = clientIds.erase(iter);
            }else{
                iter++;
            }
        }

        //before going to the next deliveryCar, remove the stock from this delivery as if it was already done!
        removeFromStock(availableStock,trueBestClientCombo,clients);
        removeFromProviders(providers,bestPath.first,deliveryCar->getShoppingList());
        deliveryCar->setNodesTravelled(bestPath.first);

    }


}

void planRouteForCarsNoGVBruteForce(vector<DeliveryCar*>& deliveryCars, vector<Client* >& clients, vector<Provider *>& providers, Graph<Node>& graph, PATH_FINDING_ALGO algo){
    vector<vector<int>> bestClientCombo = {};
    vector<int> clientIds;
    for(int i = 0; i < clients.size();i++){
        clientIds.push_back(i);
    }
    //get the total stock of products the providers have
    unordered_map<string,int> availableStock;
    for(auto& provider: providers){
        for(auto& product: provider->getStock()){
            if(availableStock.find(product.first) != availableStock.end()){
                availableStock[product.first] += product.second;
            }else{
                availableStock[product.first] = product.second;
            }
        }
    }

    vector<int> clientCombo;

    double bestWeight = -1;
    double currWeight = 0;

    for(DeliveryCar* deliveryCar: deliveryCars){
        bestClientCombo = {};
        bestWeight = -1;
        cout << "Now evaluating the best route for delivery car: " << deliveryCar->getId() << "\n";
        Sleep(3000);
        for(int i = 0; i < clientIds.size();i++) {
            clientCombo = {};
            currWeight = 0;
            clientCombo.push_back(clientIds[i]);
            currWeight += clients[clientIds[i]]->getNumOfProducts();
            if(currWeight > deliveryCar->getCapacity()){
                break;
            }
            handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);
            for(int j = i + 1; j < clientIds.size();j++) {
                clientCombo.push_back(clientIds[j]);
                currWeight += clients[clientIds[j]]->getNumOfProducts();
                if(currWeight > deliveryCar->getCapacity()){
                    break;
                }
                handleNewClientCombo(bestClientCombo,clientCombo,bestWeight,currWeight,availableStock,clients);

            }

        }
        double bestDistance = 999999;
        vector<int> trueBestClientCombo;
        pair<vector<Node>, double> bestPath;
        //now we have the best combinations possible for this car, lets test them and see which one ends up with our car travelling the least!
        if(bestClientCombo.size() == 0){
            cout << "No possible deliveries for delivery car number " << deliveryCar->getId() << "\n";
        }else{
            for(auto& combo: bestClientCombo){
                deliveryCar->setClientsToDeliverTo(combo);
                bestPath = deliveryCar->getBestPossiblePathNoGVBruteForce(providers,clients,graph, algo);
                if(bestPath.second < bestDistance){
                    trueBestClientCombo = combo;
                    bestDistance = bestPath.second;
                }
            }
            cout << bestDistance<<endl;

        }

        //after the best combination is found for one delivery car we need to remove the clients already being serviced
        for(auto iter = clientIds.begin(); iter != clientIds.end();){
            auto toRemove = find(trueBestClientCombo.begin(),trueBestClientCombo.end(),*iter);
            if(toRemove != trueBestClientCombo.end()){
                iter = clientIds.erase(iter);
            }else{
                iter++;
            }
        }

        //before going to the next deliveryCar, remove the stock from this delivery as if it was already done!
        removeFromStock(availableStock,trueBestClientCombo,clients);
        removeFromProviders(providers,bestPath.first,deliveryCar->getShoppingList());
        deliveryCar->setNodesTravelled(bestPath.first);

    }

}