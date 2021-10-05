#include "client.h"

std::string Client::getTypeOfNode() {
    return "Client";
}

Client::Client(std::string id) : Node(id) {};

Client::Client(std::string id, double lat, double lng, double x, double y, std::unordered_map<std::string, int>& order) : Node(id, lat,lng,x,y) {
    this->order = order;
}

int Client::getNumOfProduct(const std::string name){
    std::unordered_map<std::string, int>::const_iterator it = order.find(name);
    if (it == order.end()){
        return 0;
    }
    else return order[name];
}

bool Client::removeProductAmount(const std::string name, int amount){
    int num = getNumOfProduct(name);
    if (num == 0) {
        return false;
    }
    else if (num <= amount){
        order.erase(name);
    }
    else{
        order[name] = order[name] - amount;
    }
    return true;
}

bool Client::addOrder(std::string productName, int stock) {
    if(this->order.find(productName) != this->order.end()){
        this->order[productName] = this->order[productName] + stock;
    }else{
        this->order[productName] = stock;
    }
    return true;
}

std::unordered_map<std::string, int>& Client::getOrder() {
    return this->order;
}
bool Client::operator!=(const Node &node) const {
    return getId() != node.getId();
}

bool Client::operator==(const Node &node) const {
    return getId() == node.getId();
}

int Client::getNumOfProducts() const{
    int totalCapacity = 0;
    for(auto& prod: this->order){
        totalCapacity += prod.second;
    }
    return totalCapacity;
}