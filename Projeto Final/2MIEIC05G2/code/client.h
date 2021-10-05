#ifndef CAL_MP1_ALGO_CLIENT_H
#define CAL_MP1_ALGO_CLIENT_H

#include "node.h"
#include <string>
#include <unordered_map>

class Client : public Node{
private:
    std::unordered_map<std::string, int> order;
public:
    Client(std::string id);
    Client(std::string id, double lat, double lng, double x, double y, std::unordered_map<std::string, int>& order );
    std::string getTypeOfNode() override;
    bool addOrder(std::string productName, int stock);
    std::unordered_map<std::string, int>& getOrder();
    bool operator==(const Node& node) const;
    bool operator!=(const Node& node) const;
    int getNumOfProducts() const;
    int getNumOfProduct(const std::string name);
    bool removeProductAmount(const std::string name, int amount);

};

#endif //CAL_MP1_ALGO_PROD_H