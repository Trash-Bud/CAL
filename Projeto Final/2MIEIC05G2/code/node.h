#ifndef CAL_MP1_ALGO_BASICNODE_H
#define CAL_MP1_ALGO_BASICNODE_H

#include <string>


class Node{
private:
    std::string id;
    double lat;
    double lng;
    double x;
    double y;
public:
    Node(std::string id);
    Node(const std::string &id, double lat, double lng, double x, double y);

    virtual std::string getTypeOfNode();
    std::string getId() const;
    bool operator==(const Node& node) const;
    bool operator!=(const Node& node) const;

    double getLat() const;

    void setLat(double lat);

    double getLng() const;

    void setLng(double lng);

    double getX() const;

    void setX(double x);

    double getY() const;

    void setY(double y);

    void setId(const std::string &id);
};

#endif //CAL_MP1_ALGO_BASICNODE_H