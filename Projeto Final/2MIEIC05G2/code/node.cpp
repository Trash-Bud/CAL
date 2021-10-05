#include "node.h"

std::string Node::getTypeOfNode() {
    return "Basic";
}

Node::Node(std::string id) {
    this->id = id;
}


std::string Node::getId() const {
    return this->id;
}

bool Node::operator==(const Node &node) const {
    return node.id == this->id;
}

bool Node::operator!=(const Node &node) const {
    return node.id != this->id;
}

double Node::getLat() const {
    return lat;
}

void Node::setLat(double lat) {
    Node::lat = lat;
}

double Node::getLng() const {
    return lng;
}

void Node::setLng(double lng) {
    Node::lng = lng;
}

Node::Node(const std::string &id, double lat, double lng, double x, double y) : id(id), lat(lat), lng(lng), x(x), y(y) {}

double Node::getX() const {
    return x;
}

void Node::setX(double x) {
    Node::x = x;
}

double Node::getY() const {
    return y;
}

void Node::setY(double y) {
    Node::y = y;
}

void Node::setId(const std::string &id) {
    Node::id = id;
}
