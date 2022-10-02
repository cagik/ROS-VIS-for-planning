#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

using namespace std;

namespace  Car{

struct State
{
    State() = default;
    State(double x, double y, double head) : x(x), y(y), heading(head){}
    double x{};
    double y{};
    double heading{};
};

}

enum NodeType{
    freeGrid = 0,
    obstacle,
    inOpenList,
    inCloseList
};

struct astarNode{
    
    pair<int, int> point;

    int F, G, H; 

    astarNode* parent; 

    astarNode(pair<int, int> _point = make_pair(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

#endif