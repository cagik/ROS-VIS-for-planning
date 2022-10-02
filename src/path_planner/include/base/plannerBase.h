#pragma once

#include <eigen3/Eigen/Core>

#include "tool/KDTree.hpp"
#include "tool/tool.h"
#include "tool/corridor.h"
#include "data_struct/data_struct.h"

using namespace Car;

class plannerBase
{
public:
    plannerBase();
    virtual ~plannerBase();
    virtual bool plan(const State &start, const State &end, const Eigen::MatrixXi &map, KDTree &tree, vector<State>* resultPath);
};


