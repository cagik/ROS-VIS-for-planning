#include "base/plannerBase.h"

plannerBase::plannerBase(){}

plannerBase::~plannerBase(){};

bool plannerBase::plan(const State &start, const State &end, const Eigen::MatrixXi &map, KDTree &tree, vector<State>* resultPath){}