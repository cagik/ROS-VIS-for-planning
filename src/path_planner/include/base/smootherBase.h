#pragma once

#include <vector>

#include "data_struct/data_struct.h"

using namespace Car;

class smootherBase
{
public:
    smootherBase();
    virtual ~smootherBase();
    virtual bool smooth(std::vector<State> &path);
};
