#pragma once

#include <string>

class VisualOdometry
{
    public:
        VisualOdometry() = default;
        int run(std::string datasetPath);
};