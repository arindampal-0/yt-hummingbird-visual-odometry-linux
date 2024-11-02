#include <filesystem>
#include <iostream>

#include "vo.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./vo path_to_kitti_dataset\n";
        return -1;
    }

    std::cout << "Visual Odometry\n";

    VisualOdometry vo;

    std::filesystem::path datasetPath = argv[1];
    // std::cout << dataset_path << "\n";

    int ret = vo.run(datasetPath.string());

    return ret;
}