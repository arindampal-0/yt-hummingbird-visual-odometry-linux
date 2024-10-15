#include <filesystem>
#include <iostream>

#include "vo.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./vo path_to_kitty_dataset\n";
        return -1;
    }

    std::cout << "Visual Odometry\n";

    VisualOdometry vo;

    // std::filesystem::path dataset_path = std::filesystem::current_path() /
    // "kitty_dataset" / "data_odometry_gray";

    std::filesystem::path datasetPath = argv[1];
    // std::cout << dataset_path << "\n";

    std::filesystem::path imagesetPath = datasetPath / "data_odometry_gray/dataset/sequences/00";

    int ret = vo.run(imagesetPath.string());

    return ret;
}