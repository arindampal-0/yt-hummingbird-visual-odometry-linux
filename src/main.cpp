#include <iostream>
#include <filesystem>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./vo path_to_kitty_dataset\n";
        return -1;
    }

    std::cout << "Visual Odometry\n";

    // std::filesystem::path dataset_path = std::filesystem::current_path() / "kitty_dataset" / "data_odometry_gray";

    std::filesystem::path dataset_path = argv[1];
    std::cout << dataset_path << "\n";

    return 0;
}