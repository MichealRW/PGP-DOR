#include "CRemoveDynamic/CRemoveDynamicV2.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct GlobalConfig {
    std::string lidar_data_path;
    int start_frame = 0;
    int end_frame = 0;
};

std::filesystem::path ResolveExecutableDirectory(const char *argv_0) {
    std::error_code error_code;
    std::filesystem::path executable_path = std::filesystem::canonical(argv_0, error_code);
    if (error_code) {
        executable_path = std::filesystem::absolute(argv_0, error_code);
    }
    if (error_code) {
        executable_path = std::filesystem::current_path() / argv_0;
    }
    return executable_path.parent_path();
}

GlobalConfig ReadGlobalConfig(const std::filesystem::path &param_path) {
    GlobalConfig global_config;

    std::ifstream fin(param_path);
    if (!fin.is_open()) {
        throw std::runtime_error("Fail to open params file: " + param_path.string());
    }

    std::string token;
    while (fin >> token) {
        if (token.empty()) {
            continue;
        }
        if (token[0] == '#' || token[0] == '/') {
            std::getline(fin, token);
            continue;
        }
        if (token == "m_LoadPath") {
            fin >> global_config.lidar_data_path;
            continue;
        }
        if (token == "start_time") {
            fin >> global_config.start_frame;
            continue;
        }
        if (token == "end_time") {
            fin >> global_config.end_frame;
            continue;
        }
    }

    if (global_config.lidar_data_path.empty()) {
        throw std::runtime_error("m_LoadPath is missing in " + param_path.string());
    }
    if (global_config.end_frame <= global_config.start_frame) {
        throw std::runtime_error("Invalid frame range in " + param_path.string());
    }

    return global_config;
}

std::filesystem::path ResolveFramePath(
    const std::filesystem::path &pcd_dir,
    int frame_id,
    bool b_zero_indexed) {
    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(6) << (b_zero_indexed ? frame_id : frame_id + 1) << ".pcd";
    return pcd_dir / stream.str();
}

bool DetectZeroIndexedDataset(const std::filesystem::path &pcd_dir) {
    return std::filesystem::exists(ResolveFramePath(pcd_dir, 0, true));
}

void EnsureOutputDirectories(const std::filesystem::path &working_dir, const std::filesystem::path &dataset_dir) {
    std::filesystem::create_directories(working_dir / "oldmap");
    std::error_code error_code;
    std::filesystem::create_directories(dataset_dir / "result_pgp", error_code);
    std::filesystem::create_directories(working_dir / "outputs");
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const std::filesystem::path executable_dir = ResolveExecutableDirectory(argv[0]);
        std::filesystem::current_path(executable_dir);

        const std::filesystem::path param_dir = executable_dir / "parameters";
        const std::filesystem::path global_param_path = param_dir / "pgp_dor_global.ini";
        const std::filesystem::path remove_dynamic_param_path = param_dir / "pgp_dor_remove_dynamic.ini";

        const GlobalConfig global_config = ReadGlobalConfig(global_param_path);
        const std::filesystem::path lidar_data_path = global_config.lidar_data_path;
        const std::filesystem::path pcd_dir = lidar_data_path / "pcd";
        const bool b_zero_indexed = DetectZeroIndexedDataset(pcd_dir);

        EnsureOutputDirectories(executable_dir, lidar_data_path);

        RemoveDynamicV2 remove_dynamic;
        remove_dynamic.SetLiDARDataPath(lidar_data_path.string());
        remove_dynamic.SetStartFrame(global_config.start_frame);
        remove_dynamic.SetParamFile(remove_dynamic_param_path.string());
        remove_dynamic.Initialize();

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int frame_id = global_config.start_frame; frame_id < global_config.end_frame; ++frame_id) {
            const std::filesystem::path frame_path = ResolveFramePath(pcd_dir, frame_id, b_zero_indexed);
            if (!std::filesystem::exists(frame_path)) {
                std::cerr << "Skip missing frame: " << frame_path << std::endl;
                continue;
            }

            current_cloud->clear();
            if (pcl::io::loadPCDFile(frame_path.string(), *current_cloud) != 0) {
                std::cerr << "Fail to load: " << frame_path << std::endl;
                continue;
            }

            std::cout << (frame_id - global_config.start_frame + 1) << " / "
                      << (global_config.end_frame - global_config.start_frame) << std::endl;

            const Eigen::Matrix3d r_w_l =
                current_cloud->sensor_orientation_.cast<double>().toRotationMatrix();
            const Eigen::Vector3d t_w_l =
                current_cloud->sensor_origin_.head<3>().cast<double>();

            std::printf("t_w_l: %f, %f, %f\n", t_w_l(0), t_w_l(1), t_w_l(2));
            remove_dynamic.Run(current_cloud, t_w_l, r_w_l, frame_id);
        }

        remove_dynamic.LoadAll();
        remove_dynamic.EndOut();
        return 0;
    } catch (const std::exception &exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }
}
