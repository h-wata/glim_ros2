/**
 * @file export_ply.cpp
 * @brief CLI tool to export PLY file from GLIM dump data (headless, no viewer required)
 *
 * Usage: ros2 run glim_ros glim_export_ply <dump_path> <output_ply_path>
 */

#include <filesystem>
#include <fstream>
#include <iostream>

#include <glim/mapping/global_mapping.hpp>
#include <glim/util/config.hpp>

namespace {

bool save_ply_binary(const std::string& path, const gtsam_points::PointCloud& points) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    return false;
  }

  const bool with_intensity = points.has_intensities();

  ofs << "ply\n";
  ofs << "format binary_little_endian 1.0\n";
  ofs << "element vertex " << points.size() << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  if (with_intensity) {
    ofs << "property float intensity\n";
  }
  ofs << "end_header\n";

  const size_t stride = with_intensity ? 4 : 3;
  std::vector<float> buffer(points.size() * stride);
  for (size_t i = 0; i < points.size(); i++) {
    float* dst = buffer.data() + i * stride;
    dst[0] = points.points[i].x();
    dst[1] = points.points[i].y();
    dst[2] = points.points[i].z();
    if (with_intensity) {
      dst[3] = points.intensities[i];
    }
  }
  ofs.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(float));
  return static_cast<bool>(ofs);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <dump_path> <output_ply_path>" << '\n';
    std::cerr << "  dump_path      : Path to GLIM dump directory" << '\n';
    std::cerr << "  output_ply_path: Output PLY file path" << '\n';
    return 1;
  }

  const std::string dump_path = argv[1];
  const std::string output_path = argv[2];

  if (!std::filesystem::exists(dump_path)) {
    std::cerr << "Error: Dump directory does not exist: " << dump_path << '\n';
    return 1;
  }

  if (!std::filesystem::exists(dump_path + "/graph.txt")) {
    std::cerr << "Error: graph.txt not found in dump directory: " << dump_path << '\n';
    return 1;
  }

  // Load config from dump if exists
  if (std::filesystem::exists(dump_path + "/config")) {
    std::cout << "Loading config from: " << dump_path << "/config" << '\n';
    glim::GlobalConfig::instance(dump_path + "/config", true);
  }

  // Create GlobalMapping with minimal params (no optimization needed for export)
  std::cout << "Initializing GlobalMapping..." << '\n';
  glim::GlobalMappingParams params;
  params.enable_optimization = false;

  auto global_mapping = std::make_shared<glim::GlobalMapping>(params);

  std::cout << "Loading map from: " << dump_path << '\n';
  if (!global_mapping->load(dump_path)) {
    std::cerr << "Error: Failed to load map from: " << dump_path << '\n';
    return 1;
  }
  std::cout << "Map loaded successfully" << '\n';

  std::cout << "Exporting points..." << '\n';
  const gtsam_points::PointCloud::Ptr points = global_mapping->export_points();

  if (!points || points->size() == 0) {
    std::cerr << "Warning: No points exported" << '\n';
    return 1;
  }

  std::cout << "Exported " << points->size() << " points" << '\n';

  std::filesystem::path output_file(output_path);
  if (output_file.has_parent_path()) {
    std::filesystem::create_directories(output_file.parent_path());
  }

  std::cout << "Saving PLY to: " << output_path << '\n';
  if (!save_ply_binary(output_path, *points)) {
    std::cerr << "Error: Failed to write PLY: " << output_path << '\n';
    return 1;
  }

  std::cout << "Done!" << '\n';
  return 0;
}
