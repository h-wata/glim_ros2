/**
 * @file export_ply.cpp
 * @brief CLI tool to export PLY file from GLIM dump data
 *
 * Usage: ros2 run glim_ros glim_export_ply <dump_path> <output_ply_path>
 */

#include <filesystem>
#include <glim/mapping/global_mapping.hpp>
#include <glim/util/config.hpp>
#include <glk/io/ply_io.hpp>
#include <iostream>

auto main(int argc, char ** argv) -> int
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <dump_path> <output_ply_path>" << '\n';
    std::cerr << "  dump_path      : Path to GLIM dump directory" << '\n';
    std::cerr << "  output_ply_path: Output PLY file path" << '\n';
    return 1;
  }

  const std::string dump_path = argv[1];
  const std::string output_path = argv[2];

  // Check if dump directory exists
  if (!std::filesystem::exists(dump_path)) {
    std::cerr << "Error: Dump directory does not exist: " << dump_path << '\n';
    return 1;
  }

  // Check if graph.txt exists
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

  // Load map
  std::cout << "Loading map from: " << dump_path << '\n';
  if (!global_mapping->load(dump_path)) {
    std::cerr << "Error: Failed to load map from: " << dump_path << '\n';
    return 1;
  }
  std::cout << "Map loaded successfully" << '\n';

  // Export points
  std::cout << "Exporting points..." << '\n';
  const glk::PLYData points = global_mapping->export_points();

  if (points.vertices.empty()) {
    std::cerr << "Warning: No points exported" << '\n';
    return 1;
  }

  std::cout << "Exported " << points.vertices.size() << " points" << '\n';

  // Ensure output directory exists
  std::filesystem::path output_file(output_path);
  if (output_file.has_parent_path()) {
    std::filesystem::create_directories(output_file.parent_path());
  }

  // Save PLY file
  std::cout << "Saving PLY to: " << output_path << '\n';
  glk::save_ply_binary(output_path, points);

  std::cout << "Done!" << '\n';
  return 0;
}
