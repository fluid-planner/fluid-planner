#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <trajectory/trajectory.h>

// Defines a collection of data handling for CSV data, as well as write to CSV.
namespace data_handling {

// Declare the IO formats for CSV.
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                       Eigen::DontAlignCols, ",", "\n", "", "",
                                       "", "\n");
const static Eigen::IOFormat CSVFlatMatrixFormat(Eigen::StreamPrecision,
                                      Eigen::DontAlignCols, ",", ",", "", "",
                                      "", "\n");

// Load a CSV file that contains a matrix of size M x N.
Eigen::MatrixXf LoadCsv(const std::string &filename, int M, int N) {
  Eigen::MatrixXf data(M, N);
  std::ifstream infile(filename);

  int i = 0;
  if (!infile) {
    std::cerr << "File '" << filename << "' not found!" << std::endl;
  }

  while (infile) {
    std::string s;
    if (!std::getline(infile, s))
      break;

    std::istringstream ss(s);

    int j = 0;
    while (ss) {
      std::string s;
      if (!std::getline(ss, s, ','))
        break;
      data(i, j) = std::stof(s);
      j++;
    }
    i++;
  }
  std::cout << "Loaded file '" << filename << "'." << std::endl;
  return data;
}

// Given a set of trajectories `traj` and a frame specification `frame`, write
// the waypoints to the output file `file`.
void WriteWptsToFile(std::vector<trajectory::Trajectory<float>> &traj,
                             std::ofstream &file, const std::string &frame) {

  // TODO(xuning@cmu.edu): Make this a parameter.
  float dt = 0.1;

  Eigen::Matrix<float, 3, Eigen::Dynamic> pos;
  Eigen::Matrix<float, 3, Eigen::Dynamic> vel;
  for (auto &tj : traj) {
    if (frame == "WORLD")
      tj.GetWaypointsWorld(dt, &pos, &vel);
    else if (frame == "LOCAL")
      tj.GetWaypointsLocal(dt, &pos, &vel);
    file << pos.format(CSVFormat);
  }
}

// Given a set of trajectories `traj`, write them to the output file `file`.
void
WriteTrajectoriesToFile(const std::vector<trajectory::Trajectory<float>> &traj,
                             std::ofstream &file) {
  for (auto &tj : traj) file << tj.Z().format(CSVFlatMatrixFormat);
}

// Writes an Eigen::Vector3d `pt` as a single row to file, in CSV format.
void WritePointToFile(const Eigen::Vector3f &pt, std::ofstream &file) {
  file << pt.format(CSVFormat);
}

// Flattens flow field Eigen::Matrix Vx and Vy, and writes them in two stacked
// rows to `file`, in CSV format.
void WriteFlowFieldToFile(const Eigen::MatrixXf &Vx, const Eigen::MatrixXf &Vy,
                          std::ofstream &file) {
  file << Vx.format(CSVFlatMatrixFormat);
  file << Vy.format(CSVFlatMatrixFormat);
}

} // namespace data_handling
