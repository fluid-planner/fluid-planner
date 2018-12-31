// Copyright 2018 Toyota Research Institute.  All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
                                       Eigen::DontAlignCols, ",", "
", "", "",
                                       "", "
");
const static Eigen::IOFormat CSVFlatMatrixFormat(Eigen::StreamPrecision,
                                      Eigen::DontAlignCols, ",", ",", "", "",
                                      "", "
");

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
  for (auto &tj : traj)
    file << tj.Z().format(CSVFlatMatrixFormat);
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
