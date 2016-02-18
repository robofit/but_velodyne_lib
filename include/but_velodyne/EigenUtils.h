/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef EIGENUTILS_HPP_
#define EIGENUTILS_HPP_

#include <iostream>
#include <fstream>
#include <exception>

namespace but_velodyne {

/**!
 * Additional operations with Eigen types and structures.
 */
class EigenUtils {

public:

  /**!
   * Save Eigen matrix to file
   *
   * @param filename destination file
   * @param m matrix to be saved
   */
  template<typename MatrixType>
  static void saveMatrix(const char *filename, const MatrixType& m)
  {
    typename MatrixType::Index rows, cols;
    rows = m.rows();
    cols = m.cols();
    typename MatrixType::Scalar const *data = m.data();

    std::ofstream f(filename, std::ios::binary);
    f.write((char *)&(rows), sizeof(m.rows()));
    f.write((char *)&(cols), sizeof(m.cols()));
    f.write((char *)data, sizeof(typename MatrixType::Scalar) * rows * cols);
    f.close();
  }

  /**!
   * Load Eigen matrix from file
   *
   * @param filename source file
   * @param m destination
   */
  template<typename MatrixType>
  static void loadMatrix(const char *filename, MatrixType& m)
  {
    typename MatrixType::Index rows, cols;
    std::ifstream f(filename, std::ios::binary);
    if(!f.is_open()) {
      std::cerr << "Unable to read matrix: " << filename << std::endl;
      exit(1);
    }
    f.read((char *)&rows, sizeof(rows));
    f.read((char *)&cols, sizeof(cols));
    //std::cerr << "rows: " << rows << " cols " << cols << std::endl;
    m.resize(rows, cols);
    f.read((char *)m.data(), sizeof(typename MatrixType::Scalar) * rows * cols);
    if (f.bad()) {
      std::cerr << "Unable to read matrix: " << filename << std::endl;
      throw std::exception();
    }
    f.close();
  }

  /**!
   * Save Eigen matrix to file
   *
   * @param filename destination file
   * @param m matrix to be saved
   */
  template<typename MatrixType>
  static void saveMatrix(const std::string filename, const MatrixType& m) {
    saveMatrix(filename.c_str(), m);
  }

  /**!
   * Load Eigen matrix from file
   *
   * @param filename source file
   * @param m destination
   */
  template<typename MatrixType>
  static void loadMatrix(const std::string filename, MatrixType& m) {
    loadMatrix(filename.c_str(), m);
  }

  /**!
   * Workarround for robodev1 (older Eigen)
   *
   * @return true if structure contains NaN values
   */
  template<typename Derived>
  inline static bool hasNaN(const Eigen::DenseBase<Derived> &o)
  {
    return !((o.derived().array()==o.derived().array()).all());
  }

  /**!
   * Workarround for robodev1 (older Eigen)
   *
   * @return true if structure contains INFINITE of NaN values
   */
  template<typename Derived>
  inline static bool allFinite(const Eigen::DenseBase<Derived> &o) {
    return !(hasNaN(o.derived()-o.derived()));
  }
};

}

#endif /* EIGENUTILS_HPP_ */
