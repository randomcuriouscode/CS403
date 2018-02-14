#include <algorithm>
#include <iostream>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using std::cout;

void EigenExamples() {
  cout << "Eigen Examples\n";

  // Declare a 3D vector.
  Vector3f v1;

  // Set coordinate values to the vector in 2 different ways.
  v1 = Vector3f(1.0, 2.0, 3.0);
  cout << "\nv1:\n" << v1 << "\n";
  v1.x() = 4.0;
  v1.y() = 5.0;
  v1.z() = 6.0;
  cout << "\nv1:\n" << v1 << "\n";

  // Access coordinate values of the vector in 2 different ways.
  Vector3f v2 = v1;
  v2.x() = v1.x();
  v2.y() = v1.y();
  v2.z() = v1.z();
  cout << "\nv2:\n" << v2 << "\n";

  // Arithmetic with vectors.
  Vector3f v3 = v1 + v2;
  Vector3f v4 = v1 - v2;
  v3 = 0.5 * v1;
  cout << "\nv3:\n" << v3 << "\n\nv4:\n" << v4 << "\n";

  // Get the length (norm) of a vector.
  float v1_norm = v1.norm();
  cout << "\nv1_norm:\n" << v1_norm << "\n";

  // Putting things together, this normalizes the vector:
  v3 = v1 / v1.norm();

  // Common vector operations.
  float dot_product_value = v1.dot(v2);
  Vector3f cross_product_value = v1.cross(v2);
  cout << "\ndot product:\n" << dot_product_value << "\n"
       << "\ncross product:\n" << cross_product_value << "\n";

  // Declare a 3x3 matrix.
  Matrix3f M;

  // Assign values to the matrix, in this case, set it to a scaling matrix.
  // Method 1:
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j) {
        M(i, j) = 2.0;
      } else {
        M(i, j) = 0.0;
      }
    }
  }
  cout << "\nM:\n" << M << "\n";
  // Method 2:
  M = 2.0 * Matrix3f::Identity();

  // Multiply a matrix with a vector.
  v2 = M * v1;
  cout << "\nv2:\n" << v2 << "\n";

  // You can do multiple operations on one line.
  v3 = M * v1 + v2 + v1.cross(v2);
  cout << "\nv3:\n" << v3 << "\n";

  // Set a single cell of M.
  M(0,2) = 1;
  // Take the transpose of a vector or a matrix.
  cout << "\nv1 transpose:\n" << v1.transpose() << "\n"
       << "\nM:\n" << M << "\n"
       << "\nM transpose:\n" << M.transpose() << "\n";

  // You can perform operations that interchange sizes, as long as the math is
  // valid.
  M = v1 * v1.transpose();

  // Reset M to a simple matrix to interpret the results of what follows.
  M = 2.0 * Matrix3f::Identity();

  // Initialize the eigenvalue solver.
  Eigen::EigenSolver<Matrix3f> eigen_solver;
  // Ask the solver to compute the eigen values and vectors.
  eigen_solver.compute(M);
  // Get all the eigenvalues. Note that the result is complex-valued, and
  // here we are asking for only the real component by calling ".real()" over
  // the result.
  Vector3f eigen_values = eigen_solver.eigenvalues().real();
  // Get all the eigenvectors. Note that the result is complex-valued, and
  // here we are asking for only the real component by calling ".real()" over
  // the result.
  Matrix3f eigen_vectors = eigen_solver.eigenvectors().real();

  // Print out all the eigenvlaues and eigenvectors.
  for (int i = 0; i < 3; ++i) {
    const float eigen_value = eigen_values(i);
    const Vector3f eigen_vector = eigen_vectors.col(i);
    cout << "\nEigen value " << i << ": " << eigen_value
         << "\nEigen vector " << i << ":\n" << eigen_vector << "\n";
  }
}

int main(int argc, char **argv) {
  EigenExamples();
  return 0;
}
