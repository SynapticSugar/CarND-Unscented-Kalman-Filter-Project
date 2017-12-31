#ifndef TOOLS_H_
#define TOOLS_H_
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
   * A file stream to save the RMSE values.
   */
  std::ofstream rmse_fs_;

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                         const vector<VectorXd> &ground_truth);
};

#endif /* TOOLS_H_ */