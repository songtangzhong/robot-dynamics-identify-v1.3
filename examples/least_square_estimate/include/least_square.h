#ifndef LEAST_SQUARE_H_
#define LEAST_SQUARE_H_

#include <robot_model/robot_model.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace ls_algo
{
class Identifier
{
public:
    Identifier(robot_dyn::RobotModel *robot_);
    ~Identifier();

    void read_mat_file();
    void cunstruct_data();
    void calcu_dynamics_parameters();

    char *q_file_path = NULL;
    char *q_variable_name = NULL;
    char *qDot_file_path = NULL;
    char *qDot_variable_name = NULL;
    char *qDDot_file_path = NULL;
    char *qDDot_variable_name = NULL;
    char *tau_file_path = NULL;
    char *tau_variable_name = NULL;

    MatrixXd q_filter;
    MatrixXd qDot_filter;
    MatrixXd qDDot_filter;
    MatrixXd tau_filter;

    MatrixXd W;
    VectorXd tau;

    MatrixXd Wb;

private:
    robot_dyn::RobotModel *robot;
};

}

#endif