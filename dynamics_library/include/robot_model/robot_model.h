#ifndef ROBOT_MODEL_H_
#define ROBOT_MODEL_H_

#include <Eigen/Dense>

using namespace Eigen;

namespace robot_dyn
{
Matrix3d Rot(const double theta, const double alpha);

Vector3d Trans(const double theta, const double d, const double r);

class RobotModel
{
public:
    RobotModel(){};
    ~RobotModel(){};

    void InitModel(const unsigned int DOF);

    void SetKinematicsParameters(const MatrixXd param);

    void SetDynamicsParameters(const VectorXd param);

    VectorXd calcu_inv_dyn(const VectorXd q, const VectorXd qDot, const VectorXd qDDot);

    int dof;

    double g;

    // [mi mrcxi mrcyi mrczi Ixxi Ixyi Ixzi Iyyi Iyzi Izzi]'
    unsigned int Psi_num;

    unsigned int Ps_num;

    VectorXi Ps_flag;

    unsigned int Pb_num;

    VectorXd qMin; VectorXd qMax; 
    VectorXd qDotMin; VectorXd qDotMax; 
    VectorXd qDDotMin; VectorXd qDDotMax;

    MatrixXd R1;
    MatrixXd R2;
    double qr_threshold;

private:
    // Standard D-H parameters
    VectorXd theta;    // z rotation
    VectorXd d;        // z translation
    VectorXd alpha;    // x rotation
    VectorXd r;        // x translation
    VectorXd offset;

    VectorXd m;

    VectorXd mrcx; VectorXd mrcy; VectorXd mrcz;

    VectorXd Ixx; VectorXd Ixy; VectorXd Ixz; VectorXd Iyy; VectorXd Iyz; VectorXd Izz;

    Vector3d Z;

    Matrix<Vector3d,1,Dynamic> mrc;

    Matrix<Matrix3d,1,Dynamic> I;

    Matrix<Vector3d,1,Dynamic> P; 

    Matrix<Matrix3d,1,Dynamic> R;
    Matrix<Matrix3d,1,Dynamic> R_T;

    Matrix<Vector3d,1,Dynamic> w;
    Matrix<Vector3d,1,Dynamic> wDot;
    Matrix<Vector3d,1,Dynamic> a;
    Matrix<Vector3d,1,Dynamic> F;
    Matrix<Vector3d,1,Dynamic> f;
    Matrix<Vector3d,1,Dynamic> n;

    VectorXd tau;
};

}

#endif
