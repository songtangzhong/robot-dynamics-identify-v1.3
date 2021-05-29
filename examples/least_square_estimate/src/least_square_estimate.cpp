#include <least_square.h>
#include <robot_model/robot_model.h>
#include <Eigen/Dense>
#include <mat.h>
#include <iostream>
#include <fstream>

using namespace Eigen;

int main(int argc, char ** argv)
{
    robot_dyn::RobotModel robot;
    const unsigned int dof = 7;

    robot.InitModel(dof);

    // Modified MDH parameters
    MatrixXd MDH = MatrixXd::Zero(4,dof);
    MDH << 0, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, -M_PI_2,            // alpha
           0, 0.081, 0, 0, 0, 0, 0,                                          // a
           0, 0.1925, 0.4, -0.1685, 0.4, 0.1363, 0,                          // d
           0, -M_PI_2, 0, 0, 0, 0, 0;                                        // offset
    Matrix<Vector3d,1,Dynamic> P;
    P.resize(dof);
    P(0) << 0, 0, 0;
    P(1) << 0.081, 0.1925, 0;
    P(2) << 0, 0.4, 0;
    P(3) << 0, 0.1685, 0;
    P(4) << 0, 0.4, 0;
    P(5) << 0, -0.1363, 0;
    P(6) << 0, 0, 0;
    robot.SetKinematicsParameters(MDH, P);

    ls_algo::Identifier identifier(&robot);

    identifier.q_file_path = (char*)"/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/filter_data/q_filter.mat";
    identifier.q_variable_name = (char*)"q_filter";
    identifier.qDot_file_path = (char*)"/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/filter_data/qDot_filter.mat";
    identifier.qDot_variable_name = (char*)"qDot_filter";
    identifier.qDDot_file_path = (char*)"/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/filter_data/qDDot_filter.mat";
    identifier.qDDot_variable_name = (char*)"qDDot_filter";
    identifier.tau_file_path = (char*)"/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/filter_data/tau_filter.mat";
    identifier.tau_variable_name = (char*)"tau_filter";
    
    identifier.read_mat_file();
    identifier.cunstruct_data();
    identifier.calcu_dynamics_parameters();

    ////////////////////////////////////////////////////////////////////////////
    std::ofstream outfile;
    outfile.open("/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/identify_result/base_dynamics_parameters.txt");
    outfile << "base dynamics parameters:" << std::endl;
    outfile << "[";
    for (unsigned int i=0; i<robot.Pb_num; i++)
    {
        outfile << robot.Pb(i);

        if (i!=(robot.Pb_num-1))
        {
            outfile << ", ";
        }
    }
    outfile << "]" << std::endl;
    outfile.close();

    outfile.open("/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/identify_result/standard_dynamics_parameters.txt");
    outfile << "standard dynamics parameters:" << std::endl;
    outfile << "[";
    for (unsigned int i=0; i<robot.Ps_num; i++)
    {
        outfile << robot.Ps(i);

        if (i!=(robot.Ps_num-1))
        {
            outfile << ", ";
        }
    }
    outfile << "]" << std::endl;
    outfile.close();

    ////////////////////////////////////////////////////////////////////////////
    /*VectorXd tau = VectorXd::Zero(identifier.Wb.rows());
    tau = identifier.Wb*robot.Pb;
    unsigned int row = identifier.tau_filter.rows();
    unsigned int col = identifier.tau_filter.cols();
    MatrixXd tau_iden = MatrixXd::Zero(row,col);
    for (unsigned int i=0; i<row; i++)
    {
        tau_iden.row(i) = tau.segment(i*col, col).transpose();
    }
    double *ptr = new double[row*col];
    for (int i=0; i<row; i++)
    {
        for (int j=0; j<col; j++)
        {
            ptr[j*row+i]=tau_iden(i,j);
        }
    }*/

    ////////////////////////////////////////////////////////////////////////////
    robot.SetDynamicsParameters(robot.Ps);
    unsigned int row = identifier.tau_filter.rows();
    unsigned int col = identifier.tau_filter.cols();
    MatrixXd tau_iden = MatrixXd::Zero(row,col);
    for (unsigned int i=0; i<row; i++)
    {
        tau_iden.row(i) = robot.calcu_inv_dyn(identifier.q_filter.row(i).transpose(), 
            identifier.qDot_filter.row(i).transpose(), identifier.qDDot_filter.row(i).transpose()).transpose();
    }
    double *ptr = new double[row*col];
    for (int i=0; i<row; i++)
    {
        for (int j=0; j<col; j++)
        {
            ptr[j*row+i] = tau_iden(i,j);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    MATFile *pmatFile = NULL;
    mxArray *pMxArray = NULL;
    pmatFile = matOpen("/home/stz/robot-dynamics-identify-v1.3/examples/least_square_estimate/identify_result/tau_iden.mat","w");
    pMxArray = mxCreateDoubleMatrix(row, col, mxREAL);
    mxSetData(pMxArray, ptr);
    matPutVariable(pmatFile, "tau_iden", pMxArray);
    matClose(pmatFile);

    return 0;
}


