#include <robot_model/robot_model.h>
#include <dynamics_identify/dynamics_identify.h>
#include <nlopt.h>
#include <iostream>
#include <fstream>
#include <math.h>

int main(int argc, char ** argv)
{
    robot_dyn::RobotModel robot;
    const unsigned int dof = 7;

    robot.InitModel(dof);

    // standard DH parameters
    MatrixXd DH = MatrixXd::Zero(4,dof);
    DH << 0.317, 0.1925, 0.4, -0.1685, 0.4, 0.1363, 0.13375,            // d
          M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, 0,            // alpha
          -0.081, 0, 0, 0, 0, 0, 0,                                     // a
          M_PI, -M_PI_2, M_PI, M_PI, M_PI, M_PI, -M_PI_2;               // offset
    Matrix<Vector3d,1,Dynamic> P;
    P.resize(dof);
    P(0) << 0.081, 0, 0.317;
    P(1) << 0, 0, 0.1925;
    P(2) << 0, 0, 0.4;
    P(3) << 0, 0, -0.1685;
    P(4) << 0, 0, 0.4;
    P(5) << 0, 0, 0.1363;
    P(6) << 0, 0, 0.13375;
    robot.SetKinematicsParameters(DH, P);

    robot.qMin << -3.0503, -2.2736, -3.0426, -3.0439, -2.9761, -2.9761, -3.14;
    robot.qMax << 3.0503, 2.2736, 3.0426, 3.0439, 2.9761, 2.9761, 3.14;
    robot.qDotMin << -1.74, -1.328, -1.957, -1.957, -3.485, -3.485, -4.545;
    robot.qDotMax << 1.74, 1.328, 1.957, 1.957, 3.485, 3.485, 4.545;
    robot.qDDotMin << -10.0, -8.0, -10.0, -10.0, -12.0, -12.0, -12.0;
    robot.qDDotMax << 10.0, 8.0, 10.0, 10.0, 12.0, 12.0, 12.0;

    robot_iden::qr_decompose(&robot);

    robot_iden::Fourier fourier(robot);
    fourier.wf = 0.05*2*M_PI; 
    fourier.Tf = 20;
    fourier.h = 0.1;

    unsigned int x_num = fourier.num*robot.dof;

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, x_num);
    nlopt_set_min_objective(opt, robot_iden::optimal_object_fun, &fourier);

    unsigned m_ineq = robot.dof*3;
    double tol_ineq[m_ineq];
    for (unsigned int i=0; i<m_ineq; i++)
    {
        tol_ineq[i]=1.0e-8;
    }
    nlopt_add_inequality_mconstraint(opt, m_ineq, 
        robot_iden::inequality_constraint, &fourier, tol_ineq);

    // Another version of inequality constraint
    /*unsigned int count = (int)(fourier.Tf/fourier.h);
    unsigned m_ineq = robot.dof*3*2*(count+1);
    double tol_ineq[m_ineq];
    for (unsigned int i=0; i<m_ineq; i++)
    {
        tol_ineq[i]=1.0e-8;
    }
    nlopt_add_inequality_mconstraint(opt, m_ineq, 
        robot_iden::inequality_constraint_v1, &fourier, tol_ineq);*/

    unsigned m_eq = robot.dof*3*2;
    double tol_eq[m_eq];
    for (unsigned int i=0; i<m_eq; i++)
    {
        tol_eq[i]=1.0e-8;
    }
    nlopt_add_equality_mconstraint(opt, m_eq, robot_iden::equality_constraint, &fourier, tol_eq);

    nlopt_set_xtol_rel(opt, 1.0e-4);

    VectorXd xx = VectorXd::Ones(x_num);
    xx = 3*xx;
    double x[x_num];
    for (unsigned int i=0; i<x_num; i++)
    {
        x[i] = xx(i);
    }

    double min_f;
    nlopt_result result = nlopt_optimize(opt, x, &min_f);

    if (result < 0) 
    {
        std::cout << "Failed to find a minimum resolution." << std::endl 
            << "Error code: " << result << std::endl;
    }
    else 
    {
        std::cout << "Find minimum resolution successfully." << std::endl;
        std::cout << "Minimum object function: " << min_f << std::endl;
    }

    nlopt_destroy(opt);

    std::ofstream outfile;
    outfile.open("optimal_x.txt");
    outfile << "number of basic dynamics parameters (Pb_num): " << robot.Pb_num << std::endl;
    outfile << "initial value of x(i) is: " << xx(0) << std::endl;
    outfile << "optimal value of x is:" << std::endl;
    outfile << "[";
    for (unsigned int i=0; i<x_num; i++)
    {
        outfile << x[i];

        if (i!=(x_num-1))
        {
            outfile << ", ";
        }
    }
    outfile << "]" << std::endl;
    outfile.close();

    return 0;
}
