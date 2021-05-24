#include <robot_model/robot_model.h>
#include <math.h>

namespace robot_dyn
{
Matrix3d Rot(const double theta, const double alpha)
{
    Matrix3d R;

    R << cos(theta),   -sin(theta)*cos(alpha),    sin(theta)*sin(alpha),             
         sin(theta),    cos(theta)*cos(alpha),   -cos(theta)*sin(alpha),
         0,             sin(alpha),               cos(alpha);

    return R;
}

Vector3d Trans(const double theta, const double d, const double a)
{
    Vector3d T;

    T << a*cos(theta), a*sin(theta), d;

    return T;
}

void RobotModel::InitModel(const unsigned int DOF)
{
    dof = DOF;

    Psi_num = 10;

    Ps_num = Psi_num*dof;

    Ps_flag.resize(Ps_num);

    Pb_num = 0;

    qMin.resize(dof);
    qMax.resize(dof);
    qDotMin.resize(dof);
    qDotMax.resize(dof);
    qDDotMin.resize(dof);
    qDDotMax.resize(dof);

    qr_threshold = 1e-10;

    theta.resize(dof);
    d.resize(dof);
    alpha.resize(dof);
    a.resize(dof);
    offset.resize(dof);

    Z << 0, 0, 1;

    m.resize(dof);

    I.resize(dof);
    Ic.resize(dof);
    Pc.resize(dof);

    P.resize(dof+1);
    R.resize(dof+1);
    R_T.resize(dof+1);

    w.resize(dof+1);
    wDot.resize(dof+1);
    vDot.resize(dof+1);

    vcDot.resize(dof);
    F.resize(dof);
    N.resize(dof);

    f.resize(dof+1);
    n.resize(dof+1);

    tau.resize(dof);

    w(0) << 0, 0, 0;
    wDot(0) << 0, 0, 0;
    vDot(0) << 0, 0, g;
    
    f(dof) << 0, 0, 0;
    n(dof) << 0, 0, 0;
}

void RobotModel::SetKinematicsParameters(const MatrixXd param)
{
    d = param.row(0).transpose();
    alpha = param.row(1).transpose();
    a = param.row(2).transpose();
    offset = param.row(3).transpose();
}

// [mi rcxi rcyi rczi Ixxi Ixyi Ixzi Iyyi Iyzi Izzi]'
void RobotModel::SetDynamicsParameters(const VectorXd param)
{
    for (unsigned int i=0; i<dof; i++)
    {
        m(i) = param(i*Psi_num+0);

        Pc(i) << param(i*Psi_num+1), param(i*Psi_num+2), param(i*Psi_num+3);
    
        I(i) <<  param(i*Psi_num+4), -param(i*Psi_num+5), -param(i*Psi_num+6),
                -param(i*Psi_num+5),  param(i*Psi_num+7), -param(i*Psi_num+8),
                -param(i*Psi_num+6), -param(i*Psi_num+8),  param(i*Psi_num+9);

        Ic(i) = I(i)-m(i)*(Pc(i).transpose()*Pc(i)*I33-Pc(i)*Pc(i).transpose());
    }
}

VectorXd RobotModel::calcu_inv_dyn(const VectorXd q, const VectorXd qDot, const VectorXd qDDot)
{
    theta = q+offset;

    for (unsigned int i=0; i<dof; i++)
    {
        R(i) = Rot(theta(i), alpha(i));
        R_T(i) = R(i).transpose();
    }

    R(dof) = Rot(0.0, 0.0);
    R_T(dof) = R(dof).transpose();

    for (unsigned int i=0; i<dof; i++)
    {
        P(i) = Trans(theta(i), d(i), a(i));
    }

    P(dof) = Trans(0.0, 0.0, 0.0);

    vDot(0) << 0, 0, g;

    for (unsigned int i=1; i<=dof; i++)
    {
        w(i) = R(i-1)*w(i-1)+qDot(i-1)*Z;
        wDot(i) = R(i-1)*wDot(i-1)+R(i-1)*w(i-1).cross(qDot(i-1)*Z)+qDDot(i-1)*Z;
        vDot(i) = R(i-1)*(wDot(i-1).cross(P(i-1))+w(i-1).cross(w(i-1).cross(P(i-1)))+vDot(i-1));
        vcDot(i-1) = wDot(i).cross(Pc(i-1))+w(i).cross(w(i).cross(Pc(i-1)))+vDot(i);
        F(i-1) = m(i-1)*vcDot(i-1);
        N(i-1) = Ic(i-1)*wDot(i)+w(i).cross(Ic(i-1)*w(i));
    }
    
    for (unsigned int i=dof; i>0; i--)
    {
        f(i-1) = R_T(i)*f(i)+F(i-1);
        n(i-1) = N(i-1)+R_T(i)*n(i)+Pc(i-1).cross(F(i-1))+P(i).cross(R_T(i)*f(i));
        tau(i-1) = n(i-1).transpose()*Z;
    }

    return tau;
}

}









