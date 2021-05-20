#include <robot_model/robot_model.h>
#include <math.h>

namespace robot_dyn
{
Matrix3d Rot(const double theta, const double alpha)
{
    Matrix3d R;

    R << cos(theta),    -sin(theta)*cos(alpha),    sin(theta)*sin(alpha),
         sin(theta),     cos(theta)*cos(alpha),   -cos(theta)*sin(alpha),
         0,              sin(alpha),               cos(alpha);

    return R;
}

Vector3d Trans(const double theta, const double d, const double r)
{
    Vector3d T;

    T << r*cos(theta), r*sin(theta), d;

    return T;
}

void RobotModel::InitModel(const unsigned int DOF)
{
    dof = DOF;

    theta.resize(dof);
    d.resize(dof);
    alpha.resize(dof);
    r.resize(dof);
    offset.resize(dof);

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

    m.resize(dof);

    mrcx.resize(dof);
    mrcy.resize(dof);
    mrcz.resize(dof);

    Ixx.resize(dof);
    Ixy.resize(dof);
    Ixz.resize(dof);
    Iyy.resize(dof);
    Iyz.resize(dof);
    Izz.resize(dof);

    g = -9.81;

    Z << 0, 0, 1;

    mrc.resize(dof);

    I.resize(dof);

    P.resize(dof+1);

    R.resize(dof+1);
    R_T.resize(dof+1);

    w.resize(dof+1);
    wDot.resize(dof+1);
    a.resize(dof+1);
    F.resize(dof+1);
    f.resize(dof+1);
    n.resize(dof+1);

    w(0) << 0, 0, 0;
    wDot(0) << 0, 0, 0;
    a(0) << 0, 0, g;
    
    f(dof) << 0, 0, 0;
    n(dof) << 0, 0, 0;

    tau.resize(dof);
}

void RobotModel::SetKinematicsParameters(const MatrixXd param)
{
    d = param.row(0).transpose();
    alpha = param.row(1).transpose();
    r = param.row(2).transpose();
    offset = param.row(3).transpose();
}

void RobotModel::SetDynamicsParameters(const VectorXd param)
{
    for (unsigned int i=0; i<dof; i++)
    {
        m(i) = param(i*Psi_num+0);
        mrcx(i) = param(i*Psi_num+1);
        mrcy(i) = param(i*Psi_num+2);
        mrcz(i) = param(i*Psi_num+3);
        Ixx(i) = param(i*Psi_num+4);
        Ixy(i) = param(i*Psi_num+5);
        Ixz(i) = param(i*Psi_num+6);
        Iyy(i) = param(i*Psi_num+7);
        Iyz(i) = param(i*Psi_num+8);
        Izz(i) = param(i*Psi_num+9);

        mrc(i) << mrcx(i), mrcy(i), mrcz(i);

        I(i) << Ixx(i), Ixy(i), Ixz(i),
                Ixy(i), Iyy(i), Iyz(i),
                Ixz(i), Iyz(i), Izz(i);
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
        P(i) = Trans(theta(i), d(i), r(i));
    }

    P(dof) = Trans(0.0, 0.0, 0.0);

    a(0) << 0, 0, g;

    for (unsigned int i=1; i<=dof; i++)
    {
        w(i) = R(i-1)*w(i-1)+qDot(i-1)*Z;
        wDot(i) = R(i-1)*wDot(i-1)+R(i-1)*w(i-1).cross(qDot(i-1)*Z)+qDDot(i-1)*Z;
        a(i) = R(i-1)*(wDot(i-1).cross(P(i-1))+w(i-1).cross(w(i-1).cross(P(i-1)))+a(i-1));
        F(i) = wDot(i).cross(mrc(i-1))+w(i).cross(w(i).cross(mrc(i-1)))+m(i-1)*a(i);
    }
    
    for (int i=dof-1; i>=0; i--)
    {
        f(i)=R_T(i+1)*f(i+1)+F(i+1);
        n(i)=R_T(i+1)*n(i+1)+P(i+1).cross(R_T(i+1)*f(i+1))+I(i)*wDot(i+1)+w(i+1).cross(I(i)*w(i+1))-a(i+1).cross(mrc(i));
        tau(i)=n(i).transpose()*Z;
    }

    return tau;
}

}









