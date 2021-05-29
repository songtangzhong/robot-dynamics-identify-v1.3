#include <least_square.h>
#include <mat.h>
#include <dynamics_identify/dynamics_identify.h>
#include <iostream>
#include <cmath>

namespace ls_algo
{
Identifier::Identifier(robot_dyn::RobotModel *robot_)
{
    robot = robot_;
}

Identifier::~Identifier(){}

void Identifier::read_mat_file()
{
    MATFile *pmatFile = NULL;
    mxArray *pMxArray = NULL;

    double *ptr = NULL;

    int m;
    int n;

    pmatFile = matOpen(q_file_path,"r");
    pMxArray = matGetVariable(pmatFile, q_variable_name);
    ptr = (double*) mxGetData(pMxArray);
    m = mxGetM(pMxArray);
    n = mxGetN(pMxArray);
    q_filter.resize(m,n);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            q_filter(i,j)=ptr[j*m+i];
        }
    }
    matClose(pmatFile);

    pmatFile = matOpen(qDot_file_path,"r");
    pMxArray = matGetVariable(pmatFile, qDot_variable_name);
    ptr = (double*) mxGetData(pMxArray);
    m = mxGetM(pMxArray);
    n = mxGetN(pMxArray);
    qDot_filter.resize(m,n);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            qDot_filter(i,j)=ptr[j*m+i];
        }
    }
    matClose(pmatFile);

    pmatFile = matOpen(qDDot_file_path,"r");
    pMxArray = matGetVariable(pmatFile, qDDot_variable_name);
    ptr = (double*) mxGetData(pMxArray);
    m = mxGetM(pMxArray);
    n = mxGetN(pMxArray);
    qDDot_filter.resize(m,n);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            qDDot_filter(i,j)=ptr[j*m+i];
        }
    }
    matClose(pmatFile);

    pmatFile = matOpen(tau_file_path,"r");
    pMxArray = matGetVariable(pmatFile, tau_variable_name);
    ptr = (double*) mxGetData(pMxArray);
    m = mxGetM(pMxArray);
    n = mxGetN(pMxArray);
    tau_filter.resize(m,n);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            tau_filter(i,j)=ptr[j*m+i];
        }
    }
    matClose(pmatFile);

    mxFree(ptr);
}

void Identifier::cunstruct_data()
{
    unsigned int count = q_filter.rows();

    W.resize(count*robot->dof,robot->Ps_num);
    tau.resize(count*robot->dof);

    for (unsigned int i=0; i<count; i++)
    {
        W.middleRows(i*robot->dof, robot->dof) =
            robot_iden::calcu_Ys(robot, q_filter.row(i).transpose(), 
                qDot_filter.row(i).transpose(), qDDot_filter.row(i).transpose());

        tau.segment(i*robot->dof, robot->dof) = tau_filter.row(i).transpose();
    }
}

void Identifier::calcu_dynamics_parameters()
{
    MatrixXd WW = MatrixXd::Zero(W.rows(),W.cols());
    WW = W;

    //normalization
    double factor;
    for (unsigned int i=0; i< robot->Ps_num; i++)
    {
        factor = W.col(i).norm();
        
        if (factor > 0.0)
        {
            W.col(i) = W.col(i)/factor;
        }
    }

    HouseholderQR<MatrixXd> qr;
    qr.compute(W);
    MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();

    /*
     * Following code must not be contained, which will lead to memory overflow.
     * This is due to too many simpling times to make the matrix "W" to large. 
     * Also, "Q" will be too large.
     */
    // MatrixXd Q = qr.householderQ();

    for (unsigned int i=0; i< robot->Ps_num; i++)
    {
        if (fabs(R(i,i))< robot->qr_threshold) 
        {
            robot->Ps_flag(i) = 0; 
        }
        else
        {
            robot->Ps_flag(i) = 1; 
            robot->Pb_num += 1;
        }
    }

    robot->R1.resize(robot->Pb_num, robot->Pb_num);
    robot->R2.resize(robot->Pb_num, robot->Ps_num-robot->Pb_num);
    Wb.resize(WW.rows(), robot->Pb_num);

    int j=-1;
    int k=-1;
    for (unsigned int i=0; i< robot->Ps_num; i++)
    {
        if (robot->Ps_flag(i) == 1)
        {
            j+=1;
            robot->R1.col(j) = R.block(0,i,robot->Pb_num,1);
            Wb.col(j) = WW.col(i);
        }
        else
        {
            k+=1;
            robot->R2.col(k) = R.block(0,i,robot->Pb_num,1);
        }
    }

    robot->Pb.resize(robot->Pb_num);
    robot->Pb = (Wb.transpose()*Wb).inverse()*Wb.transpose()*tau;

    // Pb = P1+inv(R1)*R2*P2
    VectorXd P1 = VectorXd::Zero(robot->Pb_num);
    VectorXd P2 = VectorXd::Zero(robot->Ps_num-robot->Pb_num);

    double m = 1.0;
    int P2_count = -1;
    for (unsigned int i=0; i< robot->Ps_num; i++)
    {
        if (robot->Ps_flag(i)==0)
        {
            P2_count += 1;
            if (i%robot->Psi_num==0)
            {
                P2(P2_count) = m;
            }
        }
    }

    P1 = robot->Pb-robot->R1.inverse()*robot->R2*P2;

    int P1_count = -1;
    P2_count = -1;
    for (unsigned int i=0; i< robot->Ps_num; i++)
    {
        if (robot->Ps_flag(i)==1)
        {
            P1_count += 1;
            robot->Ps(i) = P1(P1_count);
        }
        else
        {
            P2_count += 1;
            robot->Ps(i) = P2(P2_count);
        }
    }
}

}
