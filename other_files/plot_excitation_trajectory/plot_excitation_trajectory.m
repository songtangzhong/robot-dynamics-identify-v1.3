%%
dof=7;
wf=0.05*2*pi;
Tf=20;
h=0.1;
opt_x=[-0.0340937, 0.032194, -0.110333, 0.212013, -0.09978, 0.0259812, 0.181442, 0.751157, 0.0044091, -0.531994, 0.833307, -0.0753031, -0.176557, -0.0654039, 0.539726, -0.222462, 0.0462238, -0.213379, -0.0298595, 0.307615, -0.152069, -0.0761685, 0.236907, 0.110032, -0.131153, -0.475255, 0.259468, 0.0495177, 0.113938, 0.402873, 0.27919, -0.520554, 0.657195, -0.129425, 0.355469, -0.472935, 0.189949, 0.0569416, -0.129034, -0.191965, 0.205588, 0.254642, -0.224474, -0.438381, -0.0381377, 0.179177, 0.410617, -0.734283, 0.182627, -0.0698881, -0.048349, 1.01711, -0.946798, 0.180491, 0.14124, -0.113764, 0.0680989, -0.223912, 0.563557, -0.29398, 0.0425983, -0.165047, -0.609511, 0.899269, -0.296209, -0.246754, -0.034929, 0.374029, -0.647734, 0.162078, 0.146556, -0.00251329, -0.165276, 0.0898469, 0.481889, -0.372806, -0.0295752];

count=Tf/h;
opt_q=zeros(count+1,dof);
opt_dq=zeros(count+1,dof);
opt_ddq=zeros(count+1,dof);

t=-h;
for k=1:(count+1)
    t=t+h;
    [opt_q(k,:),opt_dq(k,:),opt_ddq(k,:)]=generate_fourier_trajectory(opt_x,wf,t); 
end

time=0:h:Tf;
close all;
figure;
plot(time,opt_q(:,1),time,opt_q(:,2),time,opt_q(:,3),time,opt_q(:,4),time,opt_q(:,5),time,opt_q(:,6),time,opt_q(:,7),'Linewidth',2);
grid on;
legend('$q_{1}$','$q_{2}$','$q_{3}$','$q_{4}$','$q_{5}$','$q_{6}$','$q_{7}$','Interpreter','Latex');

figure;
plot(time,opt_dq(:,1),time,opt_dq(:,2),time,opt_dq(:,3),time,opt_dq(:,4),time,opt_dq(:,5),time,opt_dq(:,6),time,opt_dq(:,7),'Linewidth',2);
grid on;
legend('$\dot{q}_{1}$','$\dot{q}_{2}$','$\dot{q}_{3}$','$\dot{q}_{4}$','$\dot{q}_{5}$','$\dot{q}_{6}$','$\dot{q}_{7}$','Interpreter','Latex');

figure;
plot(time,opt_ddq(:,1),time,opt_ddq(:,2),time,opt_ddq(:,3),time,opt_ddq(:,4),time,opt_ddq(:,5),time,opt_ddq(:,6),time,opt_ddq(:,7),'Linewidth',2);
grid on;
legend('$\ddot{q}_{1}$','$\ddot{q}_{2}$','$\ddot{q}_{3}$','$\ddot{q}_{4}$','$\ddot{q}_{5}$','$\ddot{q}_{6}$','$\ddot{q}_{7}$','Interpreter','Latex');








