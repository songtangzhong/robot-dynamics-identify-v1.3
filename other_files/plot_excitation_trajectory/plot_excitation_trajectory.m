%%
dof=7;
wf=0.05*2*pi;
Tf=20;
h=0.1;
opt_x=[0.000817113, 2.68821, 4.15945, 4.47061, 4.73049, 0.00199528, 2.71466, 3.96276, 4.40619, 4.93574, 2.42349, 0.670271, 2.30754, 3.61484, 4.11225, 4.64738, -0.27755, 1.55846, 2.93269, 3.85218, 4.29712, 3.02141, -0.000270171, 2.3874, 3.73917, 4.11703, 4.50281, -0.000682836, 2.93968, 3.98183, 4.51195, 4.67333, 3.16658, -5.46489e-07, 2.50019, 3.95861, 4.375, 4.66575, -7.21594e-05, 2.71323, 4.10584, 4.48033, 4.81953, 2.8084, -5.22621e-05, 2.69441, 3.82094, 4.52684, 3.31703, -3.32229e-06, 2.91357, 3.44141, 4.62826, 4.82493, 3.26616, 0.00104528, 2.72854, 3.86693, 4.13656, 4.49156, -0.000901511, 2.47799, 3.85052, 4.48853, 4.66088, 3.27847, -2.87356e-05, 2.8938, 4.00834, 4.38545, 4.54882, -0.000930248, 2.68089, 3.73689, 4.17806, 4.77281, 2.96787];

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








