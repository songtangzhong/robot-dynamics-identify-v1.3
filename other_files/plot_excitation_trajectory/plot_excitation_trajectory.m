%%
dof=7;
wf=0.05*2*pi;
Tf=20;
h=0.1;
opt_x=[-0.0508343, 0.00803749, 0.0687807, -0.0156053, -0.0103786, -0.136311, -0.163288, 0.0230056, 0.074998, 0.0187757, -0.597729, -0.0110972, 0.0268443, -0.0240648, 0.00879294, -0.000475192, 0.00669272, 0.00324295, -0.0170549, 0.01704, -0.00603474, 0.0180872, 0.0104994, -0.0248465, 0.0217965, -0.00800174, 0.000552258, -0.00448546, -0.000703155, 0.0148642, -0.0185881, 0.0071303, -0.00987801, 0.0782184, -0.161459, 0.114422, -0.0357602, 0.00457938, -0.0849552, -0.0757644, 0.11567, -0.0833047, 0.0445385, -0.306211, -0.0533871, -0.192545, 0.31608, -0.0158523, -0.0542961, -0.164077, -0.0373321, 0.295428, 0.227019, -0.311124, -0.285643, 0.380282, 0.00636954, -0.270954, -0.534734, 0.419037, 0.0176577, -0.154255, 0.33665, -0.0130243, -0.1334, 0.0726085, -0.0636554, -0.126564, 0.0952122, 0.0946308, 0.000376545, -0.0532172, -0.480997, -0.0555527, -0.278685, 0.459322, -0.923227];

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








