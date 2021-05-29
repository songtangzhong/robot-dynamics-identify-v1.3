% %
close all;

figure;
plot(t,tau(:,1),t,tau_iden(:,1));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{1}$','$\tau_{1}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,2),t,tau_iden(:,2));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{2}$','$\tau_{2}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,3),t,tau_iden(:,3));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{3}$','$\tau_{3}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,4),t,tau_iden(:,4));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{4}$','$\tau_{4}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,5),t,tau_iden(:,5));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{5}$','$\tau_{5}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,6),t,tau_iden(:,6));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{6}$','$\tau_{6}$ iden','Interpreter','Latex');

figure;
plot(t,tau(:,7),t,tau_iden(:,7));
xlabel('time (sec)');
ylabel('torque');
legend('$\tau_{7}$','$\tau_{7}$ iden','Interpreter','Latex');












