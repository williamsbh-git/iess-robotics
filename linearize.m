
clear;

syms theta phi thetadot phidot tau;
syms J M m r L I g;

global jacobianx jacobianu;

JMmr = J+(M+m)*r^2;
Mlr = M*L*r;
Ml2 = M*L^2;
Mgl = M*g*L;

M1 = [JMmr , (JMmr + Mlr*cos(theta)) ; (JMmr + Mlr *cos(theta)), (JMmr +2*Mlr*cos(theta) + (I+Ml2))];
C = [0, (-Mlr*thetadot*sin(theta)); 0, (-Mlr*thetadot*sin(theta))];
G = [0;-Mgl*sin(theta)];
Minv = inv(M1);

qdot = [phidot ; thetadot];
tor = [tau; 0];
x = [phi ; theta ; phidot ; thetadot];

f = [phidot ; thetadot ; Minv*(tor - (C*qdot) - G)];

jacobianx = jacobian(f,x);

subs(jacobianx, [theta phi thetadot phidot], [0 0 0 0]);

jacobianu = jacobian(f,tau);

subs(jacobianu, tau, 0);