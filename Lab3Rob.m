A =  [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (-l*m*g*(J + m*r^2 + m^2*l*M*m))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0;
     0, (l*M*g*(J + M*r^2 + m*r^2))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0];
B = [
    0;
    0;
    (I + J + l^2*M + M*r^2 + m*r^2 + 2*l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2);
    -(J + M*r^2 + m*r^2 + l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2)];

C = eye(4);
D = zeros(4,1);

Q = diag([100,10000000000,10000,10000000000]);
R = 0.001;

K = lqr(A,B,Q,R);

xdot = A - B*K;

eigs = eig(xdot);
display(eigs);

sys = ss(A,B,C,D);
zeroes = zero(sys);
poles = pole(sys);


