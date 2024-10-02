global r m J M I l g Ea N Ra La Km eta tm;
r = 0.021;  %radius of wheels (m)
m = 0.063;    %mass of wheels and motors (kg)
J = 5.56e-5;   %moment of inertia about axle (kg*m^2)
M = 0.225;      %mass of chassis (kg)
I = 1e-3;   %moment of inertia about COM (kg*m^2)
l = 0.095;    %COM to wheel axle (m)
g = 9.81;   %gravity (m/s^2)
Ea = 12;    %input voltage (V)
N = 50;     %gear ratio (-)
Ra = 14.69;    %armature resistance (Ohms)
La = 2.5e-3;%armature inductance (H)
Km = .191;   %motor speed constant (V/rad/s)
eta = .60;   %gearbox efficiency (%)
tm = 0.0095;   %friction torque (N*m) 
q0 = [0;2];
dq0 = [0;0];

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (-l*m*g*(J + m*r^2 + m^2*l*M*m))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0;
     0, (l*M*g*(J + M*r^2 + m*r^2))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0];
  B = [
    0;
    0;
    (I + J + l^2*M + M*r^2 + m*r^2 + 2*l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2);
    -(J + M*r^2 + m*r^2 + l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2)
];
qdq = [0;2;0;0];
u = [0];

