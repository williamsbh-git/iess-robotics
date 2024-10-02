function Aaxa = xaMatrix(x)

    r = 0.021;  %radius of wheels (m)
    m = 0.063;    %mass of wheels and motors (kg)
    J = 5.56e-5;   %moment of inertia about axle (kg*m^2)
    M = 0.225;      %mass of chassis (kg)
    I = 1e-3;   %moment of inertia about COM (kg*m^2)
    l = 0.095;    %COM to wheel axle (m)
    g = 9.81; 
    A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (-l*m*g*(J + m*r^2 + m^2*l*M*m))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0;
     0, (l*M*g*(J + M*r^2 + m*r^2))/(I*J + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2), 0, 0];
   
    C = eye(4);

    Aa = [A,zeros(4,1);-C,zeros(4,1)];

    xa = 



end
