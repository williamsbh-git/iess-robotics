function MM = MassMatrix(q)

    global r m J M I l;

    phi = q(1);
    theta = q(2);
   
    MM = zeros(2,2);

    MM(1,1) = J + (M + m)*r^2;
    MM(1,2) = J + (M+m)*r^2 + M*l*r*cos(theta);
    MM(2,1) = J + (M+m)*r^2 + M*l*r*cos(theta);
    MM(2,2) = J + (M+m)*r^2 + 2*M*l*r*cos(theta) + I + M*l^2;

end