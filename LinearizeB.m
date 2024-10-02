function B = LinearizeB(qdq)

    theta = qdq(2);
    global r m J M I l;
  B = [
    0;
    0;
    (I + J + l^2*M + M*r^2 + m*r^2 + 2*l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2);
    -(J + M*r^2 + m*r^2 + l*M*r*cos(0)) / (I*J + l^2*M^2*r^2 + J*l^2*M + I*M*r^2 + I*m*r^2 + l^2*M*m*r^2 - l^2*M^2*r^2*cos(0)^2)
];

end