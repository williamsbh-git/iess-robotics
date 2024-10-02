function C = CorMatrix(qdq)

    global r M l;

    phi = qdq(1);
    theta = qdq(2);
    dphi = qdq(3);
    dtheta = qdq(4);

    C = zeros(2,2);

    C(1,1) = 0;
    C(1,2) = M*l*r*dtheta*sin(theta);
    C(2,1) = 0;
    C(2,2) = C(1,2);
    
end