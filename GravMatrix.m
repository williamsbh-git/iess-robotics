function G = GravMatrix(q)

    global M g l;

    phi = q(1);
    theta = q(2);

    G = zeros(2,1);

    G(1,1) = 0;
    G(2,1) = M*g*l*sin(theta);

end