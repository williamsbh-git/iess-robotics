function T = torque(kmia)

    eta = 0.6;
    N = 50;
    tm = 0.0095;

    if kmia < -tm
        T = eta*N*(kmia+tm);
    elseif kmia > tm
        T = eta*N*(kmia-tm);
    elseif kmia <= tm && kmia >= -tm
        T = 0;
    end

end