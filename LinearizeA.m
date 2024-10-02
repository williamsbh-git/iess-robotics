function A = LinearizeA(~)
    
    global r m J M I l g;
    A = [0,0,1,0;0,0,0,1;0,-(l*m*g*(J+m*r^2+m^2+l*M*m))/(I*J+J*L^2*M+I*M*r^2+I*m*r^2+L^2*M*m*r^2),0,0;0,(l*M*g(J+M*r^2+m*r^2))/(I*J+J*l^2*M+I*M*r^2+I*m*r^2+l^2*M*m*r^2),0,0];
    
end