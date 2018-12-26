function vs = side_vector2(pos,X)
    k = X(1); e = X(2);
    %e = pi/60; 
    phi = pos(1); theta = pos(2); psi = pos(3);
    J = [cos(psi)*cos(theta)  -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
         sin(psi)*cos(theta)  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi)   -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
            -sin(theta)          cos(theta)*sin(phi)                              cos(theta)*cos(phi)];
    vss = [sin(e)*sin(k);cos(e);sin(e)*cos(k)];
    vs = J*vss;
end