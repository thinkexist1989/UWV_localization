%得到高度计波束方向向量
function v = get_vec(orivec,altvec)
    phi = orivec(1); theta = orivec(2); psi = orivec(3);
    J = [cos(psi)*cos(theta)  -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
         sin(psi)*cos(theta)  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi)   -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
            -sin(theta)          cos(theta)*sin(phi)                              cos(theta)*cos(phi)];
    v = J*altvec;
end