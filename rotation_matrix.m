function c = rotation_matrix(beta)
    c = [ cos(beta) sin(beta) 0;
         -sin(beta) cos(beta) 0;
          0         0         1];
end