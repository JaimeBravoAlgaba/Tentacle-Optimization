function m = yoshikawa(J)
%YOSHIKAWA Returns the Yoshikawa's manipulability given the Jacobian Matrix.

    m = sqrt(abs(det(J*J')));
end

