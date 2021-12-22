function dm = dynManip(J, M)
%DINMANIP Calculates the Dynamic Manipulability, given the J and M
%matrixes.

    dm = sqrt(abs(det(J/(M'*M)*J')));
end

