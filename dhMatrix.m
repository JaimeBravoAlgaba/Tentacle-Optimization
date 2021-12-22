function [dh] = dhMatrix(a)
%TENTACLEDH Returns the Denavit-Hartemberg Matrix for a 2D kinematic chain,
%given the lengths of the links as a vector.
%   EXAMPLE: dh = tentacleDH([10, 5, 2]);
    nSections = length(a);
    dh = zeros(nSections, 4);

    for i = 1:nSections
        dh(i, 1) = a(i);
    end
end

