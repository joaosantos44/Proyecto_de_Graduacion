function q = rot2q(R)
    tol = 1e-5; % to eval a zero value in eta
    eta = sqrt(1+R(1,1)+R(2,2)+R(3,3))/2;
    if(eta <= tol)
        eta = sqrt(1+R(1,1)-R(2,2)-R(3,3))/2;
        eps = (1/(4*eta)) * [R(1,2)+R(2,1); R(1,3)+R(3,1); R(3,2)-R(2,3)];
    else
        eps = (1/(4*eta)) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    end
    q = [eta; eps];
end