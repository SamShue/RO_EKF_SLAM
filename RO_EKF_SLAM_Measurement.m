function [x_new, P_new] = RO_EKF_SLAM_Measurement(x,P,z,R,idx)

    [x_mm,H] = h(x,idx);
    y = z' - x_mm';
    S = H*P*H' + R;
    K = P*H'*(S\eye(size(S)));
    x_new = x + (K*y)';
    P_new = (eye(size(K*H)) - K*H)*P;
end

function [x_measure,H] = h(x,idx)
    lmx = x((idx-1)*2 + 4);
    lmy = x((idx-1)*2 + 5);
    % idx is the index of the observed node
    x_measure(1) = sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    
    % Jacobian of h  
    H = zeros(1,length(x));
    H(1) = (x(1) - lmx)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H(2) = (x(2) - lmy)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H((idx-1)*2 + 4) = -((x(1) - lmx)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2));
    H((idx-1)*2 + 5) = -((x(2) - lmy)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2));
end