% ---------------------------------------------------
% Initilize
% ---------------------------------------------------

function [H, R, P] = Initilize()

    % Measurement / Observation [H]

    %     x  y  z dx dy dz
    H = [ 1  0  0  0  0  0 ;
          0  1  0  0  0  0 ;
          0  0  1  0  0  0 ];

    % Uncertainty [R]

    R = eye(3) * .5;

    % Uncertainty Covariance [P]

    %      x  y  z dx dy dz 
    P = [ .8 .0 .0 .4 .0 .0 ;
          .0 .8 .0 .0 .4 .0 ;
          .0 .0 .8 .0 .0 .4 ;
          .4 .0 .0 .2 .0 .0 ;
          .0 .4 .0 .0 .2 .0 ;
          .0 .0 .4 .0 .0 .2 ];

end