% ---------------------------------------------------
% Predict
% ---------------------------------------------------

function [x, P] = Predict(x_i, P, dt)

    x = x_i;

    % State Transition

    F = [ 1  0  0 dt  0  0 ;
          0  1  0 0  dt  0 ;
          0  0  1  0 0  dt ;
          0  0  0  1  0  0 ;
          0  0  0  0  1  0 ;
          0  0  0  0  0  1 ];

    % Noise Covariance

    Q = [ (dt^4)/4    0.0      0.0   (dt^3)/2    0.0       0.0     ; 
              0.0    (dt^4)/4    0.0     0.0    (dt^3)/2   0.0     ; 
              0.0      0.0    (dt^4)/4   0.0      0.0    (dt^3)/2  ; 
            (dt^3)/2    0.0      0.0    dt^2      0.0       0.0    ; 
              0.0    (dt^3)/2    0.0     0.0     dt^2       0.0    ; 
              0.0      0.0    (dt^3)/2   0.0      0.0      dt^2    ];

    % Process noise

    w = [ (dt ^2)/2 ;
          (dt ^2)/2 ;
          (dt ^2)/2 ;
              0     ;
              0     ;
              0     ];

    x = F * x + w;
    P = F * P * F' + Q;

end






    %meow meow meow meow meow meow meow meow meow meow meow meow meow 
    %meow meow meow meow meow meow meow MEEEEOOOWWW
        % -Jordan 