% ---------------------------------------------------
% Run Kalman
% ---------------------------------------------------

% Load Data

data = load("drive2.mat");

Acc = table2array(data.data.Acceleration);
AngV = table2array(data.data.AngularVelocity);
g = [0, 0, 9.81];

% Set Time Variables

time = seconds(timeofday(data.data.Acceleration.Timestamp(:)) ...
    - timeofday(data.data.Acceleration.Timestamp(1)));
dt = .1;

% Subtracting Gravity, and Calculate V and P

xn = zeros(3, size(Acc,1));
fuse = imufilter('SampleRate',100,'DecimationFactor',1);
q = fuse(Acc, AngV);
qF = quat2rotm(q);

for i=1:length(AngV)
    xn(1:3,i) = qF(:,:,i) * Acc(i,:)' - g'; 
end

xn = xn';
vi = xn .* dt;
ri = .5 * xn .* (dt^2);

% Filter Initials and Preset Arrays

x_n = xn(:,1);
y_n = xn(:,2);
z_n = xn(:,3);

Accxf = zeros(length(xn), 1);
Accyf = zeros(length(xn), 1);
Acczf = zeros(length(xn), 1);

% Initilize

[k_H, k_R, k_P] = Initilize();

% Prediction and Update

for k = 1:length(xn)

    k_z = [x_n(k); y_n(k); z_n(k)];

    % Initial States 
    if k == 1
        dx_k = 0;
        dy_k = 0;
        dz_k = 0;
        k_x = [k_z; dx_k; dy_k; dz_k];

    end

    % Filter
    [k_x, k_P] = Predict(k_x, k_P, dt);
    [k_x, k_P] = Update(k_z, k_x, k_P, k_R, k_H);
    
    Accxf(k) = k_x(1);
    Accyf(k) = k_x(2);
    Acczf(k) = k_x(3);

end

af = [Accxf, Accyf, Acczf];

vf = af .* dt;
rf = .5 * af .* (dt^2);

% Ploting R and V Filtered Results

x = 1:length(xn);

figure(1)
nexttile
plot(x,vi(:,1))
hold on
plot(x,vf(:,1), LineWidth=1.5)
title("X Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("X Velocity")
hold off

nexttile
plot(x,vi(:,2))
hold on
plot(x,vf(:,2), LineWidth=1.5)
title("Y Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("Y Velocity")
hold off

nexttile
plot(x,vi(:,3))
hold on
plot(x,vf(:,3), LineWidth=1.5)
title("Z Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("Z Velocity")
hold off

figure(2)
nexttile
plot(x,ri(:,1))
hold on
plot(x,rf(:,1), LineWidth=1.5)
title("X Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("X Position")
hold off

nexttile
plot(x,ri(:,2))
hold on
plot(x,rf(:,2), LineWidth=1.5)
title("Y Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("Y Position")
hold off

nexttile
plot(x,ri(:,3))
hold on
plot(x,rf(:,3), LineWidth=1.5)
title("Z Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("Z Position")
hold off

