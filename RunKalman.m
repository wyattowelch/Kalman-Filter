% ---------------------------------------------------
% Run Kalman
% ---------------------------------------------------

% Load Data

data = load("drive3.mat");

% drive2 = 30 minute car ride [replace any '.data' with '.data.data']
% drive3 = ~1 min walk, 300 ft. with elevation and turning.

Acc = table2array(data.Acceleration);
AngV = table2array(data.AngularVelocity);
g = [0, 0, 9.81];

% Set Time Variables

time = seconds(timeofday(data.Acceleration.Timestamp(:)) ...
    - timeofday(data.Acceleration.Timestamp(1)));
dt = .1;
Qi = .9; % Covariance Matrix Adjuster 

% Subtracting Gravity

xn = zeros(3, size(Acc,1));
fuse = imufilter('SampleRate',100,'DecimationFactor',1);
q = fuse(Acc, AngV);
qF = quat2rotm(q);

for i=1:length(AngV)
    xn(1:3,i) = qF(:,:,i) * Acc(i,:)' - g'; 
end

% Calculating initial Position and Velocity 

xn = xn';
vi = cumtrapz(time, xn);
ri = cumtrapz(time,vi);

% Initial and Preset Arrays

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
    [k_x, k_P] = Predict(k_x, k_P, dt, Qi);
    [k_x, k_P] = Update(k_z, k_x, k_P, k_R, k_H);
    
    Accxf(k) = k_x(1);
    Accyf(k) = k_x(2);
    Acczf(k) = k_x(3);

end

af = [Accxf, Accyf, Acczf];

vf = cumtrapz(time, af);
rf = cumtrapz(time,vf);

% Defining for Plots

rix = ri(:,1);
riy = ri(:,2);
riz = ri(:,3);

rfx = rf(:,1);
rfy = rf(:,2);
rfz = rf(:,3);

vix = vi(:,1);
viy = vi(:,2);
viz = vi(:,3);

vfx = vf(:,1);
vfy = vf(:,2);
vfz = vf(:,3);

x = 1:length(xn);

% Plotting

figure(1)
nexttile
plot(x,vix)
hold on
plot(x,vfx, LineWidth=1.5)
title("X Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("X Velocity")
hold off

nexttile
plot(x,viy)
hold on
plot(x,vfy, LineWidth=1.5)
title("Y Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("Y Velocity")
hold off

nexttile
plot(x,viz)
hold on
plot(x,vfz, LineWidth=1.5)
title("Z Velocity")
legend("Initial","Filtered")
xlabel("time")
ylabel("Z Velocity")
hold off

figure(2)
nexttile
plot(x,rix)
hold on
plot(x,rfx, LineWidth=1.5)
title("X Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("X Position")
hold off

nexttile
plot(x,riy)
hold on
plot(x,rfy, LineWidth=1.5)
title("Y Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("Y Position")
hold off

nexttile
plot(x,riz)
hold on
plot(x,rfz, LineWidth=1.5)
title("Z Position")
legend("Initial","Filtered")
xlabel("time")
ylabel("Z Position")
hold off
