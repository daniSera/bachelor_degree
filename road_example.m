clc
clear all
close all

hfig1 = figure;
hfig2 = figure;
hfig3 = figure;

wind  = [];
nover = 100;

%% Generazione profilo stradale v = 50 km/h
v           = 50/3.6;     % [m/s] vehicle speed
l           = 200;        % [m] lunghezza caratteristica
road_omega  = 2*pi*v/l;
road_kr     = 0.0186;

% Road definition
s       = tf('s');
road_G  = road_kr*s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada

norm_zr = norm(road_G,2);

% Offline road generation
dt      = 0.001;        % [s]
tend    = 20*1000/v;     % [s] lunghezza strada 2km
t       = 0:dt:tend;        % [s] tempo

n  = randn(size(t))/sqrt(dt/pi);     % normalized noise
zr = lsim(road_G, n, t);             % [m]

% Grafico strada
figure(hfig1)
plot(v*t/1000,zr*100); xlabel('x [km]'); ylabel('zr [cm]'); 
hold on

figure(hfig2)
pwelch(zr,wind,nover,[0:0.1:50],1/dt); hold on;

figure(hfig3)
bode(road_G); hold on

rms(zr)

%% Generazione profilo stradale v = 100 km/h
v           = 100/3.6;     % [m/s] vehicle speed
l           = 200;        % [m] lunghezza caratteristica
road_omega  = 2*pi*v/l;

% Road definition
s       = tf('s');
road_G  = s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada
road_kr = norm_zr/norm(road_G,2);
road_G  = road_kr*s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada

% Offline road generation
dt      = 0.001;        % [s]
tend    = 20*1000/v;     % [s] lunghezza strada 2km
t       = 0:dt:tend;        % [s] tempo

n  = randn(size(t))/sqrt(dt/pi);     % normalized noise
zr = lsim(road_G, n, t);             % [m]

% Grafico strada
figure(hfig1)
plot(v*t/1000,zr*100); xlabel('x [km]'); ylabel('zr [cm]'); 
hold on

figure(hfig2)
pwelch(zr,wind,nover,[0:0.1:50],1/dt); hold on;

figure(hfig3)
bode(road_G); hold on

rms(zr)

%% Generazione profilo stradale v = 150 km/h
v           = 150/3.6;     % [m/s] vehicle speed
l           = 200;        % [m] lunghezza caratteristica
road_omega  = 2*pi*v/l;

% Road definition
s       = tf('s');
road_G  = s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada
road_kr = norm_zr/norm(road_G,2);
road_G  = road_kr*s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada

% Offline road generation
dt      = 0.001;        % [s]
tend    = 20*1000/v;     % [s] lunghezza strada 2km
t       = 0:dt:tend;        % [s] tempo

n  = randn(size(t))/sqrt(dt/pi);     % normalized noise
zr = lsim(road_G, n, t);             % [m]

% Grafico strada
figure(hfig1)
plot(v*t/1000,zr*100); xlabel('x [km]'); ylabel('zr [cm]');  
hold on

figure(hfig2)
pwelch(zr,wind,nover,[0:0.1:50],1/dt); hold on;
xlim([0 40])

figure(hfig3)
bode(road_G); hold on

rms(zr)