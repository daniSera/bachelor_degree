clc
clear all
close all

hfig1 = figure;

%% Strada di riferimento: 90 km/h
v           = 90/3.6;     % [m/s] vehicle speed
l           = 200;        % [m] lunghezza caratteristica
road_omega  = 2*pi*v/l;
road_kr     = 0.025;

% Road definition
s       = tf('s');
road_G  = road_kr*s/(s^2 + 2*0.7*road_omega*s + road_omega^2);   % Funzione di trasferimento strada
norm_zr = norm(road_G,2);                                        % Energia del segnale stradale: deve restare costante al cambiare della velocità

% Offline road generation
dt      = 0.001;                        % [s]
tend    = 2*1000/v;                    % [s] lunghezza strada 2km
t       = 0:dt:tend;                    % [s] tempo

n  = randn(size(t))/sqrt(dt/pi);        % normalized noise
zr = lsim(road_G, n, t);                % [m]

% Grafico strada
figure(hfig1)
plot(v*t/1000,zr*100); xlabel('x [km]'); ylabel('zr [cm]'); hold on

%% Generazione profilo stradale a diversa velocità
v           = 180/3.6;     % [m/s] vehicle speed
road_omega  = 2*pi*v/l;

% Road definition
s       = tf('s');
road_G  = s/(s^2 + 2*0.7*road_omega*s + road_omega^2);              % Funzione di trasferimento strada senza kr
road_kr = norm_zr/norm(road_G,2)                                    % Calcolo kr in modo che l'energia della strada sia la stessa
road_G  = road_kr*s/(s^2 + 2*0.7*road_omega*s + road_omega^2);      % Funzione di trasferimento strada

% Offline road generation
dt      = 0.001;                        % [s]
tend    = 2*1000/v;                    % [s] lunghezza strada 2km
t       = 0:dt:tend;                    % [s] tempo

n  = randn(size(t))/sqrt(dt/pi);        % normalized noise
zr = lsim(road_G, n, t);                % [m]

% Grafico strada
figure(hfig1)
plot(v*t/1000,zr*100); xlabel('x [km]'); ylabel('zr [cm]'); hold on