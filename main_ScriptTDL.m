%% SCRIPT TESI DI LAUREA (Versione Finale)

% Lo script svolge l'analisi dati necessaria per la tesi TRIENNALE IN
% INGENIERIA MECCATRONICA all'Universita di Trento di Serafini Daniele. Lo
% scopo dello script e' quello di analizzare il Modello di QUARTER CAR
% sviluppato per la Tesi intitolata "An LMI-based approach for the control
% of semi-active magnetorheological suspensions" e rappresentato nel
% modello Simulink "ProgettoTDL.slx"

clc
close all
clear

%% DEFINIZIONE PARAMETRI --------------------------------------------------
%--------------------------------------------------------------------------
% L'analisi viene svolta variando la velocita' della macchina, e' 
% necessario variare road_omega e road_kr per mantenere costante l'energia 
% del profilo stradale.

vel_vec   = [45 90 180]; % [\frac{km}{h}] Velocita da Analizzare
kr_vec    = [0.0177 0.0250 0.0354]; % ottenuti tramite script road_example
lc        = 200; % [m] Lunghezza caratteristica (risoluzione della strada)
                 % Per valori grandi di lc vengono considerate variazioni a 
                 % bassa freqeunza della strada come colline, per valori 
                 % bassi la strada e' "piatta".
omega_vec = 2*pi*(vel_vec/3.6)/lc; 
time_vec  = 2*1000./(vel_vec/3.6); % [s] lughezza della strada 2 km

data = [kr_vec; vel_vec; time_vec];

%% PRELIMINARI ------------------------------------------------------------
%--------------------------------------------------------------------------
%% Generazione del profilo stradale

in = Simulink.SimulationInput.empty;
c = 1;

for i = 1 : 3
    
    in(c) = Simulink.SimulationInput('ProgettoTDL');
    in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,i) 0]));
    wr = 2*pi*data(2,i)/(3.6*200);
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    in(c) = setModelParameter(in(c),'StopTime',num2str(data(3,i)));
    in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value',num2str(0));
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str([0 0 0 0]));
    in(c) = setPostSimFcn(in(c),...
            @(x) x.setUserString(num2str([data(:,i)' 0]))); 
        
    c = c+1;
    
end

out = parsim(in);

close(figure(1));
figure(1);

for i = 1 : 3
    
    p1(i) = plot(out(i).tout*vel_vec(i)/3600,out(i).zr*100); hold on;
    
end

xlabel('x [km]'); ylabel('zr [cm]'); title('Profilo Stradale');
title(legend,'Velocita''');
legend('45 $[\frac{km}{h}]$','90 $[\frac{km}{h}]$','180 $[\frac{km}{h}]$');
title(legend,'Velocita''');
hold off;

%% Convergenza dell'analisi

close(figure(24))

in = Simulink.SimulationInput.empty;
for c = 1 : 3
    
    rng(c)
    in(c) = Simulink.SimulationInput('ProgettoTDL');
    in(c) = in(c).setBlockParameter('ProgettoTDL/Random Source',...
            'seed',num2str(rand*1000));
    in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([0.0250 0]));
    wr = 2*pi*90/(3.6*200);
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    in(c) = setModelParameter(in(c),'StopTime',num2str(400)); % 10 km
    in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value',num2str(1500));
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str([0 0 0 0]));
    in(c) = setPostSimFcn(in(c),...
            @(x) x.setUserString(num2str([0.0250 90 400 1500])));
            
end

out = parsim(in);

J_vec = zeros(10000,3);
z1 = out(1).zs_dd;
z2 = out(2).zs_dd;
z3 = out(3).zs_dd;

for i = 1 : 10000
    
    J_vec(i,1) = rms(z1(1:i*40));
    J_vec(i,2) = rms(z2(1:i*40));
    J_vec(i,3) = rms(z3(1:i*40));
    
end

figure(24)
hold on
title('Convergenza della Cifra di Costo al variare di Seed')
plot((1:1:10000),J_vec(:,1),(1:1:10000),J_vec(:,2),(1:1:10000),J_vec(:,3));
xlabel('x [m]'); ylabel('$J_{ax}$ $[\frac{m}{s^2}]$');
title(legend,'Rumore Gaussiano');
legend('Seed $\aleph1$','Seed $\aleph2$','Seed $\aleph3$');
ylim([2.5 3.5]);
hold off

%% ANALISI SOSPENSIONE PASSIVA --------------------------------------------
%--------------------------------------------------------------------------
%% Simulazioni Controllo Passivo

u_mr_vec = (0:1:3000)';

q = parsenv3(data,u_mr_vec);

save('Quartili.mat','q');

%% Confronto prestazioni $J_{ax}$ e J_v 

close(figure(2))
close(figure(3))  
close(figure(4))  
    
col_mat = brewermap(6,'*Blues');

for i = 1 : 3
    
    matJ = getvaluesfin('Passive',data(:,i)');
    matJ = sortrows(matJ,6);    
    
    figure(2)
    hold on
    p2(i) = plot(matJ(:,6),matJ(:,1),'Color',col_mat(i+1,:));
    hold off
    
    figure(3)
    hold on
    p3(i) = plot(matJ(:,1),matJ(:,2),'Color',col_mat(i+1,:));
    hold off
    
    figure(4)
    hold on
    p4(i) = plot(matJ(:,2),matJ(:,1),'Color',col_mat(i+1,:));
    hold off
    
end

figure(2)
title('Cifra di costo sull''Accelerazione di Cassa');
title(legend,'Velocita''');
legend('45 $[\frac{km}{h}]$','90 $[\frac{km}{h}]$','180 $[\frac{km}{h}]$',...
    'Location','east'); xlabel('$u_{MR}$ [N]'); 
ylabel('$J_{ax}$ $[\frac{m}{s^2}]$');

figure(3)
colormap(brewermap([],'Blues'))
title('Confronto delle Cifre di Costo ')
title(legend,'Parametri')
legend('45 $[\frac{km}{h}]$','90 $[\frac{km}{h}]$','180 $[\frac{km}{h}]$',...
    'Location','east'); xlabel('$J_{ax}$ $[\frac{m}{s^2}]$'); 
ylabel('$J_{v}$ $[\frac{m}{s}]$');

figure(4)
title('Confronto delle Cifre di Costo')
title(legend,'Velocita''')
legend('45 $[\frac{km}{h}]$','90 $[\frac{km}{h}]$','180 $[\frac{km}{h}]$',...
    'Location','east'); xlabel('$J_{v}$ $[\frac{m}{s}]$'); 
ylabel('$J_{ax}$ $[\frac{m}{s^2}]$');

%% Plot Saturazione Ammortizzatore

close(figure(25))
u_mr_vec = (0:1:3000)'; 

in = Simulink.SimulationInput.empty;
c = 1;
i = 2;

for o = 1 : 750 : length(u_mr_vec)              
                      
    in(c) = Simulink.SimulationInput('ProgettoTDL');
    in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,i) 0]));
    wr = 2*pi*data(2,i)/(3.6*200);
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    in(c) = setModelParameter(in(c),'StopTime',num2str(data(3,i)));
    in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value',num2str(u_mr_vec(o)));
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str([0 0 0 0]));
    in(c) = setPostSimFcn(in(c),...
            @(x) x.setUserString(num2str([data(:,i)' u_mr_vec(o)])));

    c = c+1;         
                                    
end

out = parsim(in,'ShowProgress','on','StopOnError','on'); 

colo = winter(5);

for i = 1 : 5
    
    q = out(i);
    
    f = 800*q.ztilda_d + q.f_tilda;
    
    m = zeros(length(q.tout),3);    
    m(:,1) = q.ztilda_d;
    m(:,2) = q.f_tilda;
    m(:,3) = f;
    M(:,:) = sortrows(m);
    
    figure(25)
    hold on
    subplot(2,1,1)
    plot(M(:,1),M(:,3),'Color',colo(i,:));
    hold off
    figure(25)
    hold on
    subplot(2,1,2)
    plot(M(:,1),M(:,2),'Color',colo(i,:),'DisplayName',['$u_{MR}$ = ',...
        num2str(u_mr_vec(i)*750)]);
    hold off 
    legend
      
end

figure(25)
subplot(2,1,1)
title('Forza-Velocita''')
ylabel('$f_{d}$ [N]')
xlim([-1.5 1.5])
subplot(2,1,2)
title('Non linearita'' Associata')
xlim([-1.5 1.5])
xlabel('$\dot{z}_{s}-\dot{z}_{u}$ $[\frac{m}{s}]$')
ylabel('$\tilde{f}_{d}$ [N]')


%% Rappresentazione del Parametro Ottimo Passivo

close(figure(5))
close(figure(6))
colo = brewermap(5,'*Reds');

ottJ_ax = zeros(3,6);
ottJ_v  = zeros(3,6);

for i = 1 : 3
    
    % Vado a ricercare l'ottimo in funzione delle velocita'
    
    matJ = getvaluesfin('Passive',data(:,i)'); 
    
    a = sortrows(matJ,1); ottJ_ax(i,:) = a(1,:);
    a = sortrows(matJ,2); ottJ_v(i,:) = a(1,:); 
    
end

figure(3)
hold on;
plot(ottJ_ax(1:3,1),ottJ_ax(1:3,2),'r','DisplayName','Ottimi analisi Passiva');
hold off;

for i = 1 : 3
    
    lgd1 = ['(',num2str(ottJ_ax(i,1)),' $[\frac{m}{s^2}]$ , ',...
        num2str(ottJ_ax(i,2)),' $[\frac{m}{s}]$',...
            ') $\rightarrow$ $u_{MR}$ = ',num2str(ottJ_ax(i,6))];
    lgd2 = ['(',num2str(ottJ_v(i,1)),' $[\frac{m}{s^2}]$ , ',...
        num2str(ottJ_v(i,2)),' $[\frac{m}{s}]$',...
            ') $\rightarrow$ $u_{MR}$ = ',num2str(ottJ_v(i,6))];
    
    figure(3)
    hold on
    p5(i) = plot(ottJ_ax(i,1),ottJ_ax(i,2),'.','Color',colo(i+1,:),...
        'MarkerSize',20,'DisplayName',lgd1);   
%     p6(i) = plot(ottJ_v(i,1),ottJ_v(i,2),'.','MarkerSize',20,...
%             'DisplayName',lgd2); 
    hold off
    
    figure(5)
    bj = bar(vel_vec,[ottJ_ax(:,1)'; ottJ_v(:,2)']); 
        
    figure(6)
    bu = bar(vel_vec,[ottJ_ax(:,6)'; ottJ_v(:,6)']); 
    
end

figure(5)
hold on;
xtips1 = bj(1).XEndPoints;
ytips1 = bj(1).YEndPoints;
labels1 = string(bj(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

xtips2 = bj(2).XEndPoints;
ytips2 = bj(2).YEndPoints;
labels2 = string(bj(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

legend('$J_{ax}$ $[\frac{m}{s^2}]$','$J_{v}$ $[\frac{m}{s}]$',...
    'Location','northwest');
title('Rappresentazione delle Cifre di Costo ottime')
hold off;

figure(6)
hold on;
xtips1 = bu(1).XEndPoints;
ytips1 = bu(1).YEndPoints;
labels1 = string(bu(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

xtips2 = bu(2).XEndPoints;
ytips2 = bu(2).YEndPoints;
labels2 = string(bu(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

legend('$J_{ax}$ $[\frac{m}{s^2}]$','$J_{v}$ $[\frac{m}{s}]$',...
    'Location','northwest');
title('Rappresentazione dei Parametri $u_{MR}$ ottimi')
hold off;

%% ANALISI SOSPENSIONE SEMIATTIVA -----------------------------------------
%--------------------------------------------------------------------------
%% Simulazioni controllo semiattivo Bench

Kbench = [-18901 -45920 22704 -36338];

in = Simulink.SimulationInput.empty;
c = 1;

for i = 1 : 3
    
    in(c) = Simulink.SimulationInput('ProgettoTDL');
    in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','0');
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,i) 0]));
    wr = 2*pi*data(2,i)/(3.6*200);
    in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    in(c) = setModelParameter(in(c),'StopTime',num2str(data(3,i)));
    in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value','1');
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str(Kbench));
    in(c) = setPostSimFcn(in(c),...
            @(x) x.setUserString(num2str([data(:,i)' Kbench])));
    c = c+1;
    
end

out = parsim(in);
matbench = zeros(3,9);

for o = 1 : length(out)
    
    ustr = str2num(out(o).SimulationMetadata.UserString); %#ok<*ST2NM>
    title = mat2str(ustr);
    mat(1,1) = rms(out(o).zs_dd);
    mat(1,2) = rms(out(o).zs_d);
    writematrix([mat ustr],['..\Valuesfinal\',title,'.txt']);
    matbench(o,:) = [mat ustr];
  
end

%% Save MatBench

save('matBench.mat','matbench');

%% Rappresentazione del Parametro Bench

load 'matBench.mat';
colo = brewermap(5,'*Greens');
figure(3)
hold on;
plot(matbench(:,1),matbench(:,2),'g','DisplayName','Analisi parametri Bench');
hold off;

for i = 1 : 3 
    
    lgd3 = ['(',num2str(matbench(i,1)),' $[\frac{m}{s^2}]$ , ',...
            num2str(matbench(i,2)),' $[\frac{m}{s}]$',...
                ') $\rightarrow$ Bench'];
    
    figure(3)
    hold on
    p7(i) = plot(matbench(i,1),matbench(i,2),'.','Color',colo(i+1,:),...
        'MarkerSize',20,'DisplayName',lgd3);
    hold off

end

close(figure(7))   
figure(7)
bj = bar(vel_vec,[matbench(:,1)'; matbench(:,2)']); 

figure(7)
hold on;
xtips1 = bj(1).XEndPoints;
ytips1 = bj(1).YEndPoints;
labels1 = string(bj(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

xtips2 = bj(2).XEndPoints;
ytips2 = bj(2).YEndPoints;
labels2 = string(bj(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

legend('$J_{ax}$ [$[\frac{m}{s^2}]$]','$J_{v}$ [$[\frac{m}{s}]$]',...
    'Location','northwest');
title('Confronto delle Cifre di Costo');

hold off;

%% Ottimizzazione upper/lower bound con quartili

load 'Quartili.mat';

steps = 14;

% Sezione v = 90 \frac{km}{h}

q_90 = q(q(:,6) == 90,:);
qmax_90 = max(q_90(:,1:4));
Kmax_90 = (1500./qmax_90)*16; % aumento valore 1500%


% Definisco vettore K per simulazione


k1 = 0; % in questo modo effettuo l'analisi finale
% k1 = (-Kmax_90(1):2*Kmax_90(1)/steps:Kmax_90(1)); % qua analisi iniziale

k2 = (-Kmax_90(2):2*Kmax_90(2)/steps:Kmax_90(2));
k3 = (-Kmax_90(3):2*Kmax_90(3)/steps:Kmax_90(3));
k4 = (-Kmax_90(4):2*Kmax_90(4)/steps:Kmax_90(4));

K_90 = zeros(length(k1)*length(k2)*length(k3)*length(k4),4);
c = 1;

for i = 1 : length(k1)
    for j = 1 : length(k2)
        for k = 1 : length(k3)
            for l = 1 : length(k4)              
                K_90(c,:) = [k1(i) k2(j) k3(k) k4(l)];              
                c = c+1;
            end
        end
    end
end

K_90 = fix(K_90); % fix per avere interi

% Sezione v = 45 \frac{km}{h}

q_45 = q(q(:,6) == 45,:);
qmax_45 = max(q_45(:,1:4));
Kmax_45 = (1500./qmax_45)*21; % aumento valore 2000%

% Definisco vettore K per simulazione

k1 = 0; % in questo modo effettuo l'analisi finale
% k1 = (-Kmax_45(1):2*Kmax_45(1)/steps:Kmax_45(1)); % qua analisi iniziale

k2 = (-Kmax_45(2):2*Kmax_45(2)/steps:Kmax_45(2));
k3 = (-Kmax_45(3):2*Kmax_45(3)/steps:Kmax_45(3));
k4 = (-Kmax_45(4):2*Kmax_45(4)/steps:Kmax_45(4));

K_45 = zeros(length(k1)*length(k2)*length(k3)*length(k4),4);
c = 1;

for i = 1 : length(k1)
    for j = 1 : length(k2)
        for k = 1 : length(k3)
            for l = 1 : length(k4)              
                K_45(c,:) = [k1(i) k2(j) k3(k) k4(l)];              
                c = c+1;
            end
        end
    end
end

K_45 = fix(K_45); % fix per avere interi

% Sezione v = 180 \frac{km}{h}

q_180 = q(q(:,6) == 180,:);
qmax_180 = max(q_180(:,1:4));
Kmax_180 = (1500./qmax_180)*11; % aumento valore 1000%

% Definisco vettore K per simulazione

k1 = 0; % in questo modo effettuo l'analisi finale
%k1 = (-Kmax_180(1):2*Kmax_180(1)/steps:Kmax_180(1)); %qua analisi iniziale

k2 = (-Kmax_180(2):2*Kmax_180(2)/steps:Kmax_180(2));
k3 = (-Kmax_180(3):2*Kmax_180(3)/steps:Kmax_180(3));
k4 = (-Kmax_180(4):2*Kmax_180(4)/steps:Kmax_180(4));

K_180 = zeros(length(k1)*length(k2)*length(k3)*length(k4),4);
c = 1;

for i = 1 : length(k1)
    for j = 1 : length(k2)
        for k = 1 : length(k3)
            for l = 1 : length(k4)              
                K_180(c,:) = [k1(i) k2(j) k3(k) k4(l)];              
                c = c+1;
            end
        end
    end
end

K_180 = fix(K_180); % fix per avere interi

% Matrici K
save('MatriciK.mat','K_180','K_90','K_45');

clear k1 k2 k3 k4 i j k l c ...
      q q_45 q_90 q_180 q_max45 q_max90 q_max180 steps;

%% Simulazione Controllo Semiattivo v = 45 \frac{km}{h}

load('MatriciK.mat');
load('matJcontv45.mat'); 
clear matJv45;

[matJv45,~] = parsenfilev2(data(:,1),K_45);

matJcontv45 = unique([matJcontv45; matJv45],'rows'); 

save('matJcontv45.mat','matJcontv45','matJv45');

writematrix(matJcontv45,'ValK45.txt');

% Rappresentazione Minimi su K3 e K4

tic
figure(1)
h1 = scatter3(matJv45(:,8),matJv45(:,9),matJv45(:,1),[],matJv45(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
figure(2)
h2 = scatter3(matJcontv45(:,8),matJcontv45(:,9),matJcontv45(:,1),...
     [],matJcontv45(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
h1.Marker = '.';
h2.Marker = '.';
toc

%% Simulazione Controllo Semiattivo v = 45 \frac{km}{h} con k1 = 0

load('MatriciK.mat');
load('matJcontv45k1.mat'); 
clear matJv45k1;

[matJv45k1,~] = parsenfilev2(data(:,1),K_45);

matJcontv45k1 = unique([matJcontv45k1; matJv45k1],'rows'); 

save('matJcontv45k1.mat','matJcontv45k1','matJv45k1');

writematrix(matJcontv45k1,'ValK45k1.txt');

%% Simulazione Controllo Semiattivo v = 90 \frac{km}{h}

load('MatriciK.mat');
load('matJcontv90.mat'); 
clear matJv90;

[matJv90,~] = parsenfilev2(data(:,2),K_90);

matJcontv90 = unique([matJcontv90; matJv90],'rows'); 

save('matJcontv90.mat','matJcontv90','matJv90');

writematrix(matJcontv90,'ValK90.txt');

% Rappresentazione Minimi su K3 e K4

tic
figure(26)
h1 = scatter3(matJv90(:,8),matJv90(:,9),matJv90(:,1),[],matJv90(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
figure(27)
h2 = scatter3(matJcontv90(:,8),matJcontv90(:,9),matJcontv90(:,1),...
     [],matJcontv90(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
h1.Marker = '.';
h2.Marker = '.';
toc

%% Simulazione Controllo Semiattivo v = 90 \frac{km}{h} con k1 = 0

load('MatriciK.mat');
load('matJcontv90k1.mat'); 
clear matJv90k1;

[matJv90k1,~] = parsenfilev2(data(:,2),K_90);

matJcontv90k1 = unique([matJcontv90k1; matJv90k1],'rows'); 

save('matJcontv90k1.mat','matJcontv90k1','matJv90k1');

writematrix(matJcontv90k1,'ValK90k1.txt');

%% Simulazione Controllo Semiattivo v = 180 \frac{km}{h}

load('MatriciK.mat');
load('matJcontv180.mat'); 
clear matJv180;

[matJv180,~] = parsenfilev2(data(:,3),K_180);

matJcontv180 = unique([matJcontv180; matJv180],'rows'); 

save('matJcontv180.mat','matJcontv180','matJv180');

writematrix(matJcontv180,'ValK180.txt');

% Rappresentazione Minimi su K3 e K4

tic
figure(1)
h1 = scatter3(matJv180(:,8),matJv180(:,9),matJv180(:,1),[],matJv180(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
figure(2)
h2 = scatter3(matJcontv180(:,8),matJcontv180(:,9),matJcontv180(:,1),...
     [],matJcontv180(:,1));
xlabel('K3');
ylabel('K4');
zlabel('J');
colormap(parula);
h1.Marker = '.';
h2.Marker = '.';
toc

%% Simulazione Controllo Semiattivo v = 180 \frac{km}{h} con k1 = 0

load('MatriciK.mat');
load('matJcontv180k1.mat'); 
clear matJv180k1;

[matJv180k1,~] = parsenfilev2(data(:,3),K_180);

matJcontv180k1 = unique([matJcontv180k1; matJv180k1],'rows'); 

save('matJcontv180k1.mat','matJcontv180k1','matJv180k1');

writematrix(matJcontv180k1,'ValK180k1.txt');

%% Rappresentazione del Parametro Ottimo Semi-Attivo

load 'matAttivo.mat';
colo = brewermap(5,'*Purples');
figure(3)
hold on;
plot(matattivo(:,1),matattivo(:,2),'m','DisplayName','Ottimi analisi Semi-Attiva');
hold off;

for i = 1 : 3 
    
    lgd3 = ['(',num2str(matattivo(i,1)),' $[\frac{m}{s^2}]$ , ',...
            num2str(matattivo(i,2)),' $[\frac{m}{s}]$',...
                ') $\rightarrow$ K = ',mat2str(matattivo(i,6:9))];
    
    figure(3)
    hold on
    p7(i) = plot(matattivo(i,1),matattivo(i,2),'.','Color',colo(i+1,:),...
        'MarkerSize',20,'DisplayName',lgd3);
    hold off

end

close(figure(7))
    
figure(7)
bj = bar(vel_vec,[matattivo(:,1)'; matattivo(:,2)']); 

figure(7)
hold on;
xtips1 = bj(1).XEndPoints;
ytips1 = bj(1).YEndPoints;
labels1 = string(bj(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

xtips2 = bj(2).XEndPoints;
ytips2 = bj(2).YEndPoints;
labels2 = string(bj(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

legend('$J_{ax}$ [$[\frac{m}{s^2}]$]','$J_{v}$ [$[\frac{m}{s}]$]',...
    'Location','northwest');
title('Confronto delle Cifre di Costo');

hold off;

%% Rappresentazione del Parametro Ottimo Semi-Attivo con k1 = 0

load 'matAttivok1.mat';
colo = brewermap(5,'*Greys');
figure(3)
hold on;
plot(matattivok1(:,1),matattivok1(:,2),'k','DisplayName',...
    'Ottimi analisi Semi-Attiva con k1 = 0');
hold off;

for i = 1 : 3 
    
    lgd3 = ['(',num2str(matattivok1(i,1)),' $[\frac{m}{s^2}]$ , ',...
            num2str(matattivok1(i,2)),' $[\frac{m}{s}]$',...
                ') $\rightarrow$ K = ',mat2str(matattivok1(i,6:9))];
    
    figure(3)
    hold on
    p7(i) = plot(matattivok1(i,1),matattivok1(i,2),'.','Color',colo(i+1,:),...
        'MarkerSize',20,'DisplayName',lgd3);
    hold off

end

close(figure(30))
    
figure(30)
bj = bar(vel_vec,[matattivok1(:,1)'; matattivok1(:,2)']); 

figure(30)
hold on;
xtips1 = bj(1).XEndPoints;
ytips1 = bj(1).YEndPoints;
labels1 = string(bj(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

xtips2 = bj(2).XEndPoints;
ytips2 = bj(2).YEndPoints;
labels2 = string(bj(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom'); xlabel('$[\frac{m}{s}]$');

legend('$J_{ax}$ [$[\frac{m}{s^2}]$]','$J_{v}$ [$[\frac{m}{s}]$]',...
    'Location','northwest');
title('Confronto delle Cifre di Costo');

hold off;

%% ANALISI DATI EXCEL -----------------------------------------------------
%--------------------------------------------------------------------------

load 'MExcel.mat';
load 'MExcelBound.mat';
load 'MExcelPercent.mat';
load 'MExcelOttimi.mat';

load 'MExcelk1.mat';
load 'MExcelBoundk1.mat';
load 'MExcelPercentk1.mat';
load 'MExcelOttimik1.mat';

%% velocita'' 45 \frac{km}{h}

for i = 1 : 4
    
    close(figure(i+7))

    figure(i+7)
    hold on
    area(MExcel45Percent,MExcel45Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel45Percent,-MExcel45Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel45Percent,MExcel45(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 45 $[\frac{km}{h}]$'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    legend('Location','southwest')
    
end

%% velocita'' 45 \frac{km}{h} k1 = 0

for i = 1 : 4
    
    close(figure(i+7))

    figure(i+7)
    hold on
    area(MExcel45Percentk1,MExcel45Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel45Percentk1,-MExcel45Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel45Percentk1,MExcel45k1(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 45 $[\frac{km}{h}]$ con k1 = 0'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    legend('Location','southwest')
    
end

%% velocita'' 90 \frac{km}{h}

for i = 1 : 4
    
    close(figure(i+11))

    figure(i+11)
    hold on
    area(MExcel90Percent,MExcel90Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel90Percent,-MExcel90Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel90Percent,MExcel90(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 90 $[\frac{km}{h}]$'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    xlim([0 1250])
    legend('Location','southwest')
    
end

%% velocita'' 90 \frac{km}{h} k1 = 0

for i = 1 : 4
    
    close(figure(i+11))

    figure(i+11)
    hold on
    area(MExcel90Percentk1,MExcel90Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel90Percentk1,-MExcel90Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel90Percentk1,MExcel90k1(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 90 $[\frac{km}{h}]$ con k1 = 0'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    xlim([0 1250])
    legend('Location','southwest')
    
end

%% velocita'' 180 \frac{km}{h} 

for i = 1 : 4
    
    close(figure(i+15))

    figure(i+15)
    hold on
    area(MExcel180Percent,MExcel180Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel180Percent,-MExcel180Bound(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel180Percent,MExcel180(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 180 $[\frac{km}{h}]$'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    legend('Location','southwest')
    
end

%% velocita'' 180 \frac{km}{h} k1 = 0

for i = 1 : 4
    
    close(figure(i+15))

    figure(i+15)
    hold on
    area(MExcel180Percentk1,MExcel180Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','Area di Analisi');
    h = area(MExcel180Percentk1,-MExcel180Boundk1(:,i),'FaceColor','b',...
        'FaceAlpha',.3,'EdgeAlpha',.3,'DisplayName','');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    plot(MExcel180Percentk1,MExcel180k1(:,i),'Marker','.','MarkerSize',20,...
        'LineWidth',2,'DisplayName','Parametri Ottimi')
    hold off
    title(['Bound K',num2str(i),' a velocita'' v = 180 $[\frac{km}{h}]$ con k1 = 0'])
    xlabel('Incremento $\%$')
    ylabel('Valore K')
    legend('Location','southwest')
    
end

%% Grafici Tendenze

col_mat1 = brewermap(5,'*Blues');
close(figure(20))
figure(20)
hold on
plot(MExcel45Percent,MExcel45Ott,'Color',col_mat1(2,:))
plot(MExcel90Percent,MExcel90Ott,'Color',col_mat1(3,:))
plot(MExcel180Percent,MExcel180Ott,'Color',col_mat1(4,:))
xlabel('Limiti $\%$');ylabel('$J_{ax}$');
title('Convergenza delle cifre di costo')
title(legend,'Velocita''')
legend('45 $\frac{km}{h}$','90 $\frac{km}{h}$','180 $\frac{km}{h}$')
hold off

%% Grafici Tendenze k1 = 0

col_mat1 = brewermap(7,'*Blues');
col_mat2 = brewermap(5,'*Reds');
close(figure(20))
figure(20)
hold on
plot(MExcel45Percent,MExcel45Ott,'Color',col_mat1(2,:))
plot(MExcel90Percent,MExcel90Ott,'Color',col_mat1(3,:))
plot(MExcel180Percent,MExcel180Ott,'Color',col_mat1(4,:))
plot(MExcel45Percentk1,MExcel45Ottk1,'Color',col_mat2(2,:))
plot(MExcel90Percentk1,MExcel90Ottk1,'Color',col_mat2(3,:))
plot(MExcel180Percentk1,MExcel180Ottk1,'Color',col_mat2(4,:))
legend('45 $\frac{km}{h}$','90 $\frac{km}{h}$','180 $\frac{km}{h}$',...
    '45 $\frac{km}{h}$ $\rightarrow$ k1 = 0',...
    '90 $\frac{km}{h}$ $\rightarrow$ k1 = 0',...
    '180 $\frac{km}{h}$ $\rightarrow$ k1 = 0')
xlabel('Limiti $\%$');ylabel('$J_{ax}$');
title('Convergenza della cifre di costo')
title(legend,'Parametri')
hold off

%% Funzione di Trasferimento dei parametri ottimi per ogni velocita''

load 'matBench.mat';
load 'matAttivo.mat';
load 'matAttivok1.mat';

bench = matbench(1,6:9);

% Passiva

inp = Simulink.SimulationInput.empty;

for c = 1 : 3
    
    inp(c) = Simulink.SimulationInput('ProgettoTDL');
    inp(c) = inp(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
    inp(c) = inp(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,c) 0]));
    wr = 2*pi*data(2,c)/(3.6*200);
    inp(c) = inp(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    inp(c) = setModelParameter(inp(c),'StopTime',num2str(data(3,c)));
    inp(c) = inp(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value',num2str(ottJ_ax(c,6)));
    inp(c) = setBlockParameter(inp(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    inp(c) = setBlockParameter(inp(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str([0 0 0 0]));
    inp(c) = setPostSimFcn(inp(c),...
            @(x) x.setUserString(num2str([data(:,c)' ottJ_ax(c,6)])));
    
end
    
outp = parsim(inp);

% Bench

inb = Simulink.SimulationInput.empty;

for c = 1 : 3

    inb(c) = Simulink.SimulationInput('ProgettoTDL');
    inb(c) = inb(c).setBlockParameter('ProgettoTDL/ctr','Value','0');
    inb(c) = inb(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,c) 0]));
    wr = 2*pi*data(2,c)/(3.6*200);
    inb(c) = inb(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    inb(c) = setModelParameter(inb(c),'StopTime',num2str(data(3,c)));
    inb(c) = inb(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value','1');
    inb(c) = setBlockParameter(inb(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    inb(c) = setBlockParameter(inb(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str(bench));
    inb(c) = setPostSimFcn(inb(c),...
            @(x) x.setUserString(num2str([data(:,c)' bench])));
    
end
    
outb = parsim(inb);

% Semi-Attiva

ina = Simulink.SimulationInput.empty;

for c = 1 : 3

    ina(c) = Simulink.SimulationInput('ProgettoTDL');
    ina(c) = ina(c).setBlockParameter('ProgettoTDL/ctr','Value','0');
    ina(c) = ina(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,c) 0]));
    wr = 2*pi*data(2,c)/(3.6*200);
    ina(c) = ina(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    ina(c) = setModelParameter(ina(c),'StopTime',num2str(data(3,c)));
    ina(c) = ina(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value','1');
    ina(c) = setBlockParameter(ina(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    ina(c) = setBlockParameter(ina(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str(matattivo(c,6:9)));
    ina(c) = setPostSimFcn(ina(c),...
            @(x) x.setUserString(num2str([data(:,c)' matattivo(c,6:9)])));
    
end

outa = parsim(ina);

% Semi-Attiva k1 = 0

inak1 = Simulink.SimulationInput.empty;

for c = 1 : 3

    inak1(c) = Simulink.SimulationInput('ProgettoTDL');
    inak1(c) = inak1(c).setBlockParameter('ProgettoTDL/ctr','Value','0');
    inak1(c) = inak1(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Numerator',mat2str([data(1,c) 0]));
    wr = 2*pi*data(2,c)/(3.6*200);
    inak1(c) = inak1(c).setBlockParameter('ProgettoTDL/Fcn',...
            'Denominator',mat2str([1 2*0.7*wr wr*wr]));
    inak1(c) = setModelParameter(inak1(c),'StopTime',num2str(data(3,c)));
    inak1(c) = inak1(c).setBlockParameter('ProgettoTDL/u_mr',...
            'Value','1');
    inak1(c) = setBlockParameter(inak1(c),'ProgettoTDL/Subsystem/K',...
            'Multiplication','Matrix(K*u)');           
    inak1(c) = setBlockParameter(inak1(c),'ProgettoTDL/Subsystem/K',...
            'Gain',mat2str(matattivok1(c,6:9)));
    inak1(c) = setPostSimFcn(inak1(c),...
            @(x) x.setUserString(num2str([data(:,c)' matattivok1(c,6:9)])));
    
end

outak1 = parsim(inak1);

%%

for i = 1 : 3
    
    len = 2^nextpow2(length(outp(i).zr)); 
    winlen = 2500; 
      
%     pr = [zeros(round((len-length(outp(i).zr))/2),1); outp(i).zr;...
%         zeros(round((len-length(outp(i).zr))/2)-1,1)];
%     pd = [zeros(round((len-length(outp(i).zs_dd))/2),1); outp(i).zs_dd;...
%         zeros(round((len-length(outp(i).zs_dd))/2)-1,1)];
%     br = [zeros(round((len-length(outb(i).zr))/2),1); outb(i).zr;...
%         zeros(round((len-length(outb(i).zr))/2)-1,1)];
%     bd = [zeros(round((len-length(outb(i).zs_dd))/2),1); outb(i).zs_dd;...
%         zeros(round((len-length(outb(i).zs_dd))/2)-1,1)];
%     ar = [zeros(round((len-length(outa(i).zr))/2),1); outa(i).zr;...
%         zeros(round((len-length(outa(i).zr))/2)-1,1)];
%     ad = [zeros(round((len-length(outa(i).zs_dd))/2),1); outa(i).zs_dd;...
%         zeros(round((len-length(outa(i).zs_dd))/2)-1,1)];
%     ak1r = [zeros(round((len-length(outak1(i).zr))/2),1); outak1(i).zr;...
%         zeros(round((len-length(outak1(i).zr))/2)-1,1)];
%     ak1d = [zeros(round((len-length(outak1(i).zs_dd))/2),1); outak1(i).zs_dd;...
%         zeros(round((len-length(outak1(i).zs_dd))/2)-1,1)];
%     
%     [Txy,f] = tfestimate([outp(i).zr outb(i).zr outa(i).zr outak1(i).zr],...
%         [outp(i).zs_dd outb(i).zs_dd outa(i).zs_dd outak1(i).zs_dd],[],[],...
%             (0:0.001:20),1000);  

pr = [zeros(round((len-length(outp(i).zr))/2),1); outp(i).zr;...
        zeros(round((len-length(outp(i).zr))/2)-1,1)];
    pd = [zeros(round((len-length(outp(i).zs_d))/2),1); outp(i).zs_d;...
        zeros(round((len-length(outp(i).zs_d))/2)-1,1)];
    br = [zeros(round((len-length(outb(i).zr))/2),1); outb(i).zr;...
        zeros(round((len-length(outb(i).zr))/2)-1,1)];
    bd = [zeros(round((len-length(outb(i).zs_d))/2),1); outb(i).zs_d;...
        zeros(round((len-length(outb(i).zs_d))/2)-1,1)];
    ar = [zeros(round((len-length(outa(i).zr))/2),1); outa(i).zr;...
        zeros(round((len-length(outa(i).zr))/2)-1,1)];
    ad = [zeros(round((len-length(outa(i).zs_d))/2),1); outa(i).zs_d;...
        zeros(round((len-length(outa(i).zs_d))/2)-1,1)];
    ak1r = [zeros(round((len-length(outak1(i).zr))/2),1); outak1(i).zr;...
        zeros(round((len-length(outak1(i).zr))/2)-1,1)];
    ak1d = [zeros(round((len-length(outak1(i).zs_d))/2),1); outak1(i).zs_d;...
        zeros(round((len-length(outak1(i).zs_d))/2)-1,1)];
    
    [Txy,f] = tfestimate([outp(i).zr outb(i).zr outa(i).zr outak1(i).zr],...
        [outp(i).zs_d outb(i).zs_d outa(i).zs_d outak1(i).zs_d],[],[],...
            [],1000); 
        
%         (0:0.001:20)
    
    [Txy1,f1] = tfestimate([pr br ar ak1r],[pd bd ad ak1d],winlen,[],...
        [],1000);
    
    close(figure(i+20))
    figure(i+20)   
    hold on
    pp = plot(f1,mag2db(abs(Txy1)),'LineWidth',1.5);
    pp1 = plot(f,mag2db(abs(Txy)));    
    pp1(1).Color = [0 0.4470 0.7410];
    pp(1).Color = [0 0.4470 0.7410];
    pp1(2).Color = [0.8500 0.3250 0.0980];
    pp(2).Color = [0.8500 0.3250 0.0980];
    pp1(3).Color = [0.9290 0.6940 0.1250];
    pp(3).Color = [0.9290 0.6940 0.1250];
    pp1(4).Color = [0.4660 0.6740 0.1880];
    pp(4).Color = [0.4660 0.6740 0.1880];
    pp1(1).Color(4) = 0.25;
    pp1(2).Color(4) = 0.25;
    pp1(3).Color(4) = 0.25;
    pp1(4).Color(4) = 0.25;
    hold off
%     xlim([0 20])
%     ylim([-10 60])
    title(['FdT sull''Accelerazione a velocita'' v = ',...
        num2str(data(2,i)),' $\frac{km}{h}$']);
    title(legend,'Analisi')
    legend('Passiva','parametri Bench','Semi-Attiva',...
        'Semi-Attiva con k1 = 0','Location','southeast');
    xlabel('[Hz]');
    ylabel('[dB]');   
       
end

%% Generazione FdT u_MR = 0

in = Simulink.SimulationInput.empty;
c = 1;
i = 2;
in(c) = Simulink.SimulationInput('ProgettoTDL');
in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
'Numerator',mat2str([data(1,i) 0]));
wr = 2*pi*data(2,i)/(3.6*200);
in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
'Denominator',mat2str([1 2*0.7*wr wr*wr]));
in(c) = setModelParameter(in(c),'StopTime',num2str(data(3,i)));
in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
'Value',num2str(0));
in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
'Multiplication','Matrix(K*u)');
in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
'Gain',mat2str([0 0 0 0]));
in(c) = setPostSimFcn(in(c),...
@(x) x.setUserString(num2str([data(:,i)' 0])));
out = parsim(in);

[Txy,f] = tfestimate(out.zr,out.zs_dd,[],[],...
(0:0.001:20),1000);

A =[0 0 1 0; 0 0 0 1; 0 -60 0 -(16/9); (300000/70) -(33120/7) 0 -(832/63)];
B =[0; 0; 0; -(300000/7)];
C =[0 -60 0 -(16/9)];
D = 0;

[b,a] = ss2tf(A,B,C,D);

sys=tf(b,a);

[mag,phase,wout] = bode(sys);

close(figure(28))
figure(28)
hold on
semilogx(f,20*log10(abs(Txy)),'LineWidth',1.1);
semilogx(wout(:,1)/(2*pi), 20*log10(squeeze((mag)))-20,'LineWidth',1.1); 
grid off; 
title('Funzione di Trasferimento per $u_{MR}$ = 0 [N]'); xlabel('[Hz]'); 
ylabel('[dB]');
xlim([0 20])
ylim([20 60])
legend('Stimata','Analitica')
hold off

%% Rappresentazione Segnali

close(figure(1))
figure(1)
hold on
subplot(2,2,1)
plot(out.tout,out.zs);
xticks([])
title('Segnale $z_s$')
ylabel('[m]')
subplot(2,2,2)
plot(out.tout,out.ztilda);
xticks([])
title('Segnale $\tilde{z}$')
ylabel('[m]')
subplot(2,2,3)
plot(out.tout,out.zs_d);
xticks([])
title('Segnale $\dot{z}_s$')
ylabel('[$\frac{m}{s}$]')
subplot(2,2,4)
plot(out.tout,out.ztilda_d);
title('Segnale $\dot{\tilde{z}}$')
xlabel('[t]')
ylabel('[$\frac{m}{s}$]')
hold off

%% Immagini

for i = 1 : 28
    
    myfig(0,figure(i),'FontLegend',12,'FontTick',11,'FontLabel',18,...
        'Interpreter','latex','LegendBox','off')
    
end
