function [matJ,q] = parsenfile(data,vector,matcontr)

% Funzione per l'Analisi del Modello Simulink "ProgettoTDL.slx"

tic

if length(vector(1,:)) == 1
    
    in = Simulink.SimulationInput.empty;
    matJ = zeros(length(vector(:,1)),6);
    c = 1;
    
    for i = 1 : 3
    for o = 1 : length(vector(:,1))               
                      
        in(c) = Simulink.SimulationInput('ProgettoTDL');
        in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','1');
        in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
                'Numerator',mat2str([data(1,i) 0]));
        wr = 2*pi*data(2,i)/(3.6*200);
        in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
                'Denominator',mat2str([1 2*0.7*wr wr*wr]));
        in(c) = setModelParameter(in(c),'StopTime',num2str(data(3,i)));
        in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
                'Value',num2str(vector(o)));
        in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
                'Multiplication','Matrix(K*u)');           
        in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
                'Gain',mat2str([0 0 0 0]));
        in(c) = setPostSimFcn(in(c),...
                @(x) x.setUserString(num2str([data(:,i)' vector(o)])));

        c = c+1;         
                                    
    end
    end
    
    q = zeros(c-1,8);
    
elseif length(vector(1,:)) == 4   
               
    in = Simulink.SimulationInput.empty;
    c = 1;
    q = 0;
  
    for o = 1 : length(vector(:,1))  
        
        if ~ismember([data' vector(o,:)],matcontr,'rows') 
                      
            in(c) = Simulink.SimulationInput('ProgettoTDL');
            in(c) = in(c).setBlockParameter('ProgettoTDL/ctr','Value','0');
            in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
                    'Numerator',mat2str([data(1) 0]));
            wr = 2*pi*data(2)/(3.6*200);
            in(c) = in(c).setBlockParameter('ProgettoTDL/Fcn',...
                    'Denominator',mat2str([1 2*0.7*wr wr*wr]));
            in(c) = setModelParameter(in(c),'StopTime',num2str(data(3)));
            in(c) = in(c).setBlockParameter('ProgettoTDL/u_mr',...
                    'Value','1');
            in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
                    'Multiplication','Matrix(K*u)');           
            in(c) = setBlockParameter(in(c),'ProgettoTDL/Subsystem/K',...
                    'Gain',mat2str(vector(o,:)));
            in(c) = setPostSimFcn(in(c),...
                    @(x) x.setUserString(num2str([data' vector(o,:)])));
            c = c+1;          
                             
        end
    end 
    
    matJ = zeros(length(in),9);
    
else
    
    error('Error on allcompute');
    
end

if isempty(in)
    
    return
    
end
    
c = 1;
length(in)
len = length(in)-(rem(length(in),1500));
num = (length(in)-(rem(length(in),1500)))/1500;
inn = reshape(in(1:len),num,[]);
innf = in(len+1:length(in));
clear in;
mat = zeros(1,2);
j = 1;


for m = 1 : (num)

    out = parsim(inn(m,:),'ShowProgress','on','StopOnError','on'); 

    for o = 1 : 1500
        
        ustr = str2num(out(o).SimulationMetadata.UserString); %#ok<*ST2NM>
        mat(1,1) = rms(out(o).zs_dd);
        mat(1,2) = rms(out(o).zs_d);
        matJ(j,:) = [mat ustr]; 
        j = j + 1;
        
        %Calcolo quantili
        
        if length(vector(1,:)) == 1
            q(c,1:4) = quantile(abs([out(o).zs out(o).ztilda out(o).zs_d...
                     out(o).ztilda_d]),0.95);
            q(c,5:8) = ustr;
            c = c+1;
        end
        

    end             
        clear out;
end

out = parsim(innf,'ShowProgress','on','StopOnError','on'); 

for o = 1 : length(out)
    
    ustr = str2num(out(o).SimulationMetadata.UserString); %#ok<*ST2NM>
    mat(1,1) = rms(out(o).zs_dd);
    mat(1,2) = rms(out(o).zs_d);
    matJ(j,:) = [mat ustr]; 
    j = j + 1; 
    
    %Calcolo quantili
    
    if length(vector(1,:)) == 1
        q(c,1:4) = quantile(abs([out(o).zs out(o).ztilda out(o).zs_d...
                 out(o).ztilda_d]),0.95);
        q(c,5:8) = ustr;
        c = c+1;
    end
  
end
clear out;
clc
toc
end