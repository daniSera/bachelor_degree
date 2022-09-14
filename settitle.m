function title = settitle(vec)

%   name should be compute or sens
%   If the vector has 7 arguments, we already know its CONTROL, if the
%   arguments are 4, we already know its PASSIVE 

% vec deve essere [kr vel time u_mr] o [kr vel time K]

a = num2str(vec(1));
b = num2str(vec(2));
c = num2str(vec(3));

if length(vec) == 7

    d = num2str(vec(4));
    e = num2str(vec(5));
    f = num2str(vec(6));
    g = num2str(vec(7));
    title = ['\Control\',a,'\',b,'\',c,'\',d,'@',e,'@',f,'@',g,'.txt'];
    
elseif length(vec) == 4
    
    d = num2str(vec(4));
    title = ['\Passive\',a,'\',b,'\',c,'\',d,'.txt'];
    
else 
   
    error('Incorrected vector length. Aborting');
    
end

end

