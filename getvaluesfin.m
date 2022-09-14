function matJ = getvaluesfin(d,vec)

% vec should only have [kr vel time]

title = ['..\Valuesfinal\',d,'\',num2str(vec(1)),'\',num2str(vec(2)),...
         '\',num2str(vec(3))];

list = dir(title); 
[~,index] = sortrows([list.bytes].'); list = list(index); clear index
list = list(3:end);
list = struct2cell(list)';
list = (list(:,1));
    

switch d
    case 'Passive'
        
        matJ = zeros(length(list),6);
        
    case 'Control'
        
        matJ = zeros(length(list),9);
        
    otherwise
        error('Non è ne Passive ne Control')
end

parfor i = 1 : length(list)
    
    matJ(i,:) = readmatrix([title,'\',list{i}]);
    
end

end
