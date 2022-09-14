function matJ = getvaluesfincontrol(d,vec,matrix)

% vec should only have [kr vel time]

cont = {};

for i = 1 : length(matrix)
    
    cont(i) = cellstr([num2str(matrix(i,1)),'@',num2str(matrix(i,2)),'@',...
               num2str(matrix(i,3)),'@',num2str(matrix(i,4)),'.txt']);
    
end

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
    
    
    if ~ismember(list(i),cont)
    
        matJ(i,:) = readmatrix([title,'\',list{i}]);
        
    end
    
end

end
