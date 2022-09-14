% myfig()
%  Sistema i font e allarga la figura alla dimensione desiderata
% 
% myfig(Figratio)
%    Figratio:           '0' la dimensione è quella attuale della figura
%                        '-1' la dimensione della figura è quella standard di Matlab (altezza/larghezza = 0.75)
%                        '-2' la dimensione della figura è impostata a 9:16 (con larghezza pari a quella standard di Matlab)
%                        'n'  la dimensione della figura è indicata da utente (n = altezza/larghezza, con larghezza pari a quella standard di Matlab)
% 
% myfig(Figratio,FigHandles)
%    FigHandles:         Vettore contenente gli handles delle figure a cui applicare la funzione. 
%                        Se non specificato, la scelta viene fatta da utente tramite GUI
% 
% myfig(Figratio,...,'ParamValue1',value,'ParamValue2',value...)
%  ParamValue list:
%    'FontLegend':       Dimensione del font della legend (se non specificato, il font resta quello della figura)
%    'FontTick':         Dimensione del font dei tick degli assi (se non specificato, il font resta quello della figura)
%    'FontLabel':        Dimensione del font delle label degli assi (se non specificato, il font resta quello della figura)
%    'Interpreter'       Interprete scritte: 'none', 'tex', 'latex' (se non specificato, resta quello della figura)
%    'LegendBox':        'on' per lasciare il box bianco di sfondo, 'off' per rendere il box trasparente
%    'NRows','NCols':    distribuisce i plot presenti nella figura secondo una griglia composta da NRow righe e NCols colonne
%    'Grid':             'on', 'off'
%    'Matrix':           Matrice dove sono disposti i numeri relativi ai subplot presenti nella figura. 
%                        La disposizione dei numeri riflette la disposizione desiderata dei subplot nella figura.
% 
% Esempio standard
%    figure
%    plot(1:10,randn(1,10)); xlabel('N')
%    myfig(-1,'FontLegend',10,'FontTick',12,'LegendBox','off','Grid','on')
% 
% Esempio più plots
   figure
   subplot(2,2,1); plot(1:10,randn(1,10)); xlabel('N')
   subplot(2,2,2); plot(1:10,randn(1,10)); ylabel('y2')
   subplot(2,2,3); plot(1:10,randn(1,10))
   subplot(2,2,4); plot(1:10,randn(1,10))
   myfig(-1,gcf,'FontLegend',10,'FontTick',12,'LegendBox','off','Grid','on','NRows',2,'NCols',2)
        oppure
   myfig(-1,gcf,'FontLegend',10,'FontTick',12,'LegendBox','off','Grid','on','NRows',4,'NCols',1)
        oppure
   myfig(-1,gcf,'FontLegend',10,'FontTick',12,'LegendBox','off','Grid','on','NRows',1,'NCols',4)
        oppure
   myfig(-1,gcf,'FontLegend',10,'FontTick',12,'LegendBox','off','Grid','on','Matrix',[1 2 3;1 4 4])
% 
% 
%   Tested Version:  Matlab 2018b
%   Last update:     31/5/2019
%   Contact:         giulio.panzani@polimi.it
%
