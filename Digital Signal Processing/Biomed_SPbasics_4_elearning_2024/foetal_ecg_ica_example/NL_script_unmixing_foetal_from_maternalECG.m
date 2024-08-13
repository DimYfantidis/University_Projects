% This script demonstrates the use of (two different algorithms of) ICA
% for isolating maternal from foetal cardiac signal   
% First the Jade's algorithm is invoked 
% and then the FASTICA toolbox is utilized (from command line mode)
% 

clear all, close all
X=load('FOETAL_ECG.dat');
time=X(:,1)' ; ECGdata=X(:,2:9)';

B=jadeR(ECGdata); % deriving the unmixing-matrix
Sources=B*ECGdata; % estimating the ICs (source-signal)
B1=jadeR(ECGdata); % deriving the unmixing-matrix
Sources1=B*ECGdata; % estimating the ICs (source-signal)
B2=jadeR(ECGdata); % deriving the unmixing-matrix
Sources2=B2*ECGdata; % estimating the ICs (source-signal)
B3=jadeR(ECGdata); % deriving the unmixing-matrix
Sources3=B3*ECGdata; % estimating the ICs (source-signal)
B4=jadeR(ECGdata); % deriving the unmixing-matrix
Sources4=B4*ECGdata; % estimating the ICs (source-signal)

disp(isequal(Sources1, Sources2));
disp(isequal(Sources2, Sources3));
disp(isequal(Sources3, Sources4));

subplot(1,2,1),strips(zscore(ECGdata')),grid,title('original traces'),xlabel('time')
subplot(1,2,2),strips(zscore(Sources')),grid,title('estimated source-signals'),xlabel('time')

display('selecting the 1st and 7th ICs from the JADE-ICA method')
figure,plot(time,zscore(Sources(1,:)),time,zscore(Sources(3,:))+6),grid,xlabel('time'),legend('1st','3rd')
title('selected ICs from JADE-ICA method')



% PART-II using FAST-ICA toolbox
% you need to set FASTICA_toolbox in the path
[sources] = fastica (ECGdata,'numOfIC', 3);  %  [icasig] = fastica (ECGdata, 'lastEig', 5, 'numOfIC', 3);
figure,subplot(1,2,1),strips(zscore(ECGdata')),grid,title('original traces'),xlabel('time')
subplot(1,2,2),strips(zscore(sources')),grid,title('FASTICA based estimated source-signals'),xlabel('time')

[sources1] = fastica(ECGdata,'numOfIC', 3);  %  [icasig] = fastica (ECGdata, 'lastEig', 5, 'numOfIC', 3);
[sources2] = fastica(ECGdata,'numOfIC', 3);  %  [icasig] = fastica (ECGdata, 'lastEig', 5, 'numOfIC', 3);
[sources3] = fastica(ECGdata,'numOfIC', 3);  %  [icasig] = fastica (ECGdata, 'lastEig', 5, 'numOfIC', 3);

disp(isequal(sources1, sources2))
disp(isequal(sources2, sources3))
