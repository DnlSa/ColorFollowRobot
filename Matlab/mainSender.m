clear; close all; clc
%% LEGGIMI
% Questo script viene messo sopra il rover e sara speculare a quello di
% matlab : 
% 1) Crea il pacchetto tramite la funzione PACK_REF 
% 2) invia il pacchetto tramite la write
% 3) riceve un pacchetto tramite la funzione read 
% 4) lo spacchetta con la funzione ROBOT_UNPACK 
% 5) elabora i dati ricevuti 
% 6) riesegue il punto 1 fino allo scadere del for 


%% NOTE:
% l esp non fa ritornare i valori passati in input secondo me tocca 
% modificare da byte a int il valore e poi ripassarlo in output 

%%

localIP = '192.168.1.3'; % indirizzo ip del PC
localPort = 5006;
udp_server = udpport('LocalHost', localIP, 'LocalPort', localPort);
remoteIP = '192.168.1.6'; % indirizzo ip del robot
remotePort = 5000;

%% Comandi Iniziali 
% valori cambiati dall utente (mandato all ESP)
dataOut.last_cmd = 1; % OK 0 = stop , 1 = start ,
dataOut.prefer_distance = 60; % distanza custom di inseguimento dell'oggetto in centimet

% comunication
Ntests = 1; % numero pacchetti scambiati
memoData = zeros(Ntests,6);

% for i = 1:1:Ntests
for i = 1: 1 : inf
     % Send data
     disp("Sending...")
     pcktOut = robot_pkt(dataOut); % manda il pacchetto di 3 valori 
     write(udp_server, pcktOut, 'uint32', remoteIP, remotePort);
    % Receive data
     disp("Wait to get data...")
    pcktIn = read(udp_server, 24); % prende in input i 6 valori 
     disp("got data")
    data = robot_unpack(pcktIn);
    memoData(i,:) = [data.last_cmd,data.prefer_distance,data.latency ,data.pos_x,data.pos_y,data.distance]
%     if( data.distance == 1000)
%       dataOut.last_cmd = 0; % OK 0 = stop , 1 = start ,
%       pcktOut = robot_pkt(dataOut);
%       write(udp_server, pcktOut, 'uint32', remoteIP, remotePort);
%     else
%       dataOut.last_cmd = 1;
%     end
    

    dataOut.pos_x = data.pos_x; % OK valore x dell'oggetto su la camera
    dataOut.pos_y =data.pos_y; % OK valore y dell'oggetto su la camera
    dataOut.distance= data.distance; % OK distanza dall'oggetto dato dal distanziometro
end

%% PLOTTING DEI DATI ACQUISITI
figure(1)
plot3( 0,0,0,memoData(:,6),memoData(:,4),memoData(:,5),'-bx','LineStyle','none','MarkerSize',9)
grid on
title('X-Y-Z coordinates from robt')
ylabel('posizione asse X target')
xlabel('distanza dal target-Z')
zlabel('posizione asse Y target')

Kids = figure(1).Children;
Kids(1).ZDir = 'reverse';
Kids(1).YDir = 'reverse';