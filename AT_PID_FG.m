%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID controllers for the Phase                      %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 30/04/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% 1 - Tratando o processo:

% Nesta etapa o processo é discretizado:
% Sendo:

    p1 = (1/2.5);
    p2 = (1/3.75);

    k = 2*p1*p2;
 
% Discretizamos o processo utilizando um segurador de ordem zero:

    s = tf('s');

    ft = k/((s+p1)*(s+p2))

    ftz = c2d(ft,Tc,'zoh')

%% 2 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

n = 200; % Numero de pontos de análise

eps = 0.2; 
d = 0.5;

nptos = 1000

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 

[yr,ur] = rele_h(n,Tc,dh,eps,[p1,p2],k); 

figure;
grid;
plot(yr,'c-');
hold on;
plot(ur);

%% 3 Identificar os parametros a partir do experimento com relé:

[gw,w,arm,Kp]=identificar(n, dh, eps,Tc,yr);

Ku = -1/gw;
Tu = w/(2*pi);

L = 2;

c = 1/Kp;
b = sin(w*L)/(w*Ku);
a = (c + cos(w*L))/(w^2);

%% Definições do controlador AT-PID-FG: 

Am = 3;

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;

