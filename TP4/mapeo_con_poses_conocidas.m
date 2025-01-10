%% 1. Mapeo con poses conocidas

clc; clear all; close all;
%Mapa.
global dim_map
dim_map= 21;

%Mediciones:
mediciones = [101, 82, 91, 112, 99, 151, 96, 85, 99, 105];

%Inicializo el mapa
P_priori= 0.5;
m= P_priori*ones(1, dim_map);
m_bern= P_priori*ones(1, dim_map);

%Coordenadas de las celdas(10cm/celda).
c= [0:10:200];

xt= 1; %cm
%Chances logaritmicas
L0= Log_Odds(P_priori);
L=  L0*ones(1, dim_map);

%% Modelo del sensor:
% cada celda de la grilla con una distancia menor
% que la distancia medida se asume ocupada con una probabilidad de p = 0,3. Cada celda
% más allá de la distancia medida se asume ocupada con una probabilidad de p = 0,6.
% Las celdas ubicadas a más de 20cm por detrás de la distancia medida no cambian su
% probabilidad de ocupación.


%Occupancy grid mapping:
for zt= mediciones          % para cada medicion
    for i= 1:dim_map        % para cada grilla del mapa
        
        if c(i) > zt + 20   % si la medicion es no visible |zt.|10|10|c(i)|
            L= L + 0;
        else                % Si se percibe una medicion.
            
            L(i)= L(i) + inv_sensor_model(c(i), xt, zt) - L0;
            
            %Aplico el cambio en el mapa
            m(i)= invLog_Odds(L(i));
            m_bern(i)= binornd(1, m(i));
            pause(0.1)
            
            %Grafico del mapa.
            subplot(2,1,1);
            plot(c,m, '-o')
            subplot(2,1,2);
            plot(c,m_bern, '-o')
            title("Mapeo con poses conocidas")
            xlabel("Posición [cm]")
            ylabel("Probabilidad de ocupación ")
        end
    end
  
end 


%%
disp("Fin del mapping")
% m= invLog_Odds(L);
% 
    subplot(2,1,1);
    plot(c, m, '-o')
    title("Mapeo con poses conocidas")
    xlabel("Posición [cm]")
    ylabel("Probabilidad de ocupación ")
    
    subplot(2,1,2);
    plot(c, m_bern, '-o')
    title("Mapa aplicando bernoulli")
    xlabel("Posición [cm]")
    ylabel("Ocupacion de celda ")


function [L] = inv_sensor_model(m, xt, zt)
%Computo la chance logaritmica para la celda "m" de 10cm teniendo encuenta:
%     si m < z_t          --> p= 0.3
%     si m < z_t < m + 20 --> p= 0.6
%     e.o.c               --> p= 0.5 (se encarga el por fuera)

%    if (m <= zt)&&(zt < m + 10)   % las celdas son de 10cm.
    if zt < (m + 10) %<--- siempre mido hacia delante por cada celda pero en (m <= zt) no se sumple en m+20. 
        L= Log_Odds(0.6);
    else    
        L= Log_Odds(0.3);          % m < z_t
    end
end


function L = Log_Odds(p)
    L= log (p / (1 - p));
end

function p = invLog_Odds(l)
    p= 1 - 1./(1 + exp(l));
end