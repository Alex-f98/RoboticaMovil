clc; clear all; close all;
%% 1. Transformaciones 2D y matrices afines
% _ 1. Estando el robot en la pose _$x_1 = (x_1, y_1, \theta _1)^T$_, detecta 
% un obstaculo p en la posicion _
% 
% $(p_x, p_y)$_ con respecto a su propia terna de referencia. _
% 
% _Usar la matriz _$T_1$_ para expresar las coordenadas de p con respecto 
% a la terna global._

%Dado que no se dan valores nomericos se procede a resolverlo en forma
%simbolica.
syms theta1 x1 y1 px py
assume([px py], 'real')
assume([x1 y1], 'real')

%% 
% $p= \pmatrix{px \cr px \cr 1}$ sera la coordenada generalizada del obtaculo 
% respecto de la terna del robot
% 
% $T_1= \pmatrix{cos(\theta_1) & -sen(\theta_1)  & x_1\cr sen(\theta_1) & 
% cos(\theta_1) & y1\cr 0 & 0 & 1}$ será la matriz que roto-traslación que me 
% pasa las coordenas del robot a las coordenas globales.
% 
% entonce si quiero la posicion delo obstaculo p respecto de la terna global 
% sera:
% 
% $$p_{gbl}= T_1 .p= \pmatrix{cos(\theta) & -sen(\theta)  & x_1\cr sen(\theta) 
% & cos(\theta) & y1\cr 0 & 0 & 1}\pmatrix{p_x\cr py\cr 1}= \pmatrix{ x_1 + p_x.cos(\theta 
% _1) - p_ysen(\theta_1) \cr y_1 + p_y.cos(\theta_1) + p_x.sen(\theta_1) \cr                                    
% 1}$$
% 
% esto tambien se puede hacer en codigo:

%posicion del robot respecto de la terna global
p= [px py 1]';
%posicion del robot respecto de la terna global
T1= RoTrs(theta1, [x1 y1]')
%posicion del obstaculo respecto de la terna global
p_gbl= T1*p

%% 
% _2. Dada las coordenadas de un obstaculo en la terna global, ¿como pueden 
% calcularse las coordenadas de dicho obstaculo que el robot va a medir en su 
% propia terna?_
% 
% RTA:
% 
% Facil, busco la matriz inversa de roto-traslacion que me lleve lo medido 
% desde la terna global a la terna del robot.
% 
% Esto seria la inversa de T1 el cual se puede escribir en general:
% 
% $A^{-1}= \pmatrix{R^T & -R^T t \cr 0 & 1}$donde *R* es mi matriz de rotacion 
% y *t* es la posicion.
% 
% ${T_1}^{-1}= \pmatrix{cos(\theta _1) &sen(\theta_1) & y_1.sen(\theta_1) 
% - x_1.cos(\theta_1)\cr -sen(\theta_1) & cos(\theta_1) &-y_1.cos(\theta_1) - 
% x_1.sen(\theta_1)\cr 0 & 0 & 1}$entonces para obtener las coordenas del obstaculo 
% respecto de la terna del robot:
% 
% $$p= {T_1}^{-1}.p_{gbl}= \pmatrix{cos(\theta _1) &sen(\theta_1) & y_1.sen(\theta_1) 
% - x_1.cos(\theta_1)\cr -sen(\theta_1) & cos(\theta_1) &-y_1.cos(\theta_1) - 
% x_1.sen(\theta_1)\cr 0 & 0 & 1}\pmatrix{p_{xG} \cr p_{yG} \cr 1}$$
% 
% (tener en cuenta que mi posicion t seria la 1er y 2da fila de p)
% 
% podemos corroborarlo con codigo, deberia poder recuperar *p* :

%la inversa de A se lo calcula directamente.
p_rbt= simplify(T1\p_gbl) %inv(T1)*p_gbl

%% 
% _3. El robot se mueve a una nueva pose _$x_2 = (x_2, y_2, \theta _2)^T$_ 
% en la terna global. Encontrar la matriz de transformación _$T_{12}$_ que representa 
% la nueva pose con respecto a _$x_1$_._
% 
% RTA:
% 
% Esto se piensa asi: se proyecta las nuevas ternas de $X_2$ en $X_1$ y la 
% transformacion que hace eso se llamará $T_{12}$, y la proyección de $X_2$ en 
% la terna global la llamaremos$ T_2= \pmatrix{cos(\theta_2) & -sen(\theta_2)  
% & x_2\cr sen(\theta_2) & cos(\theta_2) & y_2\cr 0 & 0 & 1}$ entonces razonado 
% se puede llegar a la siguiente relacion:
% 
% $T_2= T_1.T_{12}$ por lo cual el problema queda resuelto, dado que existe 
% la inversa de $T_1$
% 
% $$T_{12}= {T_1}^{-1}T_2= \pmatrix{cos(\theta _1) &sen(\theta_1) & y_1.sen(\theta_1) 
% - x_1.cos(\theta_1)\cr -sen(\theta_1) & cos(\theta_1) &-y_1.cos(\theta_1) - 
% x_1.sen(\theta_1)\cr 0 & 0 & 1}\pmatrix{cos(\theta_2) & -sen(\theta_2)  & x_2\cr 
% sen(\theta_2) & cos(\theta_2) & y_2\cr 0 & 0 & 1}$$
% 
% comprobemos esto en codigo:

syms theta2 x2 y2
assume([x2 y2], 'real')
T2= RoTrs(theta2, [x2 y2]')
T12= simplify(T1\T2) %inv(T1)*T2
%% 
% _4. Estando ahora el robot en la posición _$x_2$_, ¿dónde está el obstáculo 
% p con respecto a la nueva terna local del robot? _
% 
% RTA:
% 
% Hay muchos caminos, se podria calcular la matriz que pasa de la terna 1 
% a la 2 o ya teniendo el obstaculo en la terna global calcularla respecto de 
% la terna 2 y sus variantes.
% 
% $p_2= {A_{12}}^{-1} p\\p_2= {A_2}^{-1} p_{gbl}$ (tener en cuenta que mi 
% posicion t seria la 1er y 2da fila de p)
% 
% Dependiendo el caso se podría utilizar la primera o la segunda.
%%
p2_= simplify(T12\p)        %inv(T12)*p
p2=  simplify(T2\p_gbl)     %inv(T2)*p_gbl
%Notar que ambos resultados son equivalentes.
%% 2. Sensado

%POSE robot
xr= 5; yr= -7; theta_r= -pi/4; % [m m rad/s]
%POSE LIDAR
xl= 0.2; yl=0.0; theta_l= pi;  % [m m rad/s]
%. La primera medición se toma para el ángulo ? = ??/2(según la
% terna del sensor) y la última se toma para el ángulo ? =?/2
%% 
% _1. Graficar las mediciones en la terna de referencia del LIDAR._
% 
% _Para ello hay que tener en cuenta el archivo "lasercan.dat" el cual contiene 
% los archivos de lectura de un lidar, por lo que el archivo contiene  lecturas 
% de distancia._
% 
% entonces primero se cargan los datos:

%Los datos de Lidar son un array de lecturas de distancias
scan = load('laserscan.dat','-ascii');
%calculo los angulo correspondientes.
angle = linspace(-pi/2, pi/2, size(scan,2));
%% 
% Se procede a graficar las posiciones en funcion de los del angulo en un 
% grafico cartesiano  para apreciar mejor los datos necesarios y en un grafico 
% polar para tener una mejor comprension visual de lo que realmente sucede.

figure()
plot(angle*180/pi, scan,'-')
axis([-90 90 0 15])
title('distancias vs angulo')
xlabel('Apertura[gr]')
ylabel('Distacion[m]')
grid('minor')

figure()
title('distancias vs angulo')
polarplot(angle,scan,'-')
%
%% 
% _2. ¿Cómo podría interpretarse el entorno a partir de estas mediciones? 
% _
% 
% *RTA:*
% 
% Un LIDAR devuelve la distancia de un objeto con respecto al LIDAR.
% 
% "La distancia al objeto se determina midiendo el tiempo de retraso entre 
% la emisión del pulso y su detección a través de la señal reflejada".
% 
% Observando las imagenes se puede interpretar  que desde -90° a -40° hay 
% un un objeto muy proximo,largo, por un ello luego objetos distantes pero 20° 
% vuelve a aparecer otro objeto, puntual, y luego sigue detectando el entorno 
% parece estar compuesto por paredes con cierto espesor.
% 
% _3. Usar las transformaciones homogéneas para calcular y graficar: _
% 
% _a) La posición del robot en la terna global._

T_0R= RoTrs(theta_r, [xr yr]')
%
xrG= T_0R(1,3);
yrG= T_0R(2,3);
%% 
% _b) La posición del LIDAR en la terna global._

T_RL= RoTrs(theta_l, [xl yl]')
T_0L= T_0R*T_RL
%
xlG= T_0L(1,3);
ylG= T_0L(2,3);

%% 
% _c) Las mediciones en la terna global._

%creo vectores para las posiciones cartesianas respecto del lidar
xL=      zeros(1,size(scan,2));
yL=      zeros(1,size(scan,2));
%creo vectores para las posiciones cartesianas respecto de la terna Global
pos_G=      zeros(3,size(scan,2));

%Obtengo las posiciones cartesianas respecto de la terna Global.
for i= 1:size(scan,2)
    xL(i)= scan(i)*cos(angle(i));
    yL(i)= scan(i)*sin(angle(i));
    
    pos_G(:,i)= T_0L*[xL(i) yL(i) 1]';
end


figure()
hold on
plot(xrG,yrG, 'r*')
plot(xlG,ylG, 'bo')
plot(pos_G(1,:),pos_G(2,:))
legend('Robot', 'Lidar', 'Distancias')
xlabel('Posición en X_{gbl} [m]')
ylabel('Posición en Y_{gbl} [m]')

figure()
hold on
plot(xrG,yrG, 'r*')
plot(xlG,ylG, 'ko')
plot(pos_G(1,:),pos_G(2,:))
legend('Robot', 'Lidar', 'Distancias')
xlabel('Posición en X_{gbl} [m]')
ylabel('Posición en Y_{gbl} [m]')
axis([4 6 -8 -4])
%% 
% Observar las imagenes, claramenete se puede ver como graficando las distancias 
% se observan las formas procedentes de los rebotees con los distintos objetos 
% e incluso parte del mismo robot  en las proximidades del LIDAR. Ademas se puede 
% observar la posicion en la que se encuentra el robot y el sensor LIDAR con su 
% particular rotacion proveniente de la rotacion del robot respecto de la terna 
% global, ?/4.
%% 3. Accionamiento diferencial
% _1. El encabezado de la función debe tener esta forma: function *[x_n y_n 
% theta n] = diffdrive(x, y, theta, v_l, v_r, t, l)* donde* x, y*, theta es la 
% pose del robot,* v_l* y* v_r* son las velocidades de la rueda izquierda y derecha, 
% *t* es el intervalo de tiempo en movimiento y *l* es la distancia entre ruedas 
% del robot._
% 
% _ La salida de la función es una nueva pose del robot dada por *x_n*, *y_n,theta_n.* 
% _
%%
x= 1.5; y= 2.0; theta= pi/3; %[m m rad/s]
l= 0.5; %m

% a) c1 = (vl = 0,1m/s, vr = 0,5m/s, t = 2s)
% b) c2 = (vl = 0,5m/s, vr = 0,1m/s, t = 2s)
% c) c3 = (vl = 0,2m/s, vr = ?0,2m/s, t = 2s)
% d) c4 = (vl = 0,1m/s, vr = 0,6m/s, t = 6s)
% e) c5 = (vl = 0,4m/s, vr = 0,4m/s, t = 2s)
C= [0.1  0.5 2;
    0.5  0.1 2;
    0.2 -0.2 2;
    0.1  0.6 6;
    0.4  0.4 2];
    x_n= zeros(1,6); y_n= x_n;  theta_n= x_n;
    x_n(1)= x; y_n= y; theta_n= theta;
   
for k=1:5
    [x_n(k+1), y_n(k+1), theta_n(k+1)] = diffdrive(x, y, theta, C(k,1), C(k,2), C(k,3), l);
end

figure()
%quiver(x_n, y_n, gradient(x_n), gradient(y_n))
hold on
plot(x_n, y_n,'-.b','LineWidth',3)
plot(x_n(1), y_n(1),'-*r', 'LineWidth',9)
plot(x_n(2), y_n(2),'-*k', 'LineWidth',4)
plot(x_n(3), y_n(3),'-*g', 'LineWidth',4)
plot(x_n(4), y_n(4),'-oc', 'LineWidth',4)
plot(x_n(5), y_n(5),'-*m', 'LineWidth',4)
plot(x_n(6), y_n(6),'-*b', 'LineWidth',4)
legend('Trayectoria','Init', 'a', 'b', 'c', 'd', 'Final_e', 'Location','northwest')
title('Movimiento 2D')
grid('minor')
xlabel('Posición x [m]')
ylabel('Posición y [m]')


%% 
% 
%%
function [x_n, y_n, theta_n] = diffdrive(x, y, theta, v_l, v_r, t, l)
    
    if v_l==v_r
        R=10e20;  %esto será mi infinito(RngMatlab: 10e-308 a 10e+308)
    else
        R= l*(v_l + v_r)/(2*(v_r - v_l));
    end
    
    ICC= [x-R*sin(theta), y + R*cos(theta)];
    w= (v_r - v_l)/l;
    %v= (v_r + v_l)/2;
    
    POSEn= [cos(w*t) -sin(w*t) 0; sin(w*t) cos(w*t) 0; 0 0 1] * [x-ICC(1,1); y-ICC(1,2); theta] + [ICC(1,1); ICC(1,2); w*t];

    x_n  =        POSEn(1,1);
    y_n  =        POSEn(2,1);
    theta_n  =    POSEn(3,1);  
end

function R = Rot(theta)
R= [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
end

function RT = RoTrs(theta, pos)
RT= [Rot(theta) pos(:);
    zeros(1,2) 1.0];
end