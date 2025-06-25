clc
clear all

%datos de la planta

m = 0.06385 ; % masa de la bola blanca [Kg]
%x = 1; % posicion inicial de la Bola [m]
x0 = 0;
g = 9.81; % aceleracion gravedad [m/s^2]
Fg = 0.6263; % fuerza gravitacional [N]
Rb = 21; % radio de la bola [mm]21
Rr = 21; % radio de giro [mm]21
Jb = (305490.04e-12)*0.1; % Inercia de la bola [m^4]
Jv = 13576.379e-12; % Inercia de la viga [m^4]

%Tm=(3*0.0980665)*0.70588235;  % torque del motor 
Tm = 0.41; % torque del motor [N*m]
B = 0.3441; % coeficiente de fricción [N.m.s] 0.3441
R = 1.6;   % resistencia constante [Ω]
L = 0.52; % inductancia constante [H]
Km = 0.147;  % constante de torque [N.m/A]
Ka = 0.2335;  % constante contraelectromotriz [V/rad/s]
J = 0.0112; % momento de inercia del motor [kg.m^2]

%validación posición pelota 
%grafica1 = readmatrix("Experimento 5 grados.xlsx")
%grafica2 = readmatrix("Experimento 10 grados.xlsx")
%grafica3 = readmatrix("Experimento 15 grados.xlsx")

%validación angulo viga
%grafica4 = readmatrix("Experimento Angulo Viga.xlsx")
%grafica5 = readmatrix("Caracterización Motor1.xlsx")



% Cargando funcion de transferencia del motor
Ts = 0.01;
s = tf('s');
num_mot = Km * J * s + Km * B;
den_mot = L * J * s^2 + ((R * J) + (L * B)) * s + (R * B) + (Km * Ka);
FT_motor = tf(num_mot, den_mot);
FT_motorz = c2d(FT_motor, Ts, 'tustin');;

% Cargando funcion de transferencia de la bola y viga
num_viga = Jv;
den_viga = (m + (Jb / Rr^2));
%FT_bola_viga = ft(num_viga, den_viga);


%FT_total = FT_motor * FT_bola_viga;
%FT_totalz = c2d(FT_total, Ts, 'tustin');


%δθ'' Jv  + 2  δθ' δx' x m= -Fg

num = [0.0016464, 0.050583];
den = [0.005824, 0.19685, 0.58488];
tf_sys = tf(num, den);
pole(tf_sys)