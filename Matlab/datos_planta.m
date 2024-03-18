clc
clear all

%datos de la planta

m = 0.0452 ; %masa de la pelota en Kg
r = 0 ; % posicion inicial de la Bola
Rb = 0.0428/2 ; % radio de la bola
s = 0.0093; % radio de giro *valor ficticio
g = 9.81; % aceleracion gravedad 

Km = 0.01;  % constante de torque [N.m/A]
Ka = 0.01;  % constante contraelectromotriz [V/rad/s]
R = 1;   % resistencia constante [Ω]
L = 0.5; % inductancia constante [H]
B = 0.1; % coeficiente de fricción [N.m.s]
J = 0.01; % momento de inercia del motor [kg.m2]


T = 4.02; % torque del motor
Tc = 1; % torque de carga acoplada al motor
Iv = 1; % Inercia de la viga
