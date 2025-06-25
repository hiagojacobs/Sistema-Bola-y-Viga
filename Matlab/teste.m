%Programa para calculo de constantes K e A para caracterização do motor
%Necessário implementar um controlador Kp para controlar posição e tirar
%valores praticos de MP e tr.

s=tf('s');


%Tempo de subida
Tr=0.284; %Medidos no livro
%Tr=(1/Wd)*(pi-arctg((sqrt(1-zeta^2))/(zeta))); %Em segundos

% Overshoot
MP=15.3; %Medido no livro
%MP=100*exp^((-zeta*pi)/(sqrt(1-zeta^2))); %Em porcentagem

%Formulas para zeta
zeta=sqrt((log(MP/100)^2)/(log(MP/100)^2+pi^2));

%frequência de oscilação amortecida, é a parte imaginaria dos polos
%Wd = 1;
Wd=(1/Tr)*(pi-atan((sqrt(1-zeta^2))/(zeta))); %Em segundos

%Frecuencia natural
%Wn=;
Wn=Wd/(sqrt(1-zeta^2));

%Calculo constante "k"
kp=4.5; %ganho aplicado no teste pratico
k=Wn^2/kp;

%Calculo constante "a"
a=zeta*2*sqrt(kp*k);

%teste discretizando sistemas bola e viga
%num=[675.4471];
%den=[1 2.8681 0];
%T=0.1;
%G=tf(num,den,T);

%num=[13.162 13.162];
%den=[1 3.628]; 
%T=0.1;
%G=tf(num,den);
%Gd=c2d(G,T,'zoh');