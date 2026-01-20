clc; clear all; close all;
%Parametros
global M F m L J S g
%Carro
M=0.301;   %[Kg]
F=5;    %[N*s/m] 
%Pendulo
m=0.101;   %[Kg]
L=0.3;     %[m]
J=0.01;  %[Kg*m^2] 
S=0.00001; %[N*m*s/rad] 
g=9.81;    %[m/s^2]
%Constantes auxiliares
Tm=0.01;   %[s]
%Modelado y discretización
dt= J*(M+m)+M*m*L^2;
A = [0, 1, 0, 0; 0, -F*(J+m*L^2)/dt, -m^2*L^2*g/dt, m*L*F*S/dt;
      0, 0, 0, 1; 0, m*L*F/dt, (M+m)*m*L*g/dt, -S*(M+m)/dt ];
B = [0; (J+m*L^2)/dt; 0; -m*L/dt];
C=[1 0 0 0; 0 0 1 0];
D=0;
[Ad,Bd]=c2d(A,B,Tm);
%Matrices de costo
Q=diag([5,0,10,5]);
R=10;
%Calculo de ganancias
P=zeros(size(Ad));
for i=1:3000
    P=Q+Ad'*P*Ad-Ad'*P*Bd*inv(R+Bd'*P*Bd)*Bd'*P*Ad;
end
K=inv(R+Bd'*P*Bd)*Bd'*P*Ad;
disp(P)
disp(K)


