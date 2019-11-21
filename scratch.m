%%

clear all; clc

Vx=60/3.6;
m=1600;
Iz=1705;
lf=1.0;
lr=1.0;
Caf=66243;
Car=66243;
dt=0.01;

A =          [0,                     1,              0,                 0;
              0,         -(2*Caf + 2*Car)/(m*Vx),    0,    -Vx-(2*Caf*lf-2*Car*lr)/(m*Vx);
              0,                     0,              0,                 1;
              0,       -(2*lf*Caf-2*lr*Car)/(Iz*Vx), 0,  -(2*lf^2*Caf+2*lr^2*Car)/(Iz*Vx)];

B =          [      0     ;
                 2*Caf/m  ;
                    0     ;
               2*lf*Caf/Iz];

C =          [1, 0, 0, 0;
              0, 0, 1, 0];

D = 0;

model = ss(A,B,C,D,dt)

Ad = model.A;
Bd = model.B;
Cd = model.C;

N = 10;
x = [0 0 0 0]';
u = 3*pi/180;
for i=1:N
    y = C*x;
    x = Ad*x + Bd*u;
end
    
%%