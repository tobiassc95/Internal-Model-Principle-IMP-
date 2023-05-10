clear; %clear the workspace.
clc;
close all;

ref=70*2*pi/60; %reference speed (7.33 rad/s).
s=tf('s');

%% plant.
P=2.62/(0.019*s+1);

[Ap,Bp,Cp,Dp]=ssdata(ss(P)); %plant in state space form.
Kp=acker(Ap,Bp,-10); %plant poleplacement gain.
T=tf(ss(Ap-Bp*Kp,Bp,Cp,Dp));
Kpre=1/evalfr(T,0); %preamplifier (assuming step input).
Jp=acker(Ap',Cp',-100)'; %plant observer gain.

%% disturbances.
Vstep=1/s;
Vsin=2*ref/(s^2+(2*ref)^2);
V=[Vsin Vstep];

[Av,Bv,Cv,Dv]=ssdata(ss(V)); %disturbances in state space form.
Jv=acker(Av',Cv',[-10 -10 -10])'; %disturbances observer gain.
