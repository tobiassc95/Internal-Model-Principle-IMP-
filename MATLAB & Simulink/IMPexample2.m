clear; %clear the workspace.
clc;
close all;

ref=70*2*pi/60; %reference speed (7.33 rad/s).
s=tf('s');
Ts=0.01;

%% plant.
P=2.62/(0.019*s+1);

[Ap,Bp,Cp,Dp]=ssdata(ss(P)); %plant in state space form.
Kp=acker(Ap,Bp,-10); %plant poleplacement gain.
T=tf(ss(Ap-Bp*Kp,Bp,Cp,Dp));
Kpre=1/evalfr(T,0); %preamplifier (assuming step input).
Jp=acker(Ap',Cp',-100)'; %plant observer gain.

Bpo=[Jp Bp];
Po=tf(ss(Ap-Bp*Kp-Jp*Cp,Bpo,Kp,zeros(size(Kp,1),size(Bpo,2)))); %plant observer.
%step(Po);
[Apo_,Bpo_,Cpo_,Dpo_]=ssdata(c2d(Po,Ts,'zoh')); %discrete plant observer.

%% disturbances.
Vstep=1/s;
Vsin=2*ref/(s^2+(2*ref)^2);
Vsin2=4*ref/(s^2+(4*ref)^2);
Vramp=1/s^2;
%V=[Vsin Vstep]; %V1
V=[Vsin Vramp]; %V2
%V=[Vsin Vsin2 Vstep]; %V3

[Av,Bv,Cv,Dv]=ssdata(ss(V)); %disturbances in state space form.
%Jv=acker(Av',Cv',[-10 -10 -10])'; %disturbances observer gain. V1
Jv=acker(Av',Cv',[-10 -10 -10 -10])'; %disturbances observer gain. V2
%Jv=acker(Av',Cv',[-10 -10 -10 -10 -10])'; %disturbances observer gain. V3

Vo=tf(ss(Av-Jv*Cv,Jv,Cv,zeros(size(Cv,1),size(Jv,2))));
%step(Vo);
[Avo_,Bvo_,Cvo_,Dvo_]=ssdata(c2d(Vo,Ts,'zoh')); %discrete disturbance observer.

%% observer
J=[Jp;Jv];
K=[Kp Cv];
Ao=[Ap-Bp*Kp-Jp*Cp zeros(size(Ap,1),size(Av,2));-Jv*Cp Av];
Bo=[Jp Bp;Jv zeros(size(Jv,1),size(Bp,2))];
%O=tf(ss(Ao,Bo,eye(size(Ao)),zeros(size(Bo))));
O=tf(ss(Ao,Bo,K,zeros(size(K,1),size(Bo,2)))); %plant+disturbances observer.
[Ao_,Bo_,Co_,Do_]=ssdata(c2d(O,Ts,'zoh')); %discrete plant+disturbances observer.

%%
% [Ap_,Bp_,Cp_,Dp_]=ssdata(c2d(P,Ts,'zoh')); %plant.
% Kp_=acker(Ap_,Bp_,exp(-5*Ts)); %plant poleplacement gain.
% Jp_=acker(Ap_',Cp_',exp(-100*Ts))'; %plant observer gain.
% [Av_,Bv_,Cv_,Dv_]=ssdata(c2d(V,Ts,'zoh')); %disturbance.
% Jv_=acker(Av',Cv',[exp(-50*Ts) exp(-50*Ts) exp(-50*Ts)])'; %disturbances observer gain.
