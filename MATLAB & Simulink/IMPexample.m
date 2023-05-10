clear; %clear the workspace.
clc;
close all;

ref = 70*2*pi/60; %reference speed (rad/s).
s = tf('s'); %laplace variable.
P = zpk(2.62/(0.019*s+1)); %DC motor wtih encoder.
Ts = 0.01; %period sample.
pade = zpk((1-s*Ts/4)/(1+s*Ts/4)); %padé approximation.
bode(P);
title('bode(P)');
grid on;
figure;
step(P);
title('step(P)');
grid on;

k = 1500; %0.35; %0.19;
C = zpk(k*(s/8+1)^2*(0.019*s+1)/s*1/(s^2+(2*ref)^2));
%C = zpk(k*(s/10+1)/s*1/(s^2+(2*ref)^2)); %PI.
% figure;
% bode(C);
% title('bode(C)');
% grid on;

L = minreal(C*pade*P)
figure;
bode(L);
title('bode(L)');
grid on;
figure;
nyquist(L);
%nyqlog(L);
title('nyqlog(L)');
figure;
rlocus(L);
title('rlocus(L)');

S = feedback(1,L);
%pole(S)
figure;
bode(S);
title('bode(S)');
grid on;

T = minreal(1-S)
%pole(T)
figure;
bode(T);
title('bode(T)');
grid on;
figure;
step(T);
title('step(T)');
grid on;

PS = minreal(P*S);
%pole(PS)
figure;
bode(PS);
title('bode(PS)');
grid on;

CS = minreal(C*S);
%pole(CS)
figure;
bode(CS);
title('bode(CS)');
grid on;

%after designing, get Cd.
%Cd = c2d(C,Ts,'tustin')
%Cd = c2d(C,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc/(tan(wc*Ts/2))));
wc = 2*ref; %122; %maximum sensitivity frequency (measured using bode(S)).
Cd = c2d(C,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc))