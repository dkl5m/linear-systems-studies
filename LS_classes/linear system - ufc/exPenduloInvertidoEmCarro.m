%% Exemplo livro Chen: Pendulo invertido em um carro.
%       Tentei fazer o do robo com motor de inducao,
%       mas nao consegui terminar.

% states: x = [x; xdot; theta; thetadot]
% x = posicao do carro
% theta = angulo do braco do pendulo

%% 1) Controlabilidade
clear all; close all; clc;

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

ctrb(A,B)
if det(ctrb(A,B)) ~= 0
    "Is controllable"
else
    "Is not controllable"
end

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100]; % thetadot eh o estado mais importante
% seguido de theta

R = .001; % fazer o controle eh barato

K = lqr(A,B,Q,R)

tspan = 0:.001:10;
eig(A-B*K)
% o eig mais agressivo eh o thetadot
[T,D] = eig(A-B*K)

diag(real(D))
T(:,1)

% mostra que as direcoes mais estabilizaveis sao
% xdot e thetadot

%% 2) Observabilidade
clear all; close all; clc;

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C1 = [1 0 0 0]; % medida x

obsv(A,C1)
if det(obsv(A,C1)) ~= 0
    "X is observable"
else
    "X is not observable"
end

C2 = [0 1 0 0]; % medida xdot

obsv(A,C2)
if det(obsv(A,C2)) ~= 0
    "Xdot is observable"
else
    "Xdot is not observable"
end

C3 = [0 0 1 0]; % medida theta

obsv(A,C3)
if det(obsv(A,C3)) ~= 0
    "Theta is observable"
else
    "Theta is not observable"
end

C4 = [0 0 0 1]; % medida thetadot

obsv(A,C4)
if det(obsv(A,C4)) ~= 0
    "Thetadot is observable"
else
    "Thetadot is not observable"
end

% Sistema so eh observavel em relacao a posicao do carro,
% em C = [1 0 0 0].

%% 3) Observabilidade do modelo reduzido
% Desconsiderando a posicao do carro, eh possivel estabilizar
% o pendulo? Quais medidas sao melhores?

clear all; close all; clc;

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = -1; % pendulo para baixo (s=1),
% devido ao calculo do Gramiano, pois nao pode usar
% em sistemas com instabilidade

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

% Fazer subsistema
A1 = A(2:end,2:end);

B = B(2:end);

C1 = [1 0 0]; % medida xdot

obsv(A1,C1)
if det(obsv(A1,C1)) ~= 0
    "Xdot is observable"
else
    "Xdot is not observable"
end

C2 = [0 1 0]; % medida theta

obsv(A1,C2)
if det(obsv(A1,C2)) ~= 0
    "Xdot is observable"
else
    "Xdot is not observable"
end

C3 = [0 0 1]; % medida thetadot

obsv(A1,C3)
if det(obsv(A1,C3)) ~= 0
    "Xdot is observable"
else
    "Xdot is not observable"
end

% Subsistema eh observavel em todos os estados

D = zeros(size(C1,1), size(B,2));
sys1 = ss(A1,B,C1,D); % sistema com medida xdot
det(gram(sys1,"o")) % volume do elipsoide do gramiano
% quanto maior, melhor

sys2 = ss(A1,B,C2,D); % sistema com medida theta
det(gram(sys2,"o"))

sys3 = ss(A1,B,C3,D); % sistema com medida thetadot
det(gram(sys3,"o"))

% Ve-se que o estado xdot tem um maior gramiano
% quer dizer que eh possivel que seja mais facil estimar
% o estado completo se medir xdot.