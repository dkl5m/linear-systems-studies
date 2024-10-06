%% Exemplo livro Chen: Pendulo invertido em um carro.
% Parte 1
%       Tentei fazer o do robo com motor de inducao,
%       mas nao consegui terminar.

% states: x = [x; xdot; theta; thetadot]
% x = posicao do carro
% theta = angulo do braco do pendulo


clear; clc; close;
% -----------------------------------------------------------------------

% Informacoes sistema
m = 1;      % massa pendulo
M = 5;      % massa carro
L = 2;      % comprimento cabo pendulo
g = -10;    % constante gravidade
d = 1;      % atrito

% Sistema nao controlado
tspan = 0:0.1:40;
y0 = [0; 0; pi; 0.5];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);

figure(1)   % grid de graficos 1
subplot(2,2,1), plot(t,y(:,1)), title('x'), grid
subplot(2,2,2), plot(t,y(:,2)), title('dot(x)'), grid
subplot(2,2,3), plot(t,y(:,3)), title('\theta'), grid
subplot(2,2,4), plot(t,y(:,4)), title('dot(\theta)'), grid


% -----------------------------------------------------------------------

% Linearizar sistema
s = 1; % pendulo para cima (s=1)
       % pendulo para baixo (s=-1)
A = [0 1 0 0; 0 -d/M -m*g/M 0; 0 0 0 1; 0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; s*1/(M*L)];
C = eye(4);
D = 0;

eig(A)
sys = ss(A,B,C,D);

tspan = 0:0.001:40;
if(s==-1)
    y0 = [0; 0; 0; 1.5];
    [yL,t,xL] = initial(sys,y0,tspan);
    [t,yNL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);
elseif(s==1)
    y0 = [0; 0; pi + 0.0001; 0];
    [yL,t,xL] = initial(sys,y0-[0; 0; pi; 0],tspan);
    [t,yNL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);
end

figure(2)   % grid de graficos 2
subplot(2,2,1), plot(t,yNL(:,1)), title('x'), grid
subplot(2,2,2), plot(t,yNL(:,2)), title('dot(x)'), grid
subplot(2,2,3), plot(t,yNL(:,3)), title('\theta'), grid
subplot(2,2,4), plot(t,yNL(:,4)), title('dot(\theta)'), grid


% -----------------------------------------------------------------------

% Alocacao polos
rank(ctrb(A,B)) % sistema controlavel
p = [-1; -1.1; -1.2; -1.3]; % vetor eigenvalues desejados
K = place(A,B,p);

tspan = 0:.001:20;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,yL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi + 0.1; 0];
    [t,yL] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
end

figure(3)   % grid de graficos 3
subplot(2,2,1), plot(t,yL(:,1)), title('x'), grid
subplot(2,2,2), plot(t,yL(:,2)), title('dot(x)'), grid
subplot(2,2,3), plot(t,yL(:,3)), title('\theta'), grid
subplot(2,2,4), plot(t,yL(:,4)), title('dot(\theta)'), grid


% -----------------------------------------------------------------------

% Modelo LQR
Q = [1 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 100];
R = 0.0001;
det(ctrb(A,B))
K = lqr(A,B,Q,R);

tspan = 0:.001:20;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,yLQR] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi + 0.1; 0];
    [t,yLQR] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
end

figure(4)   % grid de graficos 4
subplot(2,2,1), plot(t,yLQR(:,1)), title('x'), grid
subplot(2,2,2), plot(t,yLQR(:,2)), title('dot(x)'), grid
subplot(2,2,3), plot(t,yLQR(:,3)), title('\theta'), grid
subplot(2,2,4), plot(t,yLQR(:,4)), title('dot(\theta)'), grid

% Pole placing melhor no caso de controole de x
% Modelo LQR melhor para controlar theta