%% Exemplo livro Chen: Pendulo invertido em um carro.
% Parte 2
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
s = 1;      % pendulo para cima (s=1)
            % pendulo para baixo (s=-1)
A = [0 1 0 0; 0 -d/M -m*g/M 0; 0 0 0 1; 0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; s*1/(M*L)];
C = eye(4);
D = 0;


% -----------------------------------------------------------------------

% Controlabilidade
rank(ctrb(A,B))           % rank cheio,sistema controlavel
Q = [1 0 0 0; 0 1 0 0;
    0 0 10 0; 0 0 0 100]; % thetadot >> theta
R = 0.001;                % diminui custo
K = lqr(A,B,Q,R)

tspan = 0:0.001:10;
[T,Dx] = eig(A-B*K);       % eig mais agressivo = thetadot
diag(real(Dx));
T(:,1)                    % direcoes mais estabilizaveis = xdot, thetadot


% -----------------------------------------------------------------------

% Observabilidade sistema completo
det(obsv(A,C(1,:)))       % det ~= 0, x observavel
det(obsv(A,C(2,:)))       % det = 0, xdot nao observavel
det(obsv(A,C(3,:)))       % det = 0, theta nao observavel
det(obsv(A,C(4,:)))       % det = 0, thetadot nao observavel
% sistema completo observavel apenas para x


% -----------------------------------------------------------------------

% Observabilidade sistema reduzido
% Fazer subsistema
A1 = A(2:end,2:end);      % retira medida x
C1 = eye(3);

det(obsv(A1,C1(1,:)))       % det ~= 0, xdot observavel
det(obsv(A1,C1(2,:)))       % det ~= 0, theta observavel
det(obsv(A1,C1(3,:)))       % det ~= 0, thetadot observavel
% sistema reduzido observavel para todos estados


% -----------------------------------------------------------------------
%%
% Utilizar sistema com pendulo para baixo para calcular gramiano e Kalman
% Filter, pois nao aceita modelos com dinamica instaveis.

% Informacoes sistema
m = 1;      % massa pendulo
M = 5;      % massa carro
L = 2;      % comprimento cabo pendulo
g = -10;    % constante gravidade
d = 1;      % atritos = -1;     % pendulo para baixo (s=-1)
            % pendulo para cima (s=1)
A = [0 1 0 0; 0 -d/M -m*g/M 0; 0 0 0 1; 0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; s*1/(M*L)];
C = eye(4);
D = 0;


% -----------------------------------------------------------------------

% Gramiano
A1 = A(2:end,2:end);                % retira medida x
C1 = eye(3);
B1 = B(2:end);                      % retira medida x
D1 = zeros(size(C1(1,:),1), size(B,2));

sys1 = ss(A1,B1,C1(1,:),D1);        % medida xdot
det(gram(sys1,"o"))                 % volume elipsoide gramiano xdot
sys2 = ss(A1,B1,C1(2,:),D1);        % medida theta
det(gram(sys2,"o"))                 % volume elipsoide gramiano theta
sys3 = ss(A1,B1,C1(3,:),D1);        % medida thetadot
det(gram(sys3,"o"))                 % volume elipsoide gramiano thetadot
% Estado xdot possui maior det gramiano. Quer dizer que eh possivel que
% seja mais facil estimar o estado completo do sistema se medir xdot.


% -----------------------------------------------------------------------

% Sistema aumentado com disturbio e ruido
Vd = 0.1*eye(4);                    % covariancia disturbio
Vn = 1;                             % covariancia ruido
BF = [B Vd 0*B];                    % entradas aumentadas
sysC = ss(A,BF,C(1,:),[0 0 0 0 0 Vn]);   % sistema ss SI
sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));
% sistema saida completa, disturbio, sem ruido


% -----------------------------------------------------------------------

% Filtro Kalman
[L,P,E] = lqe(A,Vd,C(1,:),Vd,Vn); % cria Kalman Filter
sysKF = ss(A-L*C(1,:),[B L],eye(4),0*[B L]); % Estimador Kalman Filter


% -----------------------------------------------------------------------

% Estima sistema linearizado na posicao para baixo
dt = 0.01; t = dt:dt:50;

uDIST = randn(4,size(t,2)); uNOISE = randn(size(t));
u = 0*t;
u(100:120) = 100;     % impulso
u(1500:1520) = -100;  % impulso

uAUG = [u; Vd*Vd*uDIST; uNOISE];

[y,t] = lsim(sysC,uAUG,t);
[xtrue,t] = lsim(sysFullOutput,uAUG,t);
[x,t] = lsim(sysKF,[u; y'],t);

plot(t,xtrue,'-',t,x,'--','LineWidth',2)

figure
plot(t,y), hold on, plot(t,xtrue(:,1),'r'), plot(t,x(:,1),'k--')

% Funciona. Falta aprender a juntar LQR com Kalman Filter para estimar
% sistema para cima com carro correndo