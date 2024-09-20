% Ex1 tempo continuo
% sistema resposta impulso h(t)=exp(-t)u(t)
num=1;
den=[1 1];
sys=tf(num,den);
[h,t]=impulse(sys);

% grafico
figure(1)
set(gca,'FontSize',18)
plot(t,h)
ylabel('h(t)')
xlabel('t(s)')

% entrada senoidal
t=0:0.01:20;
w0=3;
x=cos(w0*t);
y=lsim(sys,x,t);

% grafico (regime transitorio)
figure(2)
set(gca,'FontSize',18)
plot(t(1:1500),x(1:1500),'b',t(1:1500),y(1:1500),'r')
grid
ylabel('x(t) e y(t)')
xlabel('t(s)')
axis([0 5 -1.2 1.2])

% determina ganho e fase diretamente da resposta em frequencia
j=sqrt(-1);

disp('ganho na frequencia w0')
KH=abs(1/(j*w0+1))
disp('fase em radianos na frequencia w0')
phaseH=phase(1/(j*w0+1))

%%
% Ex2 tempo discreto
% y[n]-(3/4)y[n-1]+(1/8)y[n-2]=2x[n]
% entrada senoidal
k=0:1000;
w0=pi/10;
x=cos(w0*k);

% simulacao com condicoes iniciais nulas
y(1)=0;
y(2)=0;
for n=3:length(k)
    y(n)=(3/4)*y(n-1)-(1/8)*y(n-2)+2*x(n);
end

% grafico (regime transitorio)
figure(3)
set(gca,'FontSize',18)
stem(k(1:50),x(1:50),'b');
hold on
stem(k(1:50),y(1:50),'r');
hold off

% determina ganho e fase diretamente da resposta em frequencia
j=sqrt(-1);

disp('ganho na frequencia w0')
KHd=abs(2/(1-(3/4)*exp(-j*1*w0)+(1/8)*exp(-j*2*w0)))
disp('fase em radianos na frequencia w0')
phaseH=phase(2/(1-(3/4)*exp(-j*1*w0)+(1/8)*exp(-j*2*w0)))

u=prbs(378,6,1);
u=u-0.5;
lu=length(u);
y=dlsim([1 0.5],[-1.5 0.7],u);
e=randn(378,1);
e=0.3*(e-mnan(e));
yi=dimpulse([1 0.5;1 -1.5 0.7],lu)
