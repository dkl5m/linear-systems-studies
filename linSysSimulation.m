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