% Ex tempo continuo
% sistema resposta impulso h(t)=exp(-t)u(t)
num=1;
den=[1 1];
sys=tf(num,den);
[h,t]=impulse(sys);

figure(1)
set(gca,'FontSize',18)
plot(t,h)
ylabel('h(t)')
xlabel('t(s)')