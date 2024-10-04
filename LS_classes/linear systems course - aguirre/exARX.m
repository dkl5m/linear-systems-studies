% ARX
a1=1.5; a2=-0.7; % coef polinomio A
b1=1;   b2=0.5;  % coef polinomio B
% entrada degrau unitario atrasado de 10
u=[zeros(10,1); ones(40,1)];
% condicoes iniciais nulas
y=zeros(size(u));

for k=5:length(u)
    % atraso=1;
    y(k)=a1*y(k-1)+a2*y(k-2)+b1*u(k-1)+b2*u(k-2);
end
y1=y;

for k=5:length(u)
    % atraso=3
    y(k)=a1*y(k-1)+a2*y(k-2)+b1*u(k-3)+b2*u(k-4);
end

k=length(u);
% grafico (regime transitorio)
figure(1)
set(gca,'FontSize',18)
stem(u(1:50),'b');
hold on
stem(y1(1:50),'r');
stem(y(1:50),'g');
hold off
%plot(1:length(u),y1,'k',1:length(u),y,'b-');