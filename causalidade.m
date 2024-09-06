% definicao degrau unitario 11 elementos
% corresponde ao tempo n=0 a n=10
u=ones(1,11);

% sinal aberto aa direita
% para n=1
h(1)=0.5*u(1);
for n=2:11
    h(n)=(0.5)^n*u(n) + (1/3)*0.5^(n-1)*u(n-1);
end
subplot(211)
stem(0:10,h)

% sinal aberto aa esquerda
% para n=-1
h(1)=-(0.5)^(-1) - (1/3)*0.5^(-2)*u(1);
for n=-2:-1:-11
    h(-n)=-(0.5)^n*u(-n-1) - (1/3)*0.5^(n-1)*u(-n);
end
subplot(212)
stem(0:-1:-10,h)