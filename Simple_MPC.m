%% Programa de control predictivo usando casadi como un solver
%% Metodo de discretizacion simple usando euler

%% Eliminar variables del sistema
clear all;
close all;
clc;

%% Generacion de los tiempos del sistema
ts = 0.1; 
tfinal = 15;
t = [0:ts:tfinal];

%% Definicion del horizonte de prediccion
N = 20; 

%% Dimensiones del robot
a = 0.1;
%% Restricciones acciones de control
bounded =[0.2;-0.2;2.50;-2.50];

%% Matrices del sistema
Q=0.01*eye(3);
R =0.03*eye(2);
Q(1,1)=1;
Q(2,2)=1;
%% Generacion del sistema
[f,ff, solver, args] = MPC_Solver(bounded, a, N, ts,Q, R);

%% Definciond de los estados iniciales del sistema
x = 0;
y = 0;
th = zeros(1,length(t)+1);

%% Definicoion del vector para la integracion numerica
hx = zeros(1,length(t)+1);
hy = zeros(1,length(t)+1);

%% Definicion de la condicion inicial del sistema
hx(1,1)=x+a*cos(th(1));
hy(1,1)=y+a*sin(th(1));

%% Definicion del vector de control inicial
v = zeros(N,2);  % two control inputs 

%% Definicion de los estados deseados del sistema
hxd=1*ones(1,length(t));
hyd=1*ones(1,length(t));
phid=pi/2*ones(1,length(t));

for k=1:length(t)
    %% definir el vector de estados del sistema
    hd=[hxd(1,k);hyd(1,k);phid(k)];
    
    h=[hx(1,k);hy(1,k);th(1,k)];
    
    he(:,k)=hd-h;
    %% Definicon del sistema con optimizacion
    args.p   = [h;hd]; 
    args.x0 = reshape(v',2*N,1); % initial value of the optimization variables
    tic;
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    sample(k)=toc;
    opti = reshape(full(sol.x)',2,N)';
    
    qpref=[opti(1,1);opti(1,2)];
    
    u(k) =qpref(1);
    w(k) =qpref(2);
    
    %% Simulacion del sistema
    h=h+system(h,qpref,f,ts);
    
    hx(1,k+1) =h(1);
    hy(1,k+1) =h(2);
    th(1,k+1) =h(3);
    
    v = [opti(2:end,:);opti(end,:)];
end

figure(1)
plot(hx,hy,'b-')
grid on;
hold on;
plot(hxd(1:length(t)-1),hyd(1:length(t)-1),'g-')

figure(2)
plot(t,u,'b-')
grid on;
hold on;
plot(t,w,'r-')

figure(3)
plot(t,sample,'b-')
grid on;
hold on;

figure(4)
plot(t,he(1,:),'b-')
grid on;
hold on;
plot(t,he(2,:),'r-')