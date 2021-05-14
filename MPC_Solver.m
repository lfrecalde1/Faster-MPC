function [f,ff,solver,args] = MPC_Solver(bounded, a, N, ts, Q, R)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;
%% Definicion de las restricciones en las acciones de control
u_max = bounded(1); 
u_min = bounded(2);
w_max = bounded(3); 
w_min = bounded(4);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y'); 
th = SX.sym('th');

%% Definicion de cuantos estados en el sistema
states = [x;y;th];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
u = SX.sym('u');
w = SX.sym('w');

%% Defincion de cuantas acciones del control tiene el sistema
control = [u;w]; 
n_control = length(control);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J = [u*cos(th)-a*sin(th)*w;...
     u*sin(th)+a*cos(th)*w;w]; 

%% Definir la funcion que describe al sistema
f = Function('f',{states,control},{J});

%% Defincion de las acciones de control
V = SX.sym('V',n_control,N); 

%% Defincion del vector que contiene el inicial y el deseado como es control de pose 
%% No trayectoria solo tiene x y th que son fijos y no cambian en el tiempo
Z = SX.sym('Z',n_states + n_states);

%% Definicion del vector para las predicciones en el sistema
H = SX.sym('H',n_states,(N+1));

%% Definciond e una funcionque nos perimte saber la evoluciondel sistema N pasos  adelante
H(:,1) = Z(1:3); % initial state
for k = 1:N
    H(:,k+1) = H(:,k)+f(H(:,k),V(:,k))*ts;
end
%% Definicion de la funcion para la evolucion del sistema
ff=Function('ff',{V,Z},{H});

%% Definicion de las funciones costoa  minimizar
obj = 0;  %% valor inicial de la funcion costo
g = [];  


% compute objective
for k=1:N
    st = H(:,k);  con = V(:,k);
    obj = obj+(st-Z(4:6))'*Q*(st-Z(4:6)) + con'*R*con; % calculate obj
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; H(1,k)];   %state x
    g = [g ; H(2,k)];   %state y
end
% make the decision variables one column vector
OPT_variables = reshape(V,2*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', Z);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = -inf;  % lower bound of the states x and y
args.ubg = inf;   % upper bound of the states x and y 

% input constraints
args.lbx(1:2:2*N-1,1) = u_min; args.lbx(2:2:2*N,1)   = w_min;
args.ubx(1:2:2*N-1,1) = u_max; args.ubx(2:2:2*N,1)   = w_max;


end

