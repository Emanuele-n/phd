clear;
clc
% Load the Symbolic Math Toolbox
syms xc xs xh Fp 'real';
syms Rc Rs Rh Ec Es Eh Ic Is Ih Ac As Ah 'real'

% % Define M and Y matrices
% M = [1/(Es*As), 1/(Ec*Ac)+1/(Es*As), 1/(Ec*Ac);
%     -1/(Es*As)-1/(Eh*Ah), -1/(Es*As), 1/(Eh*Ah);
%     -1/(Eh*Ah), 1/(Ec*Ac), 1/(Ec*Ac)+1/(Eh*Ah);
%     0, Rc/(Ec*Ic)+2*Rs/(Es*Is), -Rc/(Ec*Ic);
%     0, -2*Rs/(Es*Is), -2*Rh/(Eh*Ih);
%     0, Rc/(Ec*Ic), -Rc/(Ec*Ic)-2*Rh/(Eh*Ih);
% 
% ];
% 
% X = [xc; xs; xh];
% Y = [Fp/(Es*As), -Fp/(Es*As), 0, Rs*Fp/(Es*Is), -Rs*Fp/(Es*Is), 0]';
% 
% N = [M, Y];
% disp(M)
% disp(N)
% 
% % Rouchè-Capelli:
% % Unique solution if and only if rank(M) = rank(N) 
% % Infinite solutions if rank(M) < rank(N)
% % No solutions if rank(M) > rank(N)
% 
% disp('rank(M):')
% disp(rank(M))
% disp('rank(N):')
% disp(rank(N))
% 
% 
% % Solve the system of linear equations for X
% solution = simplify(subs(linsolve(M, Y)));
% disp(solution)
% 
% % Elongation only
% M = M((1:3),:);
% Y = Y(1:3);
% N = [M, Y];
% disp(M)
% disp(N)
% 

% 
% 
% % Solve the system of linear equations for X
% solution = simplify(subs(linsolve(M, Y)));
% disp(solution)


% Consider tube and hollow channel as parallel beams (same formulation as
% parallel springs9
M = [1/(Ec*Ac) + Es*As + Eh*Ah, 1/(Ec*Ac) + Es*As + Eh*Ah;
    Rc/(Ec*Ic) + 2*Rs*(Es*Is+Eh*Ih), -Rc/(Ec*Ic) - 2*Rh*(Es*Is+Eh*Ih)];

Y = [Fp*(Es*As+Eh*Ah); Fp*Rs*(Es*Is+Eh*Ih)];

N = [M, Y];
disp('rank(M):')
disp(rank(M))
disp('rank(N):')
disp(rank(N))

solution = simplify(subs(linsolve(M, Y)));
disp(solution)

Ts = solution(1);
Th = solution(2);

% Results
epsilon = (Ts+Th)/(Ec*Ac);
epsilon = simplify(subs(epsilon));
vpa(epsilon)

kc = Rc*(Ts-Th)/(Ec*Ac);
kc = simplify(subs(kc));
vpa(kc)

% Subsitute data 
Fp = Fp*1e6;

L = 5;
k_0 = 0;
Rc = 0.4;
Rs = 0.15;
Rh = 0.15;

Ac = pi*Rc^2;
As = pi*Rs^2;
Ah = pi*Rh^2;

Ec = 1e-6*0.2; % Find
Es = 1e-6*0.05; % 0.001 - 0.05 GPa
Eh = 1e-6*0.1; % Find

Ic = pi*(0.8^4 - 0.6^4)/64; % pi*(do^4 - di^4)/64
Is = pi*(0.3^4)/64;
Ih = pi*(0.3^4 - 0.1^4)/64;

epsilon = simplify(subs(epsilon));
L_p = epsilon + L;
vpa(L_p) %0.111067

kc = simplify(subs(kc));
k_p = kc + k_0;
vpa(k_p) % 0.038811

