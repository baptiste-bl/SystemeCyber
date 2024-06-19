taux = 0.016;
k = 52.98;

a = -1/taux;
b = k/taux;

A = [0 1; 0 a];
B = [0; b];
C = [1 0];

Ac = [a 0;-1 0];
Bc=[b; 0];
vpa = eig(A);

desired_poles = [vpa(2,1), 1.5*vpa(2,1)];

K = place(Ac, Bc, desired_poles);
kX = K(1);
kI = K(2);
Kb = 100;

AO = [0 1;0 a];
BO = [0; b];
CO = [1 0];
vlpc = [2*vpa(2,1), 3*vpa(2,1)];
tansAO = transpose(AO);
tansCO = transpose(CO);
Ltrans = place(tansAO, tansCO, vlpc);
L = transpose(Ltrans);

AObis = [0 1 0;0 a b;0 0 0];
BObis = [0; b; 0];
CObis = [1 0 0];
vlpcbis = [1.5*vpa(2,1), 1.6*vpa(2,1),1.7*vpa(2,1)];
tansAObis = transpose(AObis);
tansCObis = transpose(CObis);
Ltransbis = place(tansAObis, tansCObis, vlpcbis);
Lbis = transpose(Ltransbis);

Kt=place([0 0;0 0],[1 0;0 1],[-1/5,-1/5]);

