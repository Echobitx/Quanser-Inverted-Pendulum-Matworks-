%% Symbolic Variables
% Generalized Co-ordinates
syms th dth ddth alph dalph ddalph
q = [th; alph];
dq = [dth; dalph];
ddq = [ddth; ddalph];


% Pendulum-Arm Parameters
syms mr mp Lr Lp Jrr Jrl Jpl Jpr br bp
b = [br; bp];

% Motor Parameters
syms Rm kt km Lm Jm Jh torque
t = [torque; 0];

% constants
syms g

%% Linearized Matxrices
Ml = [ Jh + Jm + Jpl + Jrl + (Lp^2*mp)/4 + Lr^2*mp - Jpl*1^2 + Jpr*1^2 - (Lp^2*mp*1^2)/4, -(Lp*Lr*mp*1)/2;-(Lp*Lr*mp*1)/2, (mp*Lp^2)/4 + Jpl + Jrr];% CODE HERE

Gl = [0; -(Lp*g*mp*1*(alph))/2]% CODE HERE

Cl1 = [0]% CODE HERE
Cl2=subs(Cl1)
Cl=double(Cl2)
%% Equations of Motion
EOM = simplify(Ml\([torque; 0] - [br*dth; bp*dalph] - Cl - Gl))

%% can use the det() funciton
DetM = det(Ml)% CODE HERE

%% A quick explanation behind I get detM here. So as you can see above we are u1*g the \ operator (line 27). 
% This is MATLAB Shorthand for multiplying by the inverse of Ml. If you recall first year math the inverse is defined as the Adjoint matrix
% divided by the determinate. So we multiply by the determinate below to cancel out that division to make the coefficients more readable
% We will divide by the determinate again later. You can ignore this if you want to but it made it much easier for me to read and figure
% out what needed to go where. 

%% EOM (with terms collected according the array below)
EOM1 = collect(EOM(1)*DetM,[torque, alph, dth, dalph]) 
EOM2 = collect(EOM(2)*DetM,[torque, alph, dth, dalph])
size(EOM1)

% Coefficients (extract the coeffcients collected above) 
Coeffs1 = simplify(coeffs(EOM1,[torque, alph, dth, dalph]))
Coeffs2 = simplify(coeffs(EOM2,[torque, alph, dth, dalph]))
size(Coeffs1)
%% State Space
X = [th,alph,dth,dalph]; % these X and dX vectors are for reference as to how I've constructed my State Space model in the reference script.
dX = [dth,dalph,ddth,ddalph];

% put the correct coefficients into the correct elements of the A matrix. I have done B for you as an example 
% You need to look at the Equations of motion for this (EOM1 etc) and construct a SS model accordingly.
A = sym(zeros(4,4))
B = sym(zeros(4,1))
size(Coeffs1)
size(Coeffs2)

B(3,1) = Coeffs1(4)
B(4,1) = Coeffs2(4)


A(1,3) = 1%Coeffs1(0)% Code Here ;
A(2,4) = 1%Coeffs1(0)% Code Here ;
A(3,2) = Coeffs1(3)% Code Here ;
A(3,3) = Coeffs1(2)% Code Here ;
A(3,4) = Coeffs1(1)% Code Here ;
A(4,2) = Coeffs2(3)% Code Here ;
A(4,3) = Coeffs2(2)% Code Here ;
A(4,4) = Coeffs2(1)% Code Here ;

B(3:4) = B(3:4)/DetM;
A(3:4,:) = A(3:4,:)/DetM;

C = 0% Code Here ; % Pay careful attention to your C matrix size (your output y is alpha and theta and your X vector is as above). 
D = 0;

%% Set numerical values for SS Model
br = 2e-3;
bp = 2e-4;
Lr = 0.085;
Lp = 0.129;
mp = 0.095;
mr = 0.024;
Jpl = (1/3)*mp*(Lp^2);
Jpr = (1/2)*mp*(0.005^2);
Jrl = (1/3)*mr*(Lr^2);
Jrr = (1/2)*mr*(0.0035^2);
Jh = 6e-7;
Jm = 4e-6;
g =9.81;

A1 =subs(A)
A = double(A1);
B1 =subs(B);
B = double(B1);
C1 =subs(C);
C = double(C1);
D = double(subs(D));

% At this point I would copy all the code above into a MATLAB file so its easy to run. You will need to play around with pole positions
% and gains to find a suitable controller. 
%% Pole Placement gains
syms k1 k2 k3 k4
k = [k1,k2,k3,k4]
som = A-B*k
sm = charpoly(som)
size(sm)
y0 = sm(1) ==-1
y1 = sm(2) ==-0.5
y2 = sm(3) ==-2
y3 = sm(4) ==-3
y00 = solve(y0,[k1 k2 k3 k4])
y11 = solve(y1,[k3 k4])
y22 = solve(y2,[k1 k2 k3 k4])
y33 = solve(y3,[k1 k2 k3 k4])
P = [2,1,-2,-3]% choose your poles 
Kpp = place(A, B, P)

%% LQR Optimization gains
% Increase Q to make the system prioritize minimizing error or R to make the system prioritize minimizing control action
Q = 200*eye(4); 
R = 6;
Klqr = lqr(A,B,Q,R)
%plot(Klqr)

%% Now make a working controller in the simulink file u1*g gains as you see fit. You will hand in your simulink file under assignments
% So I can test the performance of your controller against the following criteria:
% 1. Does the response settle to alpha = 0 (i.e. does it balance correctly) [1 Mark]
% 2. Does the response settle to theta = ref (i.e. does the theta co-ordinate move to a desired angular position) [1 Mark]
% The above code in the MATLAB Grader is worth 4 marks. 
