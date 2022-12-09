function testKinematics_Meca

addpath("../")
T_init = eye(4,4);
n_joints = 6;
types = 'rrrrrr';

% T_tool = eye(4,4);

tic
Robot = RobotKinematics(n_joints, types, T_init,[]);
toc

q = zeros(n_joints,1);
DH = ...
    [0.135 0 0 -pi/2;
    0 -pi/2 0.135 0;
    0 0 0.038 -pi/2;
    0.120 pi 0 -pi/2;
    0 0 0 pi/2;
    0.070 0 0 0];

DH_params = reshape(DH',4*n_joints,1);
% 
tic
[Robot,T_val,P] = Robot.getPose(q,DH_params);
[Dp,Dor] = Robot.getDerivs(q,DH_params);
toc

tic
% [Robot,T_val_num,P_num] = Robot.getPoseNum(q,DH_params);
% [Dp_num,Dor_num] = Robot.getDerivsNum(q,DH_params);
% 
% [D,quat] = Robot.getQuatDerivNum(q,DH_params);

[Robot,T_num,P_num,Dp_num,Dor_num] = Robot.getKineDeriv_Ana(q,DH_params);
toc

q_max = [175/180*pi;-pi/2;70/180*pi;170/180*pi;115/180*pi;pi];
q_min = [-175/180*pi;-70/180*pi/2;-135/180*pi;-170/180*pi;-115/180*pi;-pi];

ns = 5; % number of samples

Q = zeros(n_joints,ns);

for i = 1:2

    Q(i,:) = linspace(q_min(i),q_max(i),ns);

end

C = Mycombvec(Q);
[~,nc] = size(C);

for i = 1:nc

q = C(:,i);
tic
% [Robot,T_val,P(:,i)] = Robot.getPose(q,DH_params);
[Robot,T_val,P(:,i)] = Robot.getPoseNum(q,DH_params);
toc

end

P_m = P;
save('tests/P_m_meca','P_m')
Q = C;
save('tests/Q_meca','Q')



end