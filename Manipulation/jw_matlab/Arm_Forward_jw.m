function [ tip_pose ] = Arm_Forward( theta )
%traj_data = load('qdata.txt');

p1 = [0.61 0.72 1.346]';
p2 = [0.61 0.72 1.866]';
p3 = [0.61 0.765 1.896]';
p4 = [0.61 0.72 1.926]';
tip = [0.61 0.72 2]';

%Set the parameters
w1 = [ 0 0 1]'; q1 = [0.61 0.72 0    ]' ; c1 = [cross(-w1,q1);w1];
w2 = [-1 0 0]'; q2 = [0    0.72 1.346]' ; c2 = [cross(-w2,q2);w2];
w3 = [ 0 0 1]'; q3 = [0.61 0.72 0    ]' ; c3 = [cross(-w3,q3);w3];
w4 = [-1 0 0]'; q4 = [0    0.765 1.896]'; c4 = [cross(-w4,q4);w4];
w5 = [ 0 0 1]'; q5 = [0.61 0.72 0    ]' ; c5 = [cross(-w5,q5);w5];

%% tip position calculation

I = eye(3);
gstTip = [I tip; 0 0 0 1];
T = eye(4);
w =[w1 w2 w3 w4 w5];
c =[c1 c2 c3 c4 c5];

for j = 1: 5
    calculation = [(expm(hat_operater(w(:,j))* theta(j))) ...
        (I-expm(hat_operater(w(:,j))* theta(j)))*cross(w(:,j),c(1:3,j))+(w(:,j)*w(:,j)'*c(1:3,j)*theta(j));
                     0 0 0 1];    
    T = T* calculation;
end

tip_pose = T*gstTip

% %% P1, P2 position calculation
% 
% I = eye(3);
% gst1 = [I p1; 0 0 0 1];
% gst2 = [I p2; 0 0 0 1];
% T = eye(4);
% w =[w1 w2 w3];
% c =[c1 c2 c3];
% 
% for j = 1: 3
%     calculation = [(expm(hat_operater(w(:,j))* theta(j))) ...
%         (I-expm(hat_operater(w(:,j))* theta(j)))*cross(w(:,j),c(1:3,j))+(w(:,j)*w(:,j)'*c(1:3,j)*theta(j));
%                      0 0 0 1];
%     T = T* calculation ;          
% end
% p1_pose = T*gst1;
% p2_pose = T*gst2;
% 
% %% P3, P4 position calculation
% 
% I = eye(3);
% gst3 = [I p3; 0 0 0 1];
% gst4 = [I p4; 0 0 0 1];
% T = eye(4);
% w =[w1 w2 w3 w4];
% c =[c1 c2 c3 c4];
% 
% for j = 1: 4
%     calculation = [(expm(hat_operater(w(:,j))* theta(j))) ...
%         (I-expm(hat_operater(w(:,j))* theta(j)))*cross(w(:,j),c(1:3,j))+(w(:,j)*w(:,j)'*c(1:3,j)*theta(j));
%                      0 0 0 1];
%     T = T* calculation ;          
% end
% p3_pose = T*gst3;
% p4_pose = T*gst4;
% 
% %% P5 position calculation
% 
% I = eye(3);
% gst5 = [I p5; 0 0 0 1];
% T = eye(4);
% w =[w1 w2 w3 w4 w5 w6];
% c =[c1 c2 c3 c4 c5 c6];
% 
% for j = 1: 6
%     calculation = [(expm(hat_operater(w(:,j))* theta(j))) ...
%         (I-expm(hat_operater(w(:,j))* theta(j)))*cross(w(:,j),c(1:3,j))+(w(:,j)*w(:,j)'*c(1:3,j)*theta(j));
%                      0 0 0 1];
%     T = T* calculation ;          
% end
% p5_pose = T*gst5;
% 
end