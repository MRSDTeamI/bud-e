% ini_condition= [0.16320 1.00402 1.42058 -0.29301 0.41901 0.84979 0.12817]';
ini_condition= [0.1420 0 0.2696 0 0 0 1]';

%% Initialization

des_position =ini_condition(1:3);
des_q_r = ini_condition(7);
des_q_i = ini_condition(4:6);

% initial pose
theta =[0.1 0.2 0.3 0.4 0.5 0.6]';
iteration_num =1000;
time_step =0.01;

err = zeros(1000,1);
% trajectory_record =zeros(1000,4);
% tip_position_record = zeros(1000,3);
% p1_position_record = zeros(1000,3);
% p2_position_record = zeros(1000,3);
% p3_position_record = zeros(1000,3);
% p4_position_record = zeros(1000,3);

for i =1:iteration_num
   
%    trajectory_record(i,:) = theta;
   % Use FK to calculate current pose
   [tip]= Arm_Forward_jw(theta);   
   
   %% Save position of each joint for simulation
   
%    tmp_pos1 = p1(1:3,4);
%    p1_position_record(i,:)= tmp_pos1';
%    
%    tmp_pos2 = p2(1:3,4);
%    p2_position_record(i,:)= tmp_pos2';
%    
%    tmp_pos3 = p3(1:3,4);
%    p3_position_record(i,:)= tmp_pos3';
%    
%    tmp_pos4 = p4(1:3,4);
%    p4_position_record(i,:)= tmp_pos4'; 
   
   %%  End effector pose
   
   % retrieve pos and orientation respectively
   % transform rotation matrix to quaternion
   tmp_pos = tip(1:3,4);
%    tip_position_record(i,:)= tmp_pos';  
   tmp_orientation = tip(1:3, 1:3);
   tmp_quaternion = RotationM2Q(tmp_orientation);
   
   tmp_q_r = tmp_quaternion(4);
   tmp_q_i = tmp_quaternion(1:3);
   inv_tmp_q_i = -tmp_q_i;
   
   % Original wrong version
%    err_pos = des_position-tmp_pos;
%    err_ori = quatmultiply([des_q_r, des_q_i'], [tmp_q_r, inv_tmp_q_i']);
   
   % Updated version
   % Calculate error
   err_ori = quatmultiply([des_q_r, des_q_i'], [tmp_q_r, inv_tmp_q_i']);
   mytmp = cross(err_ori(2:4)',tmp_pos)+des_position;
   err_pos = -cross(err_ori(2:4)',tmp_pos)+des_position-tmp_pos;
   
   % the error matrix of each moment
   e_matrix =[err_pos; err_ori(2:4)'];
   
   % calculate jacobian
   tmp_jacobian = Jacobian_gen_jw(theta);
   
   % calculate the pose of next frame
   inv_jac = inv(tmp_jacobian*tmp_jacobian')
   d_theta = tmp_jacobian'*inv(tmp_jacobian*tmp_jacobian') * [err_pos; err_ori(2:4)'];
   
   theta = theta + d_theta .* time_step;
   
   err(i,1) = sqrt(sum(e_matrix.^2)); 
end

theta

%%Draw for simulation

for i = 1:1000
    
    xlabel('X axis');
    ylabel('Y axis');
    zlabel('Z axis');
    
    scatter3(des_position(1), des_position(2), des_position(3),500,'k','.');
    text(des_position(1)+0.1, des_position(2)+0.1, des_position(3)+0.1,'End Pose');
    
    scatter3(tip_position_record(1,1), tip_position_record(1,2), tip_position_record(1,3),500,'r','.');
    text(tip_position_record(1,1)+0.1, tip_position_record(1,2)+0.1, tip_position_record(1,3)+0.1,'Start Pose');
    
   
    grid on;
    x1=plot3([p1_position_record(i,1) p2_position_record(i,1)],[p1_position_record(i,2) p2_position_record(i,2)],[p1_position_record(i,3) p2_position_record(i,3)],'LineWidth',2);
    hold on; 
    x2=plot3([p2_position_record(i,1) p3_position_record(i,1)],[p2_position_record(i,2) p3_position_record(i,2)],[p2_position_record(i,3) p3_position_record(i,3)],'LineWidth',2);
    hold on; 
    x3=plot3([p3_position_record(i,1) p4_position_record(i,1)],[p3_position_record(i,2) p4_position_record(i,2)],[p3_position_record(i,3) p4_position_record(i,3)],'LineWidth',2);
    hold on; 
    x4=plot3([p4_position_record(i,1) p5_position_record(i,1)],[p4_position_record(i,2) p5_position_record(i,2)],[p4_position_record(i,3) p5_position_record(i,3)],'LineWidth',2);
    hold on; 
    x5=plot3([p5_position_record(i,1) p6_position_record(i,1)],[p5_position_record(i,2) p6_position_record(i,2)],[p5_position_record(i,3) p6_position_record(i,3)],'LineWidth',2);
    hold on; 
    x6=plot3([p6_position_record(i,1) tip_position_record(i,1)],[p6_position_record(i,2) tip_position_record(i,2)],[p6_position_record(i,3) tip_position_record(i,3)],'LineWidth',2);
    axis([0.5 1.5 0.5 2 0.5 2]);
    drawnow;
        
    if i~= 1000
        delete(x1);delete(x2);delete(x3);delete(x4);delete(x5);delete(x6);
    end
    
    disp(i);
end

