function [ arm_jacobian ] = Jacobian_gen( joint_theta )

tip = [0.61 0.72 2]';

%Set the parameters
w1 = [ 0 0 1]'; q1 = [0.61 0.72 0    ]' ; c1 = [cross(-w1,q1);w1];
w2 = [-1 0 0]'; q2 = [0    0.72 1.346]' ; c2 = [cross(-w2,q2);w2];
w3 = [ 0 0 1]'; q3 = [0.61 0.72 0    ]' ; c3 = [cross(-w3,q3);w3];
w4 = [-1 0 0]'; q4 = [0    0.765 1.896]'; c4 = [cross(-w4,q4);w4];
w5 = [ 0 0 1]'; q5 = [0.61 0.72 0    ]' ; c5 = [cross(-w5,q5);w5];
w6 = [-1 0 0]'; q6 = [0    0.72 2.196]' ; c6 = [cross(-w6,q6);w6];
w7 = [ 0 0 1]'; q7 = [0.61 0.72 0    ]' ; c7 = [cross(-w7,q7);w7];

I = eye(3);
T = eye(4);
c = [c1 c2 c3 c4 c5 c6];
w = [w1 w2 w3 w4 w5 w6];
arm_jacobian = [c1 c2 c3 c4 c5 c6];

for i= 2:6
    for j =1:i
        
        calculation = [(expm(hat_operater(w(:,j))* joint_theta(j))) ...
        (I-expm(hat_operater(w(:,j))* joint_theta(j)))*cross(w(:,j),c(1:3,j))+(w(:,j)*w(:,j)'*c(1:3,j)*joint_theta(j));
                     0 0 0 1];
        T = T* calculation;   
    end
    
    tmp_c = adjoint(T)* c(:,i);    
    arm_jacobian(:,i)= tmp_c;
    
    T= eye(4);
end

arm_jacobian

end

