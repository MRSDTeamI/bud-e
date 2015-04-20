function [ arm_jacobian ] = Jacobian_gen( joint_theta )

tip = [0 0 0.3]';

%Set the parameters
w1 = [ 0 0 1]'; q1 = [0.61 0.72 0    ]' ; c1 = [cross(-w1,q1);w1];
w2 = [-1 0 0]'; q2 = [0    0.72 1.346]' ; c2 = [cross(-w2,q2);w2];
w3 = [ 0 0 1]'; q3 = [0.61 0.72 0    ]' ; c3 = [cross(-w3,q3);w3];

I = eye(3);
T = eye(4);
c = [c1 c2 c3];
w = [w1 w2 w3];
arm_jacobian = [c1 c2 c3];

for i= 2:3
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

