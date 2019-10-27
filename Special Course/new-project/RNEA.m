function [Q,t1,t2] = RNEA(z,R,I,m, alphaT, a, d, re,rc,q, qd,qdd,n)
% [Q,t1,t2,M,CGJF]
g = [0; 0; -9.8]; b = []; w = []; wd = []; alpha = [];
ae = []; ac = []; gload = []; f = []; tau = [];


for i = 1:n
% % Forward
R{i}';
R0 = eye(3);

% z{i};

b{i} = (R{i}')* z(:,i);
if (i == 1)
    w{i} = ((R0)') * zeros(3,1) + b{i} * qd(i); 
    wd{i} = ((R0)') * zeros(3,1) + b{i} * qdd(i); 
    alpha{i} = (R0') * zeros(3,1) + b{i}*qdd(i)  + cross(w{i},(b{i}*qd(i) ));
    
    ac{i} = cross(wd{i},rc{i}) + cross( w{i} ,cross(w{i},rc{i}));
    ae{i} = cross(wd{i},re{i}) + cross( w{i} ,cross(w{i},re{i}));

else
    w{i} = (inv(R{i-1})') * w{(i-1)} + b{i} * qd(i);
    wd{i} = (inv(R{i-1})') * wd{(i-1)} + b{i} * qdd(i);
    alpha{i} = (R{i-1}') * alpha{(i-1)} + b{i}*qdd(i)  + cross(w{i},(b{i}*qd(i) ));
    
    ae{i} = (inv(R{i})') * ae{(i-1)} + cross(wd{i},re{i}) +  cross( w{i} ,cross(w{i},re{i}));
    ac{i} = (inv(R{i})') * ac{(i-1)} + cross(wd{i},rc{i}) +  cross( w{i} ,cross(w{i},rc{i}));
end
gload{i} = -R{i}*g;
end
f = zeros(3,2)
tau = zeros(3,2)
% % Backward
for i=n:-1:1
%     j = n-i
    if (i == n);
        T = calc_transformation(i-1, i, alphaT, a, q, d);
        R2 = T(1:3,1:3);
        f3 = zeros(3,1);
        tau3 = zeros(3,1);
        R03 = zeros(3,3);
        rc3 = zeros(3,1);
        
        f(:,i) =  double(m(i)*ac{i} - m(i)*gload{i});
        tau(:,i) =  R03*tau3 - cross(f(:,i),rc{i}) +  cross((R03*f3) ,rc3) + alpha{i} + cross( w{i}, (I{i}*w{i}) )
    else
       
%         j = ((n+1)-i);        
        f(:,i) = double(R2*f(:,i+1) + m(i)*ac{i} - m(i)*gload{i});
        tau(:,i) =  R2*tau(:,i+1) - cross(f(:,i),rc{i}) +  cross((R2*f(:,i+1)) ,rc{i+1}) + alpha{i} + cross( w{i}, (I{i}*w{i}) )
    end
end

t1 = tau(:,1);
t2 = tau(:,2);

Q = [];
for i = 1:n
    Q = [Q; (tau(:,i))];    
end
% Q = Q(1:2,1:2);

% Q = fliplr(Q(1:2,:));

% 
% 
% % Calculating M
% for i = 1:n
% % % Forward
% R{i}'
% z(:,i)
% j = (i-1);
% qddM = qdd;
% qdM = [0;0];
% 
% b{i} = (R{i}')* (z(:,i));
% if (i == 1)
%     w{i} = (inv(R{i})') * zeros(3,1) + b{i} * qdM(i); 
%     wd{i} = (inv(R{i})') * zeros(3,1) + b{i} * qddM(i); 
%     alpha{i} = (R{i}') * zeros(3,1) + b{i}*qddM(i)  + cross(w{i},(b{i}*qdM(i) ));
%     
%     ac{i} = cross(wd{i},rc{i}) + cross( w{i} ,cross(w{i},rc{i}));
%     ae{i} = cross(wd{i},re{i}) + cross( w{i} ,cross(w{i},re{i}));
% 
% else
%     if (j ~= i)
% %        display('qddM res:');
%        qddM(j) = 0;
%     end 
%     w{i} = (inv(R{i})') * w{(i-1)} + b{i} * qdM(i);
%     wd{i} = (inv(R{i})') * w{(i-1)} + b{i} * qddM(i);
%     alpha{i} = (R{i}') * alpha{(i-1)} + b{i}*qddM(i)  + cross(w{i},(b{i}*qdM(i) ));
%     
%     ae{i} = (inv(R{i})') * ae{(i-1)} + cross(wd{i},re{i}) +  cross( w{i} ,cross(w{i},re{i}));
%     ac{i} = (inv(R{i})') * ac{(i-1)} + cross(wd{i},rc{i}) +  cross( w{i} ,cross(w{i},rc{i}));
% end
% gload{i} = -R{i}*zeros(3,1);
% end
% 
% % % Backward
% for i=n:-1:1
% %     j = n-i
%     if (i == n);
%         f3 = zeros(3,1);
%         tau3 = zeros(3,1);
%         R03 = zeros(3,3);
%         rc3 = zeros(3,1);
%         
%         f{i} =  m(i)*ac{i} - m(i)*gload{i};
%         tau{i} =  R03*tau3 - cross(f{i},rc{i}) +  cross((R03*f3) ,rc3) + alpha{i} + cross( w{i}, (I{i}*w{i}) );
%     else
% %         j = ((n+1)-i);        
%         f{i} = R{i+1}*f{(i+1)} + m(i)*ac{i} - m(i)*gload{i};
%         tau{i} =  R{i+1}*tau{(i+1)} - cross(f{i},rc{i}) +  cross((R{i+1}*f{(i+1)}) ,rc{i+1}) + alpha{i} + cross( w{i}, (I{i}*w{i}) );
%     end
% end
% 
% M = [];
% for i = 1:n
%     M = [M (tau{i})];    
% end
% M = M(1:2,1:2);
% 
% 
% 
% % Compute CGJF
% 
% 
% qdd0 = zeros(1,length(qdd));
% for i = 1:n
% % % Forward
% R{i}';
% z(:,i);
% 
% b{i} = (R{i}')* (z(:,i));
% if (i == 1)
%     w{i} = (inv(R{i})') * zeros(3,1) + b{i} * qd(i); 
%     wd{i} = (inv(R{i})') * zeros(3,1) + b{i} * qdd0(i); 
%     alpha{i} = (R{i}') * zeros(3,1) + b{i}*qdd0(i)  + cross(w{i},(b{i}*qd(i) ));
%     
%     ac{i} = cross(wd{i},rc{i}) + cross( w{i} ,cross(w{i},rc{i}));
%     ae{i} = cross(wd{i},re{i}) + cross( w{i} ,cross(w{i},re{i}));
% 
% else
%     w{i} = (inv(R{i})') * w{(i-1)} + b{i} * qd(i);
%     wd{i} = (inv(R{i})') * w{(i-1)} + b{i} * qdd0(i);
%     alpha{i} = (R{i}') * alpha{(i-1)} + b{i}*qdd0(i)  + cross(w{i},(b{i}*qd(i) ));
%     
%     ae{i} = (inv(R{i})') * ae{(i-1)} + cross(wd{i},re{i}) +  cross( w{i} ,cross(w{i},re{i}));
%     ac{i} = (inv(R{i})') * ac{(i-1)} + cross(wd{i},rc{i}) +  cross( w{i} ,cross(w{i},rc{i}));
% end
% gload{i} = -R{i}*g;
% end
% 
% % % Backward
% for i=n:-1:1
% %     j = n-i
%     if (i == n);
%         f3 = zeros(3,1);
%         tau3 = zeros(3,1);
%         R03 = zeros(3,3);
%         rc3 = zeros(3,1);
%         
%         f{i} =  m(i)*ac{i} - m(i)*gload{i};
%         tau{i} =  R03*tau3 - cross(f{i},rc{i}) +  cross((R03*f3) ,rc3) + alpha{i} + cross( w{i}, (I{i}*w{i}) );
%     else
% %         j = ((n+1)-i);        
%         f{i} = R{i+1}*f{(i+1)} + m(i)*ac{i} - m(i)*gload{i};
%         tau{i} =  R{i+1}*tau{(i+1)} - cross(f{i},rc{i}) +  cross((R{i+1}*f{(i+1)}) ,rc{i+1}) + alpha{i} + cross( w{i}, (I{i}*w{i}) );
%     end
% end
% CGJF = [];
% for i = 1:n
%     CGJF = [CGJF (tau{i})];    
% end
% CGJF = CGJF(1:2,1:2);





% % % % % % % % % % % % % For Testing
% tau = fliplr(tau)
% Q = []
% for i = 1:n
%     disp('b vec')
%     b{i}
%     disp('w vec')
%     w{i} 
%     disp('wd vec')
%     wd{i}
%     disp('alpha vec')
%     alpha{i}
%     disp('ae vec')
%     simplify(ae{i})
%     disp('ac vec')
%     simplify(ac{i})
%     disp('gload vec')
%     gload{i}
%     disp('f vec')
%     f{i}
%     disp('tau vec')
%     test = tau{i}
%     test(1:2,:)
%     Q = [Q (tau{i})];    
% end

% Q = Q(1:2,:)

% f2 = R03*f3 + m2*ac2 - m2*g2
% tau2 =  R03*tau3 - cross(f2,r2c2) +  cross((R03*f3) ,r2c2) + alpha2 + cross( w2, (I2*w2) )

% f1 = R02*f2 + m1*ac1 - m1*g1
% tau1 =  R02*tau2 - cross(f1,r1c1) +  cross((R02*f2) ,r1c1) + alpha1 + cross( w1, (I1*w1) )




end