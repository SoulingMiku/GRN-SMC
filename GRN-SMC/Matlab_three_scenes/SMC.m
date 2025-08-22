%»¬Ä¤¿ØÖÆÆ÷
clc
clear all;
close all;

T=10;
t1=0.1;
i=1:T/t1;
pi=3.1419526;

l1=0.32;
l2=0.30;
l3=0.32;
l4=0.30;
r1=0.08;
r=0.08;
% 
% w1follow=zeros(1,length(i));
% w2follow=zeros(1,length(i));
% w3follow=zeros(1,length(i));
% w4follow=zeros(1,length(i));
% 
% 
% w1leader=zeros(1,length(i));
% w2leader=zeros(1,length(i));
% w3leader=zeros(1,length(i));
% w4leader=zeros(1,length(i));

X_f=zeros(1,length(i));
Y_f=zeros(1,length(i));
theta_f=zeros(1,length(i));
X_f(:,1)=5;             
Y_f(:,1)=3;   
theta_f(:,1)=pi; 

X_d=zeros(1,length(i));
Y_d=zeros(1,length(i));
theta_d=zeros(1,length(i));
X_d(:,1)=4;                   
Y_d(:,1)=4;
theta_d(:,1)=pi;

a=1;
b=0.4;
K1=[a 0 0;
    0 a 0;
    0 0 a];
K2=[b 0 0;
    0 b 0;
    0 0 b];

dde = [0;0;0];
e=zeros(3,length(i));
ie = zeros(3,length(i));
epsilon = zeros(3,length(i));
S=zeros(3,length(i));
V=zeros(3,length(i));
sgnS=zeros(3,length(i));
 
 for  k=1:length(i)
     
R1=r/4*[1 1 1 1;
       -1 1 1 -1;
       -1/(l1+l2) 1/(l1+l2) -1/(l1+l2) 1/(l1+l2)];
   
% R2=1/r*[1 -1 -(l1+l2);
%         1 1 (l1+l2);
%         1 1 -(l1+l2);
%         1 -1 (l1+l2)];
    
R3=[cos(theta_f(:,k)) -sin(theta_f(:,k)) 0;
    sin(theta_f(:,k)) cos(theta_f(:,k)) 0;
    0 0 1];

dot_xd=[1;1;0];
e(:,k) = [X_d(:,k)-X_f(:,k);
          Y_d(:,k)-Y_f(:,k);
          theta_d(:,k)-theta_f(:,k)];


      
if k == 1
    S(:,k) = K1*e(:,k) + K2*e(:,1);
else
    ie(1,k) = ie(1,k-1)+e(1,k)*t1;
    ie(2,k) = ie(2,k-1)+e(2,k)*t1;
    ie(3,k) = ie(3,k-1)+e(3,k)*t1;
    S(:,k) = K1*e(:,k) + K2*ie(:,k) + 0.002*(e(:,k)-e(:,k-1))/t1;    
end

for i=1:3
    if (S(i,k)>1) 
        sgnS(i,k)=1;
    elseif (S(i,k)<-1) 
        sgnS(i,k)=-1;
    else 
        sgnS(i,k)=S(i,k);
    end
end
sgnS(:,k)=[sgnS(1,k);sgnS(2,k);sgnS(3,k)];

depsilon1 = 1.5*S(1,k)*abs(S(1,k))^0.8*sgnS(1,k);
depsilon2 = 4.5*S(2,k)*abs(S(2,k))^0.8*sgnS(2,k);
depsilon3 = 1.5*S(3,k)*abs(S(3,k))^0.8*sgnS(3,k);
if k > 1
epsilon(1,k) = epsilon(1,k-1)+depsilon1*t1;
epsilon(2,k) = epsilon(2,k-1)+depsilon2*t1;
epsilon(3,k) = epsilon(3,k-1)+depsilon3*t1;
end

if k > 2
    dde = ((e(:,k)-e(:,k-1))/t1-(e(:,k-1)-e(:,k-2))/t1)/t1;
end

    
% Sgn=[sign(S(1,k));
%      sign(S(2,k));
%      sign(S(3,k))];
% V(:,k)=inv(R3)*(dot_xd-K1*Sgn-K2*S(:,k));


%V(:,k)=inv(R3)*(dot_xd-K1*sgnS(:,k)-K2*S(:,k));
V(1,k) = (1/a)*(a*X_d(1,k)+b*e(1,k)+0.02*dde(1)+epsilon(1,k)*abs(S(1,k))^0.8*sign(S(1,k))+0.1*abs(S(1,k))^0.065*S(1,k));
V(2,k) = (1/a)*(a*Y_d(1,k)+b*e(2,k)+0.02*dde(2)+epsilon(2,k)*abs(S(2,k))^0.8*sign(S(2,k))+0.1*abs(S(2,k))^0.065*S(2,k));
V(3,k) = (1/a)*(a*theta_d(1,k)+b*e(3,k)+0.02*dde(3)+epsilon(3,k)*abs(S(3,k))^0.8*sign(S(3,k))+0.1*abs(S(3,k))^0.065*S(3,k));



theta_d(:,k+1)=theta_d(:,k)+t1*0;
X_d(:,k+1)=X_d(:,k)+t1*1;
Y_d(:,k+1)=Y_d(:,k)+t1*1;

theta_f(:,k+1)=theta_f(:,k)+t1*V(3,k);
X_f(:,k+1)=X_f(:,k)+t1*V(1,k);
Y_f(:,k+1)=Y_f(:,k)+t1*V(2,k);   
 end
 
figure(1);
plot(X_f,Y_f,'r-','LineWidth',2);
hold on
plot(X_d,Y_d,'b--','LineWidth',2);
hold on
plot(X_f(:,1),Y_f(:,1),'rd','LineWidth',2);
hold on
plot(X_d(:,1),Y_d(:,1),'go','LineWidth',2);
%       xlim([0,15]);
%       ylim([-5,5]);
xlabel('x(m)','FontSize',18);
ylabel('y(m)','FontSize',18);
% legend('trajectory of R','trajectory of Rr','FontSize',18);
 
