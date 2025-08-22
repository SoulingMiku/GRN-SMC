function m = calc_mor(pos,i,j)

global cell_wid;
[num,dim] = size(pos); %% num: number of organizing agents in the env; dim: the dimension of the system, typically, 2.Can be generalized
                       %% the last element of pos reprsents the strength of
                       %% the indexed source
epi = 1;    %���Ǹ���ģ�û��������Ĳ�����
m = 0;
temp(1:3) = 0;

for index = 1:num                                       %������Ŀ�꣺���Բ�û�д�������Χ��
    r = ((cell_wid*(i-0.5)-pos(index,1))^2+(cell_wid*(j-0.5)-pos(index,2))^2)^0.5;  %����Ŀ��ľ���
%     m = m + pos(index,3)*exp(-r/epi);                   %���ž����Զ��Ӱ���𽥱�С
%     m = m + 1/(30*r);                   %���ž����Զ��Ӱ���𽥱�С

%     m = m + exp(- r / epi); 
%     r = r * 3;
%     temp(1) = 2 * exp(-r / epi);
%     temp(2) = 1 / (1 + exp(-r)) - 0.1;
%     temp(3) = 2 * (1 - 1/(1+exp(-(r - 3))));
%     
%     m = m + temp(1) + temp(2) + temp(3);

    m = m +1*exp(-r/epi); 

%      m = m + pos(index,3)*exp(-r/epi);    
end

m=m;