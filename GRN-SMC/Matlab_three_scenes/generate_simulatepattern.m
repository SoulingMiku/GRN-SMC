 function pattern = generate_simulatepattern(p_captor)

global cell_wid;
N_cell_x = 100;
N_cell_y = 100;

p2 = zeros(N_cell_x,N_cell_y);
sita1 = 0.25;
for i = 1:N_cell_x
    for j = 1:N_cell_y
         p2(i,j) = calc_mor(p_captor,i,j);     %ʵ��Ӱ��ľ�ֻ�к�Ŀ��ľ����Ŀ���Ӱ����
%          p2(i,j) = 1/(1+exp(-15*(p(i,j)-sita1)));
    end
end

p2 = p2/max(max(p2(:)));

% [c,h]=contour(x_mat,y_mat,g1);                        %�������꼰�߶Ȼ��ȸ���
% %set(h,'ShowText','on')%��ʾ�ȸ��ߵ�ֵ
% %set(h,'ShowText','on','LevelList',0.3643)%�趨�ȸ��ߵ�ֵΪ0.3643
% set(h,'LevelList',0.4)%�趨�ȸ��ߵ�ֵΪ0.3643
% axis([0,25,0,25]);                              %x��y�ᶼ��25֮�ڣ�������100*100
% hold on;

pattern =  p2;