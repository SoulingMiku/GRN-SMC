function move = trapping(p_captor,p_fish,i,pattern,pattern_point,bp_d)
%% ��ʼ��

global  distent_detect;
global  distent_fish;                        %��Ŀ��İ�ȫ����
global  distent_capter;                      %�����������˵İ�ȫ����
global  v2 ;                                  %�������˶��ٶ�
flag = 0;                                %ϵ������֤�������ٶ�һ��
move(1)=0;                               %��ʱ�̺������ƶ��ľ���
move(2)=0;                               %��ʱ���������ƶ��ľ���
factor=[1 1 1 1 1 1];                %Ӱ�����ӡ�Զ��Ŀ�꣬Զ��ͬ�࣬����Ŀ�꣬����g3����,ȥ�ܶȵ͵ķ���,�����˽����Χ�����١�
[num1,dim] = size(p_fish);
[num2,dim] = size(p_captor);
xaxis = fix(p_captor(i,1)/0.25)+1;   %��ǰ����
yaxis = fix(p_captor(i,2)/0.25)+1;
%% �ڰ�ȫ��������:����Զ��
for j=1:num1
    if(((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5 <  distent_fish )
        move(1) = move(1) + factor(1)*(p_captor(i,1)-p_fish(j,1))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
        move(2) = move(2) + factor(1)*(p_captor(i,2)-p_fish(j,2))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
        flag = flag + 1;
    end
end
for j=1:num2
    if(j~=i)                            %���ж��Լ�
        if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_capter)
            move(1) = move(1) + factor(2)*(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
            move(2) = move(2) + factor(2)*(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
            flag = flag + 1;
        end
    end
end
if flag~=0
    move(1)=move(1)/flag/2;             %���������ƶ��ľ���
    move(2)=move(2)/flag/2;
end
%% ��Ŀ��ǰ��
nearfish = near(p_fish, p_captor(i,1), p_captor(i,2));      %�������Ŀ��
if(flag==0)         %���Ȳ����ڰ�ȫ��Χ�ڣ�Ȼ������ƶ�
    if(xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101)
        move(1) = move(1) - factor(3)*(p_captor(i,1)-p_fish(nearfish,1))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
        move(2) = move(2) - factor(3)*(p_captor(i,2)-p_fish(nearfish,2))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
        flag = flag + factor(3);
        
    end
    
    %% ��̽�ⷶΧ��patternŨ�ȸ��ҽ��ĵ�ǰ��
    
    if (xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101 ) % && pattern(xaxis,yaxis)>0.5
        
        %         [bigg3(:,1),bigg3(:,2)] = find( pattern < 0.563 & pattern > 0.560);
        %         bigg3(:,1) = bigg3(:,1)*0.25;
        %         bigg3(:,2) = bigg3(:,2)*0.25;
        bigg3 = pattern_point;
        [num,~] = size(bigg3);
        tem_point = bigg3(round(linspace(1, num, num2+1)),:);
        tem_point(end,:) = [];
        move(1) = move(1) - factor(4)*(p_captor(i,1)-tem_point(i,1))/((p_captor(i,1)-tem_point(i,1))^2+(p_captor(i,2)-tem_point(i,2))^2)^0.5;
        move(2) = move(2) - factor(4)*(p_captor(i,2)-tem_point(i,2))/((p_captor(i,1)-tem_point(i,1))^2+(p_captor(i,2)-tem_point(i,2))^2)^0.5;

        
%         nearpattern = near(bigg3, p_captor(i,1), p_captor(i,2));            %�������g3>0.5
%         move(1) = move(1) - factor(4)*(p_captor(i,1)-bigg3(nearpattern,1))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
%         move(2) = move(2) - factor(4)*(p_captor(i,2)-bigg3(nearpattern,2))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
        flag = flag + factor(4);
        
    end
    %% ���ܶȵ͵ķ���ǰ��
    flagdensity = 0;
    density(1)=0;
    density(2)=0;
    for j=1:num2
        if(j~=i)                            %���ж��Լ�
            if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_detect)
                density(1)=density(1)+(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                density(2)=density(2)+(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                flagdensity = flagdensity+1;
            end
        end
    end
    if flagdensity~=0
        density(1)=factor(5)*density(1)/flagdensity;
        density(2)=factor(5)*density(2)/flagdensity;
        flag = flag+factor(5);
    end
    
    move(1) = move(1) + density(1);
    move(2) = move(2) + density(2);
    
    if flag~=0
        move(1)=move(1)/flag;             %���������ƶ��ľ���
        move(2)=move(2)/flag;
    end
end

if(xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101)        %pattern�ڵ��ƶ��ٶ�
    if pattern(xaxis,yaxis)>0.2
        v2 = v2 * factor(6);
    end
end

theta = atand(move(2)/move(1));
if (move(1)<0 )
    theta = theta +180;
end
move(1) = v2 * cosd(theta);
move(2) = v2 * sind(theta);
end