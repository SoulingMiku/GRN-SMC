function move = CHtrapping(p_captor,p_fish,i,pattern,pattern_barrier,factors,bp_d)
  %% ��ʼ��

global  distent_detect;
global  distent_fish;                        %��Ŀ��İ�ȫ����
global  distent_capter;                      %�����������˵İ�ȫ����
global  v1 ;                                  %�������˶��ٶ�
    flag1 = 0;                               %ϵ�����Ƿ�����������
    flag2 = 0;
    
    acceleration=zeros(7,2);                 %Ӱ�����ӡ�Զ��Ŀ�꣬Զ��ͬ�࣬����Ŀ�꣬����g3����,ȥ�ܶȵ͵ķ��򣬱��ϡ�
    factor=factors(1,:);                     %Ĭ��ʹ�ù���1
    [num1,dim] = size(p_fish); 
    [num2,dim] = size(p_captor); 
        xaxis = fix(p_captor(i,1)/0.25)+1;   %��ǰ����
        yaxis = fix(p_captor(i,2)/0.25)+1;
        


%% ��Ŀ�갲ȫ��������:����Զ��
    for j=1:num1
        if(((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5 <  distent_fish )
            acceleration(1,1) = acceleration(1,1) + (p_captor(i,1)-p_fish(j,1))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
            acceleration(1,2) = acceleration(1,2) + (p_captor(i,2)-p_fish(j,2))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;  
            flag1 = flag1 + 1;
        end
    end
        if flag1~=0                                              %�������:���ٶȹ�һ
                acceleration(1,1) = acceleration(1,1)/flag1/2;
                acceleration(1,2) = acceleration(1,2)/flag1/2;
                factor=factors(2,:);                             %ʹ�ù���2
        end
    
%% ����Χ�����˰�ȫ��������:����Զ��     
    for j=1:num2
        if(j~=i)                            %���ж��Լ�
            if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_capter)
                acceleration(2,1) = acceleration(2,1) + (p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                acceleration(2,2) = acceleration(2,2) + (p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                flag2 = flag2 + 1;
            end
        end
    end
        if flag2~=0                                              %�������
            acceleration(2,1) = acceleration(2,1)/flag2/2;
            acceleration(2,2) = acceleration(2,2)/flag2/2;
            factor=factors(2,:);                             %ʹ�ù���2
        end
        
%% ��Ŀ��ǰ�� 
    nearfish = near(p_fish, p_captor(i,1), p_captor(i,2));      %�������Ŀ��
   % if(flag==0)         %���Ȳ����ڰ�ȫ��Χ�ڣ�Ȼ������ƶ�
        if(xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101)
            acceleration(3,1) = acceleration(3,1) - (p_captor(i,1)-p_fish(nearfish,1))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
            acceleration(3,2) = acceleration(3,2) - (p_captor(i,2)-p_fish(nearfish,2))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
           % flag = flag + factor(3);
        else
            acceleration(3,1) = 0;
            acceleration(3,2) = 0;
        end

%% ��̽�ⷶΧ��patternŨ�ȸ��ҽ��ĵ�ǰ��

        if (xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101 && pattern(xaxis,yaxis)<0.2)
            
            [bigg3(:,1),bigg3(:,2)] = find(pattern>0.5);
            bigg3(:,1) = bigg3(:,1)*0.25;
            bigg3(:,2) = bigg3(:,2)*0.25;
            nearpattern = near(bigg3, p_captor(i,1), p_captor(i,2));            %�������g3>0.5
            acceleration(4,1) = acceleration(4,1) - (p_captor(i,1)-bigg3(nearpattern,1))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
            acceleration(4,2) = acceleration(4,2) - (p_captor(i,2)-bigg3(nearpattern,2))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
           % flag = flag + factor(4);
 
        end

    %% ���ܶȵ͵ķ���ǰ��
    flag5 = 0;
    density(1)=0;
    density(2)=0;
        for j=1:num2
            if(j~=i)                            %���ж��Լ�
                if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_detect/2)
                    density(1)=density(1)+(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                    density(2)=density(2)+(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                    flag5 = flag5+1;

                end
            end
        end
        if flag5~=0
            density(1)=density(1)/flag5;
            density(2)=density(2)/flag5;
          %  flag = flag+factor(5);
        end

        acceleration(5,1) = acceleration(5,1) + density(1);
        acceleration(5,2) = acceleration(5,2) + density(2);
        

   % end
   
   %% �����ĵ�ǰ��
%    acceleration(6,1) = acceleration(6,1) - (p_captor(i,1)-p_captor(i,4))/((p_captor(i,1)-p_captor(i,4))^2+(p_captor(i,2)-p_captor(i,5))^2)^0.5;
%    acceleration(6,2) = acceleration(6,2) - (p_captor(i,2)-p_captor(i,5))/((p_captor(i,1)-p_captor(i,5))^2+(p_captor(i,2)-p_captor(i,5))^2)^0.5;


    %% ���ݸ����ٶ��������ƶ�����
    move = zeros(1,2);                  %��ÿ���ٶȼ�Ȩ���
    for p=1:6
        move = move + factor(p)*acceleration(p,:);
    end
    
    
%     [a,die] = avoiding_die(p_captor,move,i,pattern_barrier,p_fish);
%     die = 0;
%     acceleration(7,1) = a(1);
%     acceleration(7,2) = a(2);
    
    factor(7) = factor(7)*200;
    move = zeros(1,2);                  %���ϱ��Ϻ󣬸�ÿ���ٶȼ�Ȩ���
    for p=1:7
        move = move + factor(p)*acceleration(p,:);
    end

    
    theta = atand(move(2)/move(1));     %�����շ���
    if (move(1)<0 )
        theta = theta +180;
    end
    move(1) = v1 * cosd(theta);          %�������ٶ�
    move(2) = v1 * sind(theta);
    
%     if pattern(round(p_captor(i,1)/0.25),round(p_captor(i,2)/0.25))>0.2
%         move= closing(p_captor,move,i,pattern,p_fish);
%     end
    

    
 %   if pattern_barrier(round((p_captor(i,1)+move(1))/0.25),round((p_captor(i,2))+move(2)/0.25))<0.4
%         if ((p_captor(i,1)-p_captor(i,6))^2+(p_captor(i,2)-p_captor(i,7))^2)^0.5< v %���������ľ���С���ٶȣ�ֱ��ȥ�����
%             move(1) = p_captor(i,6)-p_captor(i,1);
%             move(2) = p_captor(i,7)-p_captor(i,2);
%         end
  %  end
