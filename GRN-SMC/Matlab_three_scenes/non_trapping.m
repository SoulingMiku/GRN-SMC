function non_move = non_trapping(p_captor,p_fish,i)
  %% ��ʼ��
global  distent_detect;
global  distent_fish;                        %��Ŀ��İ�ȫ����
global  distent_capter;                      %�����������˵İ�ȫ����
    flag = 0;                       %ϵ������֤�������ٶ�һ��
    v=0.5;                          %�ٶ�
    non_move(1)=0;
    non_move(2)=0;
    factor=[1 1 1 0 0.5];                         %Ӱ�����ӡ�Զ��Ŀ�꣬Զ��ͬ�࣬����Ŀ�꣬����g3����,ȥ�ܶȵ͵ķ���
    [num1,dim] = size(p_fish); 
    [num2,dim] = size(p_captor); 
%% �ڰ�ȫ��������:����Զ��
    for j=1:num1
        if(((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5 <  distent_fish )
            non_move(1) = non_move(1) + factor(1)*(p_captor(i,1)-p_fish(j,1))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
            non_move(2) = non_move(2) + factor(1)*(p_captor(i,2)-p_fish(j,2))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;  
            flag = flag + 1;
        end
    end
    for j=1:num2
        if(j~=i)                            %���ж��Լ�
            if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_capter)
                non_move(1) = non_move(1) + factor(2)*(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                non_move(2) = non_move(2) + factor(2)*(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                flag = flag + 1;
            end
        end
    end
    if flag~=0;
        non_move(1)=non_move(1)/flag*v;
        non_move(2)=non_move(2)/flag*v;
    end
    
%% ��̽�ⷶΧ�ڣ����棬�����ھ��ǵķ���ʸ����
if(flag==0)
        for j=1:num2
            if(j~=i && (p_captor(j,3)==2)    )                            %���ж��Լ�������ֻ������֯������
                if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_detect)
                    non_move(1) = non_move(1) + factor(3) * p_captor(j,4);
                    non_move(2) = non_move(2) + factor(3) * p_captor(j,5);
                    flag = flag + 1;
                end
            end
        end
        if flag~=0;
            non_move(1)=non_move(1)/flag;               %���ٶ���ģ�����Ҫ�ٳ��ٶ�
            non_move(2)=non_move(2)/flag;
        end
end