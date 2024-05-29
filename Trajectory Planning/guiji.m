clear all; clc;

%% 定义常量
l = 0.2; % 车辆的轴距
R = 1; % 参考轨迹的半径
K = 0.1; % 控制增益  
delta_T = 0.5; % 采样时间

%% 初始化位置和预期轨迹
Pos = [0, 0, 0]; % 初始化位置,格式为[X, Y, Theta]
T = linspace(0, 2*pi, 260); % 时间向量

% 预期轨迹位置
% desire_Pos = [16 .* power(sin(T), 3); 13*cos(T) - 5*cos(2.*T) - 2*cos(3.*T) - 2*cos(4.*T)];
% % 更新为八字形轨迹位置
 desire_Pos = [20 .* sin(T); 20 .* sin(T) .* cos(T)];

%% 图形界面设置  
f3 = figure;
xlabel('x (m)')
ylabel('y (m)') 
grid on

%% 设置车辆的尺寸
A.R_w = 1/2; % 车辆宽度的一半
A.R_l = 2/2; % 车辆长度的一半
A.a1 = [-A.R_l -A.R_w]';
A.b1 = [A.R_l -A.R_w/2]';
A.b2 = [A.R_l A.R_w/2]';
A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];

% 根据车辆的当前姿态旋转车辆的顶点
A.Rot = [cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*A.P;

% 将旋转后的顶点加上车辆中心的偏移
A.Prot_trasl = A.Rot + [ones(1, 4)*Pos(1); ones(1, 4)*Pos(2)];

% 绘制车辆
A.P_robot = patch(A.P(1, :), A.P(2, :), 'b');
A.P_robot.XData = A.Prot_trasl(1, :)';
A.P_robot.YData = A.Prot_trasl(2, :)';

%% 绘制图像
h = animatedline('Color', 'r', 'LineStyle', '--');
h_car = animatedline('Color', 'b');
h_car_model = animatedline('Marker', 'o', 'MaximumNumPoints', 1); % 只显示1个新的点
axis([-20 20 -20 15])

%% 初始化GIF文件名
gif_filename = 'tracking.gif';

% 初始化控制器参数
k1 = 0.05; % 控制器增益,需要根据系统性能进行调整
k2 = 0.000001;
k3 = 0.001;
vr = [1; 0.5]; % 参考速度,可能需要根据具体轨迹进行调整
e = [0; 0; 0]; % 初始化误差

for k = 1:length(T)
    
    % 计算期望位置和实际位置的误差
    x_tild = desire_Pos(1,k) - Pos(1); 
    y_tild = desire_Pos(2,k) - Pos(2);
    theta_tild = wrapToPi(atan2(y_tild, x_tild) - Pos(3));

    % 动态计算参考速度 vr(1) 和 vr(2)
    nextPos = desire_Pos(:, k+1);
    dx = nextPos(1) - Pos(1);
    dy = nextPos(2) - Pos(2);
    distance = sqrt(dx^2 + dy^2);
    % 线速度基于两点间的直线距离
    vr1 = distance / delta_T; % 假设每段路径的时间是固定的
    
    % 角速度基于角度变化
    desired_theta = atan2(dy, dx);
    dtheta = wrapToPi(desired_theta - Pos(3));
    vr2 = dtheta / delta_T;
    
    % 根据公式(20)和(21)计算控制输入
    v = vr(1)*cos(theta_tild) + k1*x_tild; 
    omega = k2*vr(1)*y_tild + vr(2)*sin(theta_tild) + k3*theta_tild*vr(1);
    
    % 更新位置
    Pos(1) = Pos(1) + v*cos(Pos(3))*delta_T;
    Pos(2) = Pos(2) + v*sin(Pos(3))*delta_T;  
    Pos(3) = wrapToPi(Pos(3) + (v/l)*tan(omega*l/v)*delta_T);
    
    % 添加轨迹点
    addpoints(h, desire_Pos(1,k), desire_Pos(2,k)); % 预期轨迹点
    
    % 更新实际轨迹线 
    addpoints(h_car, Pos(1), Pos(2)); % 添加小车当前位置点到轨迹
    
    % 更新车辆图像位置
    A.Rot = [cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*A.P;
    A.Prot_trasl = A.Rot + [ones(1, 4)*Pos(1); ones(1, 4)*Pos(2)];
    
    % 更新车辆图形（删除旧图形,绘制新图形）
    delete(A.P_robot); % 删除上一帧的小车图形
    A.P_robot = patch(A.Prot_trasl(1, :), A.Prot_trasl(2, :), 'b'); % 绘制新的小车图形
    
    % 绘制参考轨迹点
    addpoints(h, desire_Pos(1, k), desire_Pos(2, k));
    
    % 捕获当前图像帧并更新GIF
    frame = getframe(f3);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % 将当前帧写入GIF文件
    if k == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    
    % 强制图形更新
    drawnow
    
end