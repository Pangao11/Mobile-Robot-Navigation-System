clear all; clc;

%% 定义常数
l = 0.2;           % 车辆的轴距
R = 1;             % 车辆的半径
delta_T = 0.5;     % 采样时间

%% PID 控制器参数（假定为常数，应根据系统调整）
Kp1 = 0.1;          % 比例增益
Ki1 = 0.01;         % 积分增益
Kd1 = 0.05;         % 微分增益

Kp2 = 0.05;
Ki2 = 0.01;
Kd2 = 0.05;

% 定义λ
lambda = 0.8; 


%% 初始化位置和目标位置
Pos = [0, 0, 0]; % X, Y, Theta 初始位置和朝向
x_r = 10; y_r = 10; theta_r = pi/4; % 目标位置和朝向

%% 初始化误差和PID项
e = zeros(3,1); 
integral = zeros(3,1);
derivative = zeros(3,1);
prev_e = zeros(3,1);

%% 存储轨迹
trajectory = [];

%% 定义小车的形状
car_width = 0.4; % 小车宽度
car_length = 0.6; % 小车长度

car_shape = [0.5 -0.5 -0.5 0.5;   % x坐标
             0.25 0.25 -0.25 -0.25]; % y坐标
%% 主循环
% 准备 GIF 文件
filename = 'robot_trajectory.gif';

figure;
hold on;
% 绘制小车
car = patch(car_shape(1,:), car_shape(2,:), 'g');
car.FaceAlpha = 0.35;
car.EdgeColor = 'none';

%% 主循环
for t = 0:delta_T:300
    
    % 计算当前状态误差
    e(1) = x_r - Pos(1); % 目标点与当前点的x方向距离差
    e(2) = y_r - Pos(2); % 目标点与当前点的y方向距离差
    e(3) = wrapToPi(theta_r - Pos(3)); % 目标点与当前点的角度差（保持在 -pi 到 pi 之间）
    
    % 状态变换
    e_t = [cos(Pos(3)) sin(Pos(3)) 0; -sin(Pos(3)) cos(Pos(3)) 0; 0 0 1] * e;
    
    % 更新积分和微分项
    integral = integral + e_t * delta_T;
    derivative = (e_t - prev_e) / delta_T;

    % 加权平均误差
    weighted_error = lambda * e_t(2) + (1 - lambda) * e_t(3);
    
    % 计算控制增益矩阵K
    K = calculateK(t, e_t, Kp1, Ki1, Kd1);
    K1 = calculateK(t, e_t(1), Kp1, Ki1, Kd1);
    K2 = calculateK(t, e_t(2:3), Kp2, Ki2, Kd2);

    
    % 计算PID控制
%     v = K(1) * e_t(1) + K(2) * integral(1) + K(3) * derivative(1); % 线速度
%     omega = K(1) * weighted_error + K(2) * (lambda * integral(2) + (1 - lambda) * integral(3)) + K(3) * (lambda * derivative(2) + (1 - lambda) * derivative(3)); % 角速度
     v = K1(1) * e_t(1) + K1(2) * integral(1) + K1(3) * derivative(1); % 线速度
     omega = K2(1) * weighted_error + K2(2) * (lambda * integral(2) + (1 - lambda) * integral(3)) + K2(3) * (lambda * derivative(2) + (1 - lambda) * derivative(3)); % 角速度
    
    % 更新位置
    Pos(1) = Pos(1) + v * cos(Pos(3)) * delta_T;
    Pos(2) = Pos(2) + v * sin(Pos(3)) * delta_T;
    Pos(3) = wrapToPi(Pos(3) + (v/l) * tan(omega*l/v) * delta_T); % 更新角度
    
    % 更新小车位置
    car_X = Pos(1) + car_shape(1,:)*cos(Pos(3)) - car_shape(2,:)*sin(Pos(3));
    car_Y = Pos(2) + car_shape(1,:)*sin(Pos(3)) + car_shape(2,:)*cos(Pos(3));
    set(car, 'XData', car_X, 'YData', car_Y);
    

   
    prev_e = e_t;

    % 记录轨迹
    trajectory = [trajectory; Pos(1), Pos(2)];

    % 绘制轨迹
    plot(trajectory(:,1), trajectory(:,2), 'b', x_r, y_r, 'ro');
    xlabel('x (m)');
    ylabel('y (m)');
    title('Robot Trajectory');
    axis equal;
    grid on;
    drawnow;
    
    % 捕获当前帧
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % 将帧写入 GIF 文件
    if t == 0
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    
end
%% 绘制轨迹
figure;
plot(trajectory(:,1), trajectory(:,2), 'b', x_r, y_r, 'ro');
xlabel('x (m)');
ylabel('y (m)');
title('Robot Trajectory');
axis equal;
grid on;

%% 计算控制增益矩阵K的函数
function K = calculateK(t, e_t, Kp, Ki, Kd)
    % 基于时间t和状态误差e_t计算控制增益矩阵K
    % 这里我们假设增益为常数。根据实际情况，可能需要将t和e_t考虑在内。
    
    % 比例控制增益矩阵
    Kp_matrix = 0.45*Kp * eye(2); % 2x2单位矩阵乘以Kp
    
    % 积分控制增益矩阵
    Ki_matrix = 0.0001*Ki * eye(2); % 2x2单位矩阵乘以Ki
    
    % 微分控制增益矩阵
    Kd_matrix = 0.12*Kd * eye(2); % 2x2单位矩阵乘以Kd
    
    % 组合PID增益矩阵
    K = Kp_matrix + Ki_matrix + Kd_matrix;
end

