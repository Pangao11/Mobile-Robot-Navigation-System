clear all; clc;

%% 定义常数
l = 0.2;           % 车辆的轴距
R = 1;             % 车辆的半径
delta_T = 0.5;     % 采样时间

%% PID 控制器参数
Kp1 = 0.1; Ki1 = 0.01; Kd1 = 0.05;
Kp2 = 0.05; Ki2 = 0.01; Kd2 = 0.05;

%% 初始化目标位置和朝向
x_r = 0; y_r = 0; theta_r = 0; % 目标位置和朝向

%% 定义初始点位姿
init_positions = [0, 3, pi/2; 3, 3, pi/2; 3, 0, pi/2; 3, -3, pi/2; ...
                  0, -3, pi/2; -3, -3, pi/2; -3, 0, pi/2; -3, 3, pi/2];

figure; hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('Robot Trajectories from Various Starting Points');

% 循环遍历每个初始点
for idx = 1:size(init_positions, 1)
    % 重置位置和误差记录
    Pos = init_positions(idx, :);
    trajectory = [];
    e = zeros(3,1); integral = zeros(3,1); derivative = zeros(3,1); prev_e = zeros(3,1);

    % 主循环
    for t = 0:delta_T:50 % 仿真时间可能需要根据情况调整
        % 计算极坐标下的误差
        dx = x_r - Pos(1);
        dy = y_r - Pos(2);
        rho = sqrt(dx^2 + dy^2);
        alpha = wrapToPi(atan2(dy, dx) - Pos(3));
        beta = wrapToPi(theta_r - atan2(dy, dx));
        
        % 计算控制输入
        v = 0.08 * rho; % ρ的比例增益调整
        omega = 0.6 * alpha - 0.05 * beta; % α和β的比例增益调整
        
        % 更新位置
        Pos(1) = Pos(1) + v * cos(Pos(3)) * delta_T;
        Pos(2) = Pos(2) + v * sin(Pos(3)) * delta_T;
        Pos(3) = wrapToPi(Pos(3) + omega * delta_T);

        % 记录轨迹
        trajectory = [trajectory; Pos(1), Pos(2)];
    end
    
    % 绘制轨迹
    plot(trajectory(:,1), trajectory(:,2));
    plot(x_r, y_r, 'ro'); % 目标位置
end

legend('Start 1','Start 2','Start 3','Start 4','Start 5','Start 6','Start 7','Start 8', 'Target');
