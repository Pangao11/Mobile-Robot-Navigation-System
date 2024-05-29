%% 定义超参数和初始设置
over = 50;
delta_t = 1; % 时间间隔
u = 1; % 加速度

Q = [0.1 0; 0 0.1]; % 过程噪声协方差矩阵
R = [1 0; 0 1]; % 观测噪声协方差矩阵
A = [1 delta_t; 0 1];
B = [(delta_t^2)/2; delta_t];
H = eye(2);
I = eye(2);

% 初始化状态
X = zeros(over, 2);
Xbar = zeros(over, 2);
Z = zeros(over, 2);
P = repmat(eye(2), over, 1); % 多次复制单位矩阵
X(1,:) = [0 1];
Xbar(1,:) = [0 1];

%% 卡尔曼滤波核心算法
for n = 2:over
        % 计算实际状态和观测值
    w1 = normrnd(0, sqrt(Q(1,1)));
    w2 = normrnd(0, sqrt(Q(2,2)));
    W = [w1, w2];
    X(n,:) = (A * X(n-1,:)' + B * u + W')';

    v1 = normrnd(0, sqrt(R(1,1)));
    v2 = normrnd(0, sqrt(R(2,2)));
    V = [v1, v2];
    Z(n,:) = H * X(n,:)' + V';

    % 先验估计
    X_bar = A * Xbar(n-1,:)' + B * u;

    P_ = A * reshape(P((n-1)*2-1:n*2-2,:), [2, 2]) * A' + Q;

    % 卡尔曼增益和后验更新
    K = P_ * H' / (H * P_ * H' + R);
    Xbar(n,:) = (X_bar + K * (Z(n,:)' - H * X_bar))';
    P((n-1)*2+1:n*2,:) = (I - K * H) * P_;
end

%% 绘图
% figure;
% subplot(2,1,1);
% plot(Z(:,1), 'k-'); hold on;
% plot(Xbar(:,1), 'b-'); hold on;
% plot(X(:,1), 'y-');
% title('位置状态曲线');
% legend('位置测量值', '位置最优估计值', '实际位置');
% 
% subplot(2,1,2);
% plot(Z(:,2), 'k-'); hold on;
% plot(Xbar(:,2), 'b-'); hold on;
% plot(X(:,2), 'y-');
% title('速度状态曲线');
% legend('速度测量值', '速度最优估计值', '实际速度');

%% 绘图
figure;
% 位置状态曲线放大显示第 20 到第 30 阶段
subplot(2,1,1);
plot(20:30, Z(20:30,1), 'k-'); hold on; % 显示第20到30阶段的位置测量值
plot(20:30, Xbar(20:30,1), 'b-'); hold on; % 显示第20到30阶段的最优估计位置值
plot(20:30, X(20:30,1), 'y-'); hold on; % 显示第20到30阶段的实际位置值
title('位置状态曲线 (阶段 20-30)');
legend('位置测量值', '位置最优估计值', '实际位置');

% 速度状态曲线放大显示第 20 到第 30 阶段
subplot(2,1,2);
plot(20:30, Z(20:30,2), 'k-'); hold on; % 显示第20到30阶段的速度测量值
plot(20:30, Xbar(20:30,2), 'b-'); hold on; % 显示第20到30阶段的最优估计速度值
plot(20:30, X(20:30,2), 'y-'); hold on; % 显示第20到30阶段的实际速度值
title('速度状态曲线 (阶段 20-30)');
legend('速度测量值', '速度最优估计值', '实际速度');

