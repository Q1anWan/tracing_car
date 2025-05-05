%————— 参数 —————%
ld = 0.084;      % d1/2
m  = 1.267;      % 质量 kg
r  = 0.0325;     % 轮半径 m
I  = 5035.443e-6;     % 转动惯量 kg·m^2

Ts = 0.001;    % 采样周期 s
T  = 5;        % 仿真总时长 s
N  = T / Ts;   % 仿真步数

%————— LQR 权重 —————%
Q = diag([8,1,6,0.25]);
R = diag([1,1]);

%————— 连续系统 (已修正) —————%
A = [ 0 1 0 0;
      0 0 0 0;
      0 0 0 1;  % 偏航角到角速度
      0 0 0 0 ];

B = [ 0,                 0;
      1/(m*r),          -1/(m*r);
      0,                 0;
     -ld/(I*r),         -ld/(I*r) ];  % 差动符号

C = eye(4);
D = zeros(4,2);

%————— 离散化 —————%
sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

%————— 计算 LQR 增益 —————%
[K,~,~] = dlqr(Ad, Bd, Q, R);
disp('-K = ');
fprintf('{%f,%f,%f,%f,%f,%f,%f,%f};\n',-K(1,1),-K(1,2), -K(1,3), -K(1,4),-K(2,1),-K(2,2), -K(2,3), -K(2,4));

%————— 初始化—— 状态 & 控制 —————%
x = zeros(4, N+1);
u = zeros(2, N);
% 初始状态：[横向位移 e1；横向速度 e1_dot；偏航角 e2；偏航角速度 e2_dot]
x(:,1) = [1; 0; 0; 0];

%————— 仿真循环 —————%
for k = 1:N
    u(:,k)   = -K * x(:,k);       % 2×1 向量
    x(:,k+1) = Ad * x(:,k) + Bd * u(:,k);
end

%————— 时间向量 —————%
time  = 0 : Ts : T;         % 长度 N+1
timeu = time(1:end-1);      % 长度 N

%————— 绘制误差收敛 —————%
figure;
plot(time, x(1,:), 'b-', time, x(3,:), 'r-');
xlabel('Time (s)');
ylabel('Error');
legend('Lateral e_1 (m)', 'Heading e_2 (rad)');
grid on;
title('Error Convergence');

%————— 绘制扭矩指令 —————%
figure;
plot(timeu, u(1,:), 'k-', timeu, u(2,:), 'm--');
xlabel('Time (s)');
ylabel('Torque (N·m)');
legend('\tau_L','\tau_R');
grid on;
title('Wheel Torque Commands');
