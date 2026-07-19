%% ============================================================
% Demo: PID_Controller 积木 — MSD 系统位置控制
% 演示如何加载积木库中的 PID 模块并快速搭建控制系统
% ============================================================

clear; close all;

% 加载积木
blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'controllers', 'PID_Controller.slx'));

mdl = 'demo_pid_msd';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 被控对象：MSD 系统 G(s) = 1/(s²+0.5s+10) ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], 'Position', [30,80,90,120]);
set_param([mdl '/Setpoint'], 'Time', '0.5', 'After', '1');

% 误差计算
add_block('simulink/Math Operations/Sum', [mdl '/Error'], 'Position', [160,80,190,120]);
set_param([mdl '/Error'], 'Inputs', '|+-');

% === 加载积木：PID 控制器 ===
add_block('PID_Controller/PID', [mdl '/PID'], 'Position', [280,70,380,130]);
% 设置 Mask 参数
set_param([mdl '/PID'], 'Kp', '50', 'Ki', '20', 'Kd', '5', ...
    'N', '100', 'UL', '10', 'LL', '-10', 'Kaw', '1');

% 被控对象
add_block('simulink/Continuous/Transfer Fcn', [mdl '/MSD_Plant'], 'Position', [480,75,570,125]);
set_param([mdl '/MSD_Plant'], 'Numerator', '[1]', 'Denominator', '[1 0.5 10]');

% 示波器
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [660,75,720,125]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');

% 连线
add_line(mdl, 'Setpoint/1', 'Error/1');
add_line(mdl, 'Error/1', 'PID/1');
add_line(mdl, 'PID/1', 'MSD_Plant/1');
add_line(mdl, 'MSD_Plant/1', 'Scope/1');
add_line(mdl, 'Setpoint/1', 'Scope/2');
add_line(mdl, 'MSD_Plant/1', 'Error/2');

% 信号记录
ph = get_param([mdl '/MSD_Plant'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_out');

% 仿真
simOut = sim(mdl);
y = simOut.logsout.getElement('y_out').Values;

figure('Name', 'Demo: PID积木 — MSD控制', 'Position', [100,100,600,400]);
plot(y.Time, y.Data, 'b', 'LineWidth', 2);
title('PID Controller Block Demo — MSD Step Response');
xlabel('Time (s)'); ylabel('Position (m)'); grid on;

fprintf('========================================\n');
fprintf('  Demo: PID积木控制MSD系统\n');
fprintf('========================================\n');
fprintf('  打开模型，双击 PID 模块查看内部结构\n');
fprintf('  双击 PID 模块弹出参数对话框\n');
fprintf('  试试把 Kp 从 50 改成 20，重新运行看效果\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));

%% Auto-save figure
figs = findall(0,'Type','figure');
img_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'docs', 'images');
if ~exist(img_dir,'dir'), mkdir(img_dir); end
for k = 1:length(figs)
    if isgraphics(figs(k),'figure')
        [~,sn] = fileparts(mfilename('fullpath'));
        exportgraphics(figs(k), fullfile(img_dir, [sn '_fig' num2str(k) '.png']), 'Resolution', 150);
    end
end
fprintf('Image saved: %s\n', sn);
