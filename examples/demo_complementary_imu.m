%% ============================================================
% Demo: Complementary_Filter 积木 — IMU 姿态估计
% 演示：陀螺仪(高频准) + 加速度计(低频准) → 融合角度
% ============================================================

clear; close all;

root = fileparts(mfilename('fullpath'));
blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'filters', 'Complementary_Filter.slx'));

%% 模拟 IMU 数据: 陀螺仪积分(有漂移) + 加速度计(有噪声)
dt=0.01; t=0:dt:5; N=length(t);
true_angle = 10*sin(0.5*t);           % 真实角度 (慢振荡)
gyro_rate = gradient(true_angle)/dt;  % 真实角速度
gyro_angle = cumsum(gyro_rate*dt) + 0.01*t;  % 陀螺仪积分(漂移0.01deg/s)
acc_angle = true_angle + 2*randn(size(t));   % 加速度计(噪声2deg RMS)

fprintf('互补滤波 Demo — IMU 姿态估计\n');
fprintf('  陀螺仪: 短期准但长期漂移 %.2f deg/s\n',0.01);
fprintf('  加速度计: 长期无漂移但噪声 RMS=%.0f deg\n\n',2);

%% 搭建模型
mdl='demo_comp_imu'; if bdIsLoaded(mdl),close_system(mdl,0); end; new_system(mdl,'Model');

add_block('simulink/Sources/From Workspace',[mdl '/Gyro'],'Position',[30,50,100,90]);
set_param([mdl '/Gyro'],'VariableName','gyro_data');
gyro_data=[t(:),gyro_angle(:)];

add_block('simulink/Sources/From Workspace',[mdl '/Accel'],'Position',[30,140,100,180]);
set_param([mdl '/Accel'],'VariableName','acc_data');
acc_data=[t(:),acc_angle(:)];

add_block('Complementary_Filter/CompFilter',[mdl '/CF'],'Position',[220,70,320,140]);
set_param([mdl '/CF'],'alpha','0.98');

add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[420,60,480,150]);
set_param([mdl '/Scope'],'NumInputPorts','2');

add_line(mdl,'Gyro/1','CF/1'); add_line(mdl,'Accel/1','CF/2');
add_line(mdl,'CF/1','Scope/1'); add_line(mdl,'Accel/1','Scope/2');

ph=get_param([mdl '/CF'],'PortHandles');
set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','fused');
set_param(mdl,'StopTime',num2str(t(end))); simOut=sim(mdl);
fused=simOut.logsout.getElement('fused').Values;

figure('Name','Demo: Complementary Filter','Position',[100,100,700,400]);
plot(t,true_angle,'k--','LineWidth',1.5); hold on;
plot(t,gyro_angle,'b','LineWidth',0.8);
plot(t,acc_angle,'Color',[0.6,0.6,0.6],'LineWidth',0.5);
plot(fused.Time,fused.Data,'r','LineWidth',1.5);
legend('真实角度','陀螺仪(漂移)','加速度计(噪声)','互补滤波融合');
title('互补滤波积木 — IMU 姿态融合 (α=0.98)');
xlabel('t(s)'); ylabel('角度(deg)'); grid on;

err_gyro=abs(gyro_angle-true_angle); err_acc=abs(acc_angle-true_angle);
err_fused=abs(interp1(fused.Time,fused.Data,t)-true_angle);
fprintf('均方根误差: 陀螺仪=%.2fdeg  加速度计=%.2fdeg  融合=%.2fdeg\n',...
    rms(err_gyro),rms(err_acc),rms(err_fused(~isnan(err_fused))));

save_system(mdl,fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
close_system(mdl,0); close_system('Complementary_Filter',0);

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
