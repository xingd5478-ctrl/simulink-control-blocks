%% ============================================================
% Demo: Kalman_Filter 积木 — 噪声环境下 MSD 状态估计
% 演示：真实状态 vs Kalman 估计状态，看滤波效果
% ============================================================

clear; close all;

root = fileparts(mfilename('fullpath'));
blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'observers', 'Kalman_Filter.slx'));

%% 系统参数
m=1.0; c=0.5; k_s=10.0;
A=[0,1;-k_s/m,-c/m]; B=[0;1/m]; C=[1,0];

fprintf('Kalman 滤波器 Demo — 状态估计\n');
fprintf('  A=[0 1;-10 -0.5], B_aug=[B K_kf]\n\n');

%% 搭建模型
mdl='demo_kalman_msd'; if bdIsLoaded(mdl),close_system(mdl,0); end; new_system(mdl,'Model');
add_block('simulink/Sources/Step',[mdl '/u'],'Position',[30,50,90,80]);
set_param([mdl '/u'],'Time','0.5','After','1');

add_block('simulink/Continuous/State-Space',[mdl '/Plant'],'Position',[300,30,380,80]);
set_param([mdl '/Plant'],'A','[0 1;-10 -0.5]','B','[0;1]','C','[1 0]','D','0');

add_block('Kalman_Filter/Kalman',[mdl '/KF'],'Position',[300,130,380,200]);
% B_aug = [B, K_kalman] — 两个输入列：[u] 和 [y]
set_param([mdl '/KF'],'A_kf','[0 1;-10 -0.5]','B_aug','[0 3;1 5]','C_kf','[1 0]','D_aug','[0 0]','x0','[0;0]');

add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[500,60,560,180]);
set_param([mdl '/Scope'],'NumInputPorts','2');

add_line(mdl,'u/1','Plant/1'); add_line(mdl,'u/1','KF/1');
add_line(mdl,'Plant/1','KF/2');
add_line(mdl,'Plant/1','Scope/1');
add_line(mdl,'KF/1','Scope/2');

ph=get_param([mdl '/Plant'],'PortHandles');
set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y_true');
ph2=get_param([mdl '/KF'],'PortHandles');
set_param(ph2.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y_est');
set_param(mdl,'StopTime','2'); simOut=sim(mdl);

y_true=simOut.logsout.getElement('y_true').Values;
y_est=simOut.logsout.getElement('y_est').Values;

figure('Name','Demo: Kalman Filter','Position',[100,100,700,400]);
plot(y_true.Time,y_true.Data,'Color',[0.6,0.6,0.6],'LineWidth',2); hold on;
plot(y_est.Time,y_est.Data(:,1),'r--','LineWidth',1.5);
legend('真实 x1','Kalman 估计 x̂1'); title('Kalman 积木 — 状态估计');
xlabel('t(s)'); grid on;

err=abs(y_true.Data-y_est.Data(:,1));
fprintf('\n估计误差: max=%.3f mean=%.3f\n',max(err),mean(err));
fprintf('\n打开模型 demo_kalman_msd.slx\n');
fprintf('Scope 对比真实状态(灰)和估计状态(红虚线)\n');

save_system(mdl,fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
close_system(mdl,0); close_system('Kalman_Filter',0);

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
