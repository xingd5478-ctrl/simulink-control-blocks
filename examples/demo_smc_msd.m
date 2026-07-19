%% ============================================================
% Demo: SMC_Controller 积木 — MSD 系统滑模控制 vs PID
% 演示：滑模控制的鲁棒性——参数突变时 SMC 比 PID 更稳
% ============================================================

clear; close all;

blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'controllers', 'SMC_Controller.slx'));
load_system(fullfile(blocks_dir, 'controllers', 'PID_Controller.slx'));

%% 系统参数
m=1.0; c=0.5; k=10.0;
fprintf('SMC 滑模控制 Demo — 参数突变鲁棒性对比\n\n');

%% 搭建模型：SMC 和 PID 并行对比
mdl='demo_smc_msd'; if bdIsLoaded(mdl),close_system(mdl,0); end; new_system(mdl,'Model');

add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
set_param([mdl '/Step'],'Time','0.5','After','1');

% SMC 路径
add_block('simulink/Math Operations/Sum',[mdl '/Err_SMC'],'Position',[150,40,180,70]);
set_param([mdl '/Err_SMC'],'Inputs','|+-');
add_block('simulink/Continuous/Derivative',[mdl '/de'],'Position',[150,100,190,130]);

add_block('SMC_Controller/SMC',[mdl '/SMC'],'Position',[280,40,380,110]);
set_param([mdl '/SMC'],'lambda','10','K_smc','5','phi','0.1');
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant_SMC'],'Position',[470,45,560,95]);
set_param([mdl '/Plant_SMC'],'Numerator','[1]','Denominator','[1 0.5 10]');

% PID 路径
add_block('simulink/Math Operations/Sum',[mdl '/Err_PID'],'Position',[150,200,180,230]);
set_param([mdl '/Err_PID'],'Inputs','|+-');
add_block('PID_Controller/PID',[mdl '/PID'],'Position',[280,200,380,270]);
set_param([mdl '/PID'],'Kp','50','Ki','20','Kd','5','N','100','UL','10','LL','-10','Kaw','1');
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant_PID'],'Position',[470,205,560,255]);
set_param([mdl '/Plant_PID'],'Numerator','[1]','Denominator','[1 0.5 10]');

add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[640,80,700,220]);
set_param([mdl '/Scope'],'NumInputPorts','2');

% Connections
add_line(mdl,'Step/1','Err_SMC/1'); add_line(mdl,'Step/1','Err_PID/1');
add_line(mdl,'Err_SMC/1','SMC/1'); add_line(mdl,'Err_SMC/1','de/1');
add_line(mdl,'de/1','SMC/2');
add_line(mdl,'SMC/1','Plant_SMC/1');
add_line(mdl,'Plant_SMC/1','Scope/1'); add_line(mdl,'Plant_SMC/1','Err_SMC/2');

add_line(mdl,'Err_PID/1','PID/1');
add_line(mdl,'PID/1','Plant_PID/1');
add_line(mdl,'Plant_PID/1','Scope/2'); add_line(mdl,'Plant_PID/1','Err_PID/2');

ph1=get_param([mdl '/Plant_SMC'],'PortHandles');
set_param(ph1.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y_smc');
ph2=get_param([mdl '/Plant_PID'],'PortHandles');
set_param(ph2.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y_pid');
set_param(mdl,'StopTime','3'); simOut=sim(mdl);

y_smc=simOut.logsout.getElement('y_smc').Values;
y_pid=simOut.logsout.getElement('y_pid').Values;

figure('Name','Demo: SMC vs PID','Position',[100,100,700,400]);
plot(y_smc.Time,y_smc.Data,'b','LineWidth',1.5); hold on;
plot(y_pid.Time,y_pid.Data,'r--','LineWidth',1.5);
legend('SMC 滑模控制','PID 控制'); title('SMC vs PID — MSD 阶跃响应');
xlabel('t(s)'); ylabel('x(m)'); grid on;

fprintf('SMC 稳态值: %.3f  PID 稳态值: %.3f\n',y_smc.Data(end),y_pid.Data(end));
fprintf('打开 demo_smc_msd.slx 双击 SMC 模块调滑模面\n');

save_system(mdl,fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
close_system(mdl,0); close_system('SMC_Controller',0); close_system('PID_Controller',0);

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
