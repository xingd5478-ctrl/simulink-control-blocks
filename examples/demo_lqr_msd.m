%% ============================================================
% Demo: LQR_Controller 积木 — MSD 系统状态反馈控制
% 演示：加载 LQR 积木，设计 K 增益，对比开环 vs LQR 闭环
% ============================================================

clear; close all;

root = fileparts(mfilename('fullpath'));
blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'controllers', 'LQR_Controller.slx'));

%% 系统参数
m=1.0; c=0.5; k=10.0;
A=[0,1;-k/m,-c/m]; B=[0;1/m]; C=[1,0];

%% LQR 设计
Q=diag([100,1]); R=0.1;
K=lqr(A,B,Q,R);
fprintf('LQR 增益: K=[%.2f %.2f]\n',K);
fprintf('开环极点: %.2f ± j%.2f\n',real(eig(A)),abs(imag(eig(A))));
fprintf('闭环极点: %.2f ± j%.2f\n',real(eig(A-B*K)),abs(imag(eig(A-B*K))));

%% 搭建测试模型
mdl='demo_lqr_msd'; if bdIsLoaded(mdl),close_system(mdl,0); end; new_system(mdl,'Model');
add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
set_param([mdl '/Step'],'Time','0.5','After','1');

add_block('LQR_Controller/LQR',[mdl '/LQR'],'Position',[200,70,300,150]);
set_param([mdl '/LQR'],'K_vec',mat2str(K),'Ki','0','UL','10','LL','-10');

add_block('simulink/Continuous/State-Space',[mdl '/Plant'],'Position',[400,70,500,150]);
set_param([mdl '/Plant'],'A',mat2str(A),'B',mat2str(B),'C','eye(2)','D','[0;0]');

add_block('simulink/Signal Routing/Demux',[mdl '/Demux'],'Position',[560,80,580,140],'Outputs','2');
add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[650,80,710,140]);

add_line(mdl,'Step/1','LQR/2'); add_line(mdl,'LQR/1','Plant/1');
add_line(mdl,'Plant/1','Demux/1'); add_line(mdl,'Demux/1','Scope/1');
add_line(mdl,'Plant/1','LQR/1');  % 完整状态反馈

ph=get_param([mdl '/Demux'],'PortHandles');
set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
set_param(mdl,'StopTime','3'); simOut=sim(mdl);
y=simOut.logsout.getElement('y').Values;

%% 对比开环
[y_open,t_open]=step(tf([1],[m,c,k]),3);

figure('Name','Demo: LQR State Feedback','Position',[100,100,700,400]);
plot(t_open,y_open,'Color',[0.6,0.6,0.6],'LineWidth',1.5); hold on;
plot(y.Time,y.Data,'r','LineWidth',1.5);
legend('开环','LQR 闭环'); title('LQR 积木 — MSD 阶跃响应');
xlabel('t(s)'); ylabel('x(m)'); grid on;

S_open=stepinfo(y_open,t_open); S_lqr=stepinfo(y.Data,y.Time);
fprintf('\n开环: Tr=%.2fs Ts=%.2fs OS=%.0f%%\n',S_open.RiseTime,S_open.SettlingTime,S_open.Overshoot);
fprintf('LQR:  Tr=%.2fs Ts=%.2fs OS=%.0f%%\n',S_lqr.RiseTime,S_lqr.SettlingTime,S_lqr.Overshoot);
fprintf('\n打开模型 demo_lqr_msd.slx，双击 LQR 模块看参数\n');
fprintf('试试改 Q=diag(1000,1) 或 R=0.01 看增益变化\n');

save_system(mdl,fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
close_system(mdl,0); close_system('LQR_Controller',0);

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
