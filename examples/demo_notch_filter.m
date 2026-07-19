%% ============================================================
% Demo: Notch_Filter 积木 — 去除 50Hz 工频干扰
% 演示：含 50Hz 工频干扰的信号经过 Notch 滤波后恢复
% ============================================================

clear; close all;

blocks_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'blocks');
load_system(fullfile(blocks_dir, 'filters', 'Notch_Filter.slx'));

fprintf('Notch 陷波器 Demo — 去除 50Hz 工频干扰\n\n');

%% 搭建模型
mdl='demo_notch_filter'; if bdIsLoaded(mdl),close_system(mdl,0); end; new_system(mdl,'Model');

% 信号 = 10Hz 方波 + 50Hz 干扰
add_block('simulink/Sources/Sine Wave',[mdl '/10Hz_Sig'],'Position',[30,60,100,90]);
set_param([mdl '/10Hz_Sig'],'Frequency','10','Amplitude','1');
add_block('simulink/Sources/Sine Wave',[mdl '/50Hz_Noise'],'Position',[30,140,100,170]);
set_param([mdl '/50Hz_Noise'],'Frequency','50','Amplitude','0.3');
add_block('simulink/Math Operations/Sum',[mdl '/Mix'],'Position',[160,90,190,150]);
set_param([mdl '/Mix'],'Inputs','|++','IconShape','round');

add_block('Notch_Filter/Notch',[mdl '/Notch'],'Position',[280,90,380,150]);
set_param([mdl '/Notch'],'wn',num2str(2*pi*50),'zeta1','0.01','zeta2','0.5');

add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[480,90,540,150]);
set_param([mdl '/Scope'],'NumInputPorts','2');

add_line(mdl,'10Hz_Sig/1','Mix/1'); add_line(mdl,'50Hz_Noise/1','Mix/2');
add_line(mdl,'Mix/1','Notch/1');
add_line(mdl,'Mix/1','Scope/1');
add_line(mdl,'Notch/1','Scope/2');

ph=get_param([mdl '/Notch'],'PortHandles');
set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y_notch');
set_param(mdl,'StopTime','0.5'); simOut=sim(mdl);
y=simOut.logsout.getElement('y_notch').Values;

figure('Name','Demo: Notch Filter','Position',[100,100,700,400]);
idx=y.Time>0.1;  % skip transient
subplot(2,1,1);
plot(y.Time(idx),y.Data(idx),'b','LineWidth',1);
title('陷波后信号 — 50Hz 干扰被滤除'); grid on;
subplot(2,1,2);
[P,f]=pwelch(y.Data(idx),256,128,256,1000);
plot(f,10*log10(P)); xlim([0,100]);
title('功率谱 — 50Hz 处应有深谷'); xlabel('Hz'); ylabel('dB'); grid on;

fprintf('打开 demo_notch_filter.slx，Scope 对比含噪(蓝)和滤波后(红)\n');
fprintf('试试改 zeta1=0.1（更浅的陷波）看效果变化\n');

save_system(mdl,fullfile(fileparts(mfilename('fullpath')),[mdl '.slx']));
close_system(mdl,0); close_system('Notch_Filter',0);

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
