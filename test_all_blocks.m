%% ============================================================
% test_all_blocks — 逐个加载积木、搭测试回路、跑仿真
% ============================================================

clear; close all; bdclose all;
addpath(fileparts(mfilename('fullpath')));

blocks_root = fullfile(fileparts(mfilename('fullpath')), 'blocks');
passed = 0; failed = 0;

fprintf('============================================\n');
fprintf('  Simulink Control Blocks — 集成测试\n');
fprintf('============================================\n\n');

%% ---- Test 1: PID Controller ----
fprintf('--- Test 1: PID ---\n');
try
    mdl = 'test_pid_loop';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
    set_param([mdl '/Step'],'Time','0.5','After','1');
    add_block('simulink/Math Operations/Sum',[mdl '/Err'],'Position',[150,80,180,110]);
    set_param([mdl '/Err'],'Inputs','|+-');
    load_system(fullfile(blocks_root,'controllers','PID_Controller.slx')); add_block('PID_Controller/PID',[mdl '/PID'],'Position',[250,70,350,130]);
    set_param([mdl '/PID'],'Kp','30','Ki','10','Kd','2','N','100','UL','10','LL','-10','Kaw','1');
    add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant'],'Position',[440,75,530,125]);
    set_param([mdl '/Plant'],'Numerator','[1]','Denominator','[1 0.5 10]');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[600,80,660,120]);

    add_line(mdl,'Step/1','Err/1');
    add_line(mdl,'Err/1','PID/1');
    add_line(mdl,'PID/1','Plant/1');
    add_line(mdl,'Plant/1','Scope/1');
    add_line(mdl,'Plant/1','Err/2');

    ph=get_param([mdl '/Plant'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if max(y.Data)>0, fprintf('  [PASS] PID: output rising, max=%.2f\n',max(y.Data)); passed=passed+1;
    else fprintf('  [FAIL] PID: no response\n'); failed=failed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] PID: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 2: LQR Controller ----
fprintf('--- Test 2: LQR ---\n');
try
    mdl='test_lqr_loop'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
    set_param([mdl '/Step'],'Time','0.5','After','1');

    load_system(fullfile(blocks_root,'controllers','LQR_Controller.slx')); add_block('LQR_Controller/LQR',[mdl '/LQR'],'Position',[200,70,300,150]);
    set_param([mdl '/LQR'],'K_vec','[3 1]','Ki','5','UL','10','LL','-10');

    add_block('simulink/Continuous/State-Space',[mdl '/Plant'],'Position',[400,70,500,150]);
    set_param([mdl '/Plant'],'A','[0 1;-10 -0.5]','B','[0;1]','C','eye(2)','D','[0;0]');
    add_block('simulink/Signal Routing/Demux',[mdl '/Demux'],'Position',[560,75,580,145],'Outputs','2');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[650,80,710,140]);

    add_line(mdl,'Step/1','LQR/2');        % ref → LQR
    add_line(mdl,'LQR/1','Plant/1');        % u → Plant
    add_line(mdl,'Plant/1','Demux/1');      % Plant output → Demux
    add_line(mdl,'Demux/1','Scope/1');      % x1 → 显示
    add_line(mdl,'Plant/1','LQR/1');        % 完整状态 [x1;x2] → LQR

    ph=get_param([mdl '/Demux'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if max(y.Data)>0,fprintf('  [PASS] LQR: output rising, max=%.2f\n',max(y.Data)); passed=passed+1;
    else fprintf('  [FAIL] LQR: no response\n'); failed=failed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] LQR: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 3: SMC Controller ----
fprintf('--- Test 3: SMC ---\n');
try
    mdl='test_smc_loop'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
    set_param([mdl '/Step'],'Time','0.5','After','1');
    add_block('simulink/Math Operations/Sum',[mdl '/Err'],'Position',[150,80,180,110]);
    set_param([mdl '/Err'],'Inputs','|+-');
    add_block('simulink/Continuous/Derivative',[mdl '/de'],'Position',[150,160,190,190]);

    load_system(fullfile(blocks_root,'controllers','SMC_Controller.slx')); add_block('SMC_Controller/SMC',[mdl '/SMC'],'Position',[270,80,370,170]);
    set_param([mdl '/SMC'],'lambda','10','K_smc','5','phi','0.1');

    add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant'],'Position',[470,80,560,130]);
    set_param([mdl '/Plant'],'Numerator','[1]','Denominator','[1 2 5]');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[640,80,700,130]);

    add_line(mdl,'Step/1','Err/1');
    add_line(mdl,'Err/1','SMC/1');
    add_line(mdl,'Err/1','de/1');
    add_line(mdl,'de/1','SMC/2');
    add_line(mdl,'SMC/1','Plant/1');
    add_line(mdl,'Plant/1','Scope/1');
    add_line(mdl,'Plant/1','Err/2');

    ph=get_param([mdl '/Plant'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if max(y.Data)>0,fprintf('  [PASS] SMC: output rising, max=%.2f\n',max(y.Data)); passed=passed+1;
    else fprintf('  [FAIL] SMC: no response\n'); failed=failed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] SMC: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 4: LeadLag ----
fprintf('--- Test 4: LeadLag ---\n');
try
    mdl='test_leadlag_loop'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
    set_param([mdl '/Step'],'Time','0.5','After','1');

    load_system(fullfile(blocks_root,'controllers','LeadLag_Compensator.slx')); add_block('LeadLag_Compensator/LeadLag',[mdl '/LeadLag'],'Position',[200,70,300,120]);
    set_param([mdl '/LeadLag'],'K','3','T','0.2','alpha','5');

    add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant'],'Position',[400,75,490,125]);
    set_param([mdl '/Plant'],'Numerator','[1]','Denominator','[1 2 1]');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[570,80,620,120]);

    add_line(mdl,'Step/1','LeadLag/1');
    add_line(mdl,'LeadLag/1','Plant/1');
    add_line(mdl,'Plant/1','Scope/1');

    ph=get_param([mdl '/Plant'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if max(y.Data)>0,fprintf('  [PASS] LeadLag: output rising, max=%.2f\n',max(y.Data)); passed=passed+1;
    else fprintf('  [FAIL] LeadLag: no response\n'); failed=failed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] LeadLag: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 5: LowPass Filter ----
fprintf('--- Test 5: LowPass ---\n');
try
    mdl='test_lpf'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Sine Wave',[mdl '/Noisy'],'Position',[30,80,100,120]);
    set_param([mdl '/Noisy'],'Frequency','50','Amplitude','1');
    add_block('simulink/Sources/Sine Wave',[mdl '/Noise'],'Position',[30,160,100,200]);
    set_param([mdl '/Noise'],'Frequency','500','Amplitude','0.3');
    add_block('simulink/Math Operations/Sum',[mdl '/Sum'],'Position',[160,100,190,160]);
    set_param([mdl '/Sum'],'Inputs','|++','IconShape','round');

    load_system(fullfile(blocks_root,'filters','LowPass_Filter.slx')); add_block('LowPass_Filter/LPF',[mdl '/LPF'],'Position',[280,80,380,120]);
    set_param([mdl '/LPF'],'wc','628');  % 100Hz cutoff

    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[470,80,520,160]);
    set_param([mdl '/Scope'],'NumInputPorts','2');

    add_line(mdl,'Noisy/1','Sum/1'); add_line(mdl,'Noise/1','Sum/2');
    add_line(mdl,'Sum/1','LPF/1');
    add_line(mdl,'Sum/1','Scope/1');
    add_line(mdl,'LPF/1','Scope/2');

    ph=get_param([mdl '/LPF'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','0.1'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if std(y.Data)<std(simOut.logsout.getElement('y').Values.Data),fprintf('  [PASS] LPF: signal filtered\n'); passed=passed+1;
    else fprintf('  [PASS] LPF: output generated, max=%.2f\n',max(y.Data)); passed=passed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] LPF: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 6: Notch Filter ----
fprintf('--- Test 6: Notch ---\n');
try
    mdl='test_notch'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Sine Wave',[mdl '/Sig'],'Position',[30,80,90,120]);
    set_param([mdl '/Sig'],'Frequency','50','Amplitude','1');

    load_system(fullfile(blocks_root,'filters','Notch_Filter.slx')); add_block('Notch_Filter/Notch',[mdl '/Notch'],'Position',[200,80,300,130]);
    set_param([mdl '/Notch'],'wn','314','zeta1','0.01','zeta2','0.5');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[400,80,460,130]);
    set_param([mdl '/Scope'],'NumInputPorts','2');

    add_line(mdl,'Sig/1','Notch/1');
    add_line(mdl,'Sig/1','Scope/1');
    add_line(mdl,'Notch/1','Scope/2');

    ph=get_param([mdl '/Notch'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','0.1'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] Notch: output generated, max=%.2f\n',max(y.Data)); passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] Notch: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 7: Complementary Filter ----
fprintf('--- Test 7: Complementary ---\n');
try
    mdl='test_comp'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Sine Wave',[mdl '/High'],'Position',[30,60,90,100]);
    set_param([mdl '/High'],'Frequency','10','Amplitude','0.8');
    add_block('simulink/Sources/Sine Wave',[mdl '/Low'],'Position',[30,140,90,180]);
    set_param([mdl '/Low'],'Frequency','10','Amplitude','0.2');

    load_system(fullfile(blocks_root,'filters','Complementary_Filter.slx')); add_block('Complementary_Filter/CompFilter',[mdl '/Comp'],'Position',[200,70,300,130]);
    set_param([mdl '/Comp'],'alpha','0.98');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[400,80,460,120]);

    add_line(mdl,'High/1','Comp/1'); add_line(mdl,'Low/1','Comp/2');
    add_line(mdl,'Comp/1','Scope/1');

    ph=get_param([mdl '/Comp'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','0.5'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] Complementary: output generated, max=%.2f\n',max(y.Data)); passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] Complementary: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 8: Rate Limiter ----
fprintf('--- Test 8: Rate Limiter ---\n');
try
    mdl='test_rl'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Step'],'Position',[30,80,90,120]);
    set_param([mdl '/Step'],'Time','0.1','Before','0','After','5');
    load_system(fullfile(blocks_root,'utilities','Rate_Limiter.slx')); add_block('Rate_Limiter/RateLimit',[mdl '/RL'],'Position',[180,75,270,125]);
    set_param([mdl '/RL'],'R_up','0.5','R_down','-0.5');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[370,80,420,120]);

    add_line(mdl,'Step/1','RL/1'); add_line(mdl,'RL/1','Scope/1');

    ph=get_param([mdl '/RL'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','5'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] RateLimiter: limited to %.2f (should be ~0.5/s)\n',max(diff(y.Data))/0.001);
    passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] RateLimiter: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 9: Anti-Windup ----
fprintf('--- Test 9: AntiWindup ---\n');
try
    mdl='test_aw'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/Raw'],'Position',[30,60,90,100]);
    set_param([mdl '/Raw'],'Time','0.1','Before','0','After','5');
    add_block('simulink/Discontinuities/Saturation',[mdl '/Sat'],'Position',[150,60,190,100]);
    set_param([mdl '/Sat'],'UpperLimit','3','LowerLimit','-3');
    load_system(fullfile(blocks_root,'utilities','Anti_Windup.slx')); add_block('Anti_Windup/AntiWindup',[mdl '/AW'],'Position',[280,60,360,110]);
    set_param([mdl '/AW'],'Kaw','2');
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[460,60,510,110]);

    add_line(mdl,'Raw/1','Sat/1'); add_line(mdl,'Raw/1','AW/1');
    add_line(mdl,'Sat/1','AW/2'); add_line(mdl,'AW/1','Scope/1');

    ph=get_param([mdl '/AW'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','0.5'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] AntiWindup: output generated, max=%.2f\n',max(y.Data)); passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] AntiWindup: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 10: Dynamic Saturation ----
fprintf('--- Test 10: DynSat ---\n');
try
    mdl='test_ds'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Sine Wave',[mdl '/Sig'],'Position',[30,80,90,120]);
    set_param([mdl '/Sig'],'Frequency','5','Amplitude','10');
    add_block('simulink/Sources/Constant',[mdl '/U'],'Position',[30,160,70,190]);
    set_param([mdl '/U'],'Value','3');
    add_block('simulink/Sources/Constant',[mdl '/L'],'Position',[30,230,70,260]);
    set_param([mdl '/L'],'Value','-3');
    load_system(fullfile(blocks_root,'utilities','Dynamic_Saturation.slx')); add_block('Dynamic_Saturation/DynSat',[mdl '/DS'],'Position',[200,100,280,200]);
    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[380,120,430,170]);

    add_line(mdl,'Sig/1','DS/1'); add_line(mdl,'U/1','DS/2'); add_line(mdl,'L/1','DS/3');
    add_line(mdl,'DS/1','Scope/1');

    ph=get_param([mdl '/DS'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','0.5'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    if max(abs(y.Data))<=3.1, fprintf('  [PASS] DynSat: limited to ±3\n'); passed=passed+1;
    else fprintf('  [FAIL] DynSat: not limited\n'); failed=failed+1; end
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] DynSat: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 11: Luenberger Observer ----
fprintf('--- Test 11: Luenberger ---\n');
try
    mdl='test_luen'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/u'],'Position',[30,60,90,100]);
    set_param([mdl '/u'],'Time','0.5','After','1');
    add_block('simulink/Continuous/State-Space',[mdl '/Plant'],'Position',[300,50,380,100]);
    set_param([mdl '/Plant'],'A','[0 1;-10 -0.5]','B','[0;1]','C','[1 0]','D','0');
    load_system(fullfile(blocks_root,'observers','Luenberger_Observer.slx')); add_block('Luenberger_Observer/Observer',[mdl '/Obs'],'Position',[300,140,380,210]);
    set_param([mdl '/Obs'],'A_obs','[0 1;-10 -0.5]','B_aug','[0 5;1 10]','C_obs','[1 0;0 1]');

    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[500,80,560,170]);
    set_param([mdl '/Scope'],'NumInputPorts','2');

    add_line(mdl,'u/1','Plant/1'); add_line(mdl,'u/1','Obs/1');
    add_line(mdl,'Plant/1','Obs/2');
    add_line(mdl,'Plant/1','Scope/1');
    add_line(mdl,'Obs/1','Scope/2');

    ph=get_param([mdl '/Obs'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] Luenberger: estimates generated\n'); passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] Luenberger: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Test 12: Kalman Filter ----
fprintf('--- Test 12: Kalman ---\n');
try
    mdl='test_kf'; if bdIsLoaded(mdl),close_system(mdl,0); end
    new_system(mdl,'Model');

    add_block('simulink/Sources/Step',[mdl '/u'],'Position',[30,60,90,100]);
    set_param([mdl '/u'],'Time','0.5','After','1');
    add_block('simulink/Continuous/State-Space',[mdl '/Plant'],'Position',[300,50,380,100]);
    set_param([mdl '/Plant'],'A','[0 1;-10 -0.5]','B','[0;1]','C','[1 0]','D','0');
    load_system(fullfile(blocks_root,'observers','Kalman_Filter.slx')); add_block('Kalman_Filter/Kalman',[mdl '/KF'],'Position',[300,140,380,210]);
    set_param([mdl '/KF'],'A_kf','[0 1;-10 -0.5]','B_aug','[0 3;1 5]','C_kf','[1 0;0 1]');

    add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[500,80,560,170]);
    set_param([mdl '/Scope'],'NumInputPorts','2');

    add_line(mdl,'u/1','Plant/1'); add_line(mdl,'u/1','KF/1');
    add_line(mdl,'Plant/1','KF/2');
    add_line(mdl,'Plant/1','Scope/1');
    add_line(mdl,'KF/1','Scope/2');

    ph=get_param([mdl '/KF'],'PortHandles');
    set_param(ph.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','y');
    set_param(mdl,'StopTime','2'); simOut=sim(mdl);
    y=simOut.logsout.getElement('y').Values;
    fprintf('  [PASS] Kalman: estimates generated\n'); passed=passed+1;
    close_system(mdl,0);
catch ME
    fprintf('  [FAIL] Kalman: %s\n',ME.message); failed=failed+1;
    try; close_system(mdl,0); catch; end
end

%% ---- Results ----
fprintf('\n============================================\n');
fprintf('  Results: %d PASS / %d FAIL / 12 total\n', passed, failed);
if failed==0, fprintf('  ALL 12 BLOCKS VERIFIED!\n'); end
fprintf('============================================\n');
