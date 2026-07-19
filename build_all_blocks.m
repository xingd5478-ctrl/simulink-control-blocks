%% ============================================================
% build_all_blocks — 一键生成全部 12 个控制工程积木
%
% 用法: >> build_all_blocks
% 每个积木是一个 Masked Subsystem，双击弹出参数对话框。
% ============================================================

function build_all_blocks()
    root = fileparts(mfilename('fullpath'));
    fprintf('=== Simulink Control Blocks v1.0 ===\n\n');

    build_PID(root);       build_LQR(root);
    build_SMC(root);       build_LeadLag(root);
    build_Luenberger(root);build_Kalman(root);
    build_LowPass(root);   build_Notch(root);
    build_Complementary(root);
    build_RateLimiter(root);build_AntiWindup(root);
    build_Saturation(root);

    fprintf('\n=== 12 blocks built! ===\n');
end

function setup_block(save_fn, mdl, sub_name)
    if exist(save_fn,'file'), delete(save_fn); end
    new_system(mdl);
    sub = [mdl '/' sub_name];
    add_block('simulink/Ports & Subsystems/Subsystem', sub);
    Simulink.SubSystem.deleteContents(sub);
end

function finish_block(mdl, sub, save_fn, mask_params)
    set_param(sub, 'Mask', 'on');
    if nargin >= 4
        set_param(sub, 'MaskVariables', mask_params{1});
        set_param(sub, 'MaskPrompts', mask_params{2});
        set_param(sub, 'MaskValues', mask_params{3});
        set_param(sub, 'MaskStyles', mask_params{4});
    end
    save_system(mdl, save_fn); close_system(mdl, 0);
    [~,name] = fileparts(save_fn);
    fprintf('  [OK] %s.slx\n', name);
end

%% ===== PID =====
function build_PID(root)
    out = fullfile(root,'blocks','controllers'); fn=fullfile(out,'PID_Controller.slx');
    setup_block(fn,'pid_mdl','PID'); sub=['pid_mdl/PID'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/e'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/u'],'Position',[700,220,720,240]);

    add_block('simulink/Math Operations/Gain',[sub '/Kp'],'Position',[120,30,170,70]);
    set_param([sub '/Kp'],'Gain','Kp');
    add_block('simulink/Math Operations/Gain',[sub '/Ki'],'Position',[120,100,170,140]);
    set_param([sub '/Ki'],'Gain','Ki');
    add_block('simulink/Continuous/Integrator',[sub '/Integrator'],'Position',[280,100,330,140]);
    add_block('simulink/Math Operations/Sum',[sub '/I_Sum'],'Position',[200,100,230,140]);
    set_param([sub '/I_Sum'],'Inputs','|++');

    add_block('simulink/Math Operations/Gain',[sub '/Kd'],'Position',[120,190,170,230]);
    set_param([sub '/Kd'],'Gain','Kd');
    add_block('simulink/Continuous/Transfer Fcn',[sub '/D_Filt'],'Position',[230,190,290,230]);
    set_param([sub '/D_Filt'],'Numerator','[N 0]','Denominator','[1 N]');

    add_block('simulink/Math Operations/Sum',[sub '/Sum'],'Position',[400,60,430,180]);
    set_param([sub '/Sum'],'Inputs','|+++','IconShape','round');
    add_block('simulink/Discontinuities/Saturation',[sub '/Sat'],'Position',[550,170,600,210]);
    set_param([sub '/Sat'],'UpperLimit','UL','LowerLimit','LL');

    add_block('simulink/Math Operations/Sum',[sub '/AW_Err'],'Position',[520,100,550,140]);
    set_param([sub '/AW_Err'],'Inputs','|+-');
    add_block('simulink/Math Operations/Gain',[sub '/AW_Gain'],'Position',[580,100,620,140]);
    set_param([sub '/AW_Gain'],'Gain','Kaw');

    add_line(sub,'e/1','Kp/1'); add_line(sub,'e/1','Ki/1'); add_line(sub,'e/1','Kd/1');
    add_line(sub,'Ki/1','I_Sum/1'); add_line(sub,'I_Sum/1','Integrator/1');
    add_line(sub,'Kd/1','D_Filt/1');
    add_line(sub,'Kp/1','Sum/1'); add_line(sub,'Integrator/1','Sum/2'); add_line(sub,'D_Filt/1','Sum/3');
    add_line(sub,'Sum/1','Sat/1'); add_line(sub,'Sat/1','u/1');
    add_line(sub,'Sum/1','AW_Err/1'); add_line(sub,'Sat/1','AW_Err/2');
    add_line(sub,'AW_Err/1','AW_Gain/1'); add_line(sub,'AW_Gain/1','I_Sum/2');

    set_param(sub,'MaskDisplay','disp(''PID'');');
    set_param(sub,'MaskDescription','PID Controller with anti-windup and derivative filtering');
    finish_block('pid_mdl',sub,fn,{'Kp=@1;Ki=@2;Kd=@3;N=@4;UL=@5;LL=@6;Kaw=@7',...
        {'Kp','Ki','Kd','N','Upper Limit','Lower Limit','Anti-Windup Gain'},...
        {'3','0.5','0.2','100','10','-10','1'},...
        {'edit','edit','edit','edit','edit','edit','edit'},...
        'Kp,Ki,Kd,N,UL,LL,Kaw'});
end

%% ===== LQR =====
function build_LQR(root)
    out = fullfile(root,'blocks','controllers'); fn=fullfile(out,'LQR_Controller.slx');
    setup_block(fn,'lqr_mdl','LQR'); sub=['lqr_mdl/LQR'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/x'], 'Position',[30,60,50,80]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/r'], 'Position',[30,150,50,170]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/u'],'Position',[700,100,720,120]);
    add_block('simulink/Signal Routing/Demux',[sub '/Demux'],'Position',[100,65,125,105]);
    set_param([sub '/Demux'],'Outputs','2');

    add_block('simulink/Math Operations/Sum',[sub '/Err'],'Position',[180,140,210,180]);
    set_param([sub '/Err'],'Inputs','|+-');
    add_block('simulink/Math Operations/Gain',[sub '/Ki'],'Position',[220,100,270,140]);
    set_param([sub '/Ki'],'Gain','Ki');
    add_block('simulink/Continuous/Integrator',[sub '/Int'],'Position',[330,100,380,140]);

    add_block('simulink/Math Operations/Gain',[sub '/K_gain'],'Position',[220,180,300,220]);
    set_param([sub '/K_gain'],'Gain','K_vec','Multiplication','Matrix(K*u)');

    add_block('simulink/Math Operations/Sum',[sub '/SumU'],'Position',[460,80,490,160]);
    set_param([sub '/SumU'],'Inputs','|+-+');
    add_block('simulink/Discontinuities/Saturation',[sub '/Sat'],'Position',[540,110,580,150]);
    set_param([sub '/Sat'],'UpperLimit','UL','LowerLimit','LL');

    add_line(sub,'x/1','Demux/1'); add_line(sub,'r/1','Err/1'); add_line(sub,'Demux/1','Err/2');
    add_line(sub,'Err/1','Ki/1'); add_line(sub,'Ki/1','Int/1');
    add_line(sub,'x/1','K_gain/1');
    add_line(sub,'K_gain/1','SumU/1'); add_line(sub,'Int/1','SumU/2');
    add_line(sub,'SumU/1','Sat/1'); add_line(sub,'Sat/1','u/1');

    set_param(sub,'MaskDisplay','disp(''LQR'');');
    set_param(sub,'MaskDescription','LQR State Feedback Controller with integral action');
    finish_block('lqr_mdl',sub,fn,{'K_vec=@1;Ki=@2;UL=@3;LL=@4',...
        {'K vector','Ki (integral)','Upper Limit','Lower Limit'},...
        {'[3 1]','0','10','-10'},{'edit','edit','edit','edit'},'K_vec,Ki,UL,LL'});
end

%% ===== SMC =====
function build_SMC(root)
    out = fullfile(root,'blocks','controllers'); fn=fullfile(out,'SMC_Controller.slx');
    setup_block(fn,'smc_mdl','SMC'); sub=['smc_mdl/SMC'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/e'], 'Position',[30,60,50,80]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/de'],'Position',[30,150,50,170]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/u'],'Position',[650,90,670,110]);

    add_block('simulink/Math Operations/Gain',[sub '/Lambda'],'Position',[120,50,170,90]);
    set_param([sub '/Lambda'],'Gain','lambda');
    add_block('simulink/Math Operations/Sum',[sub '/Surf'],'Position',[250,70,280,120]);
    set_param([sub '/Surf'],'Inputs','|++');
    add_line(sub,'e/1','Lambda/1'); add_line(sub,'Lambda/1','Surf/1'); add_line(sub,'de/1','Surf/2');

    add_block('simulink/Math Operations/Gain',[sub '/InvPhi'],'Position',[350,70,390,110]);
    set_param([sub '/InvPhi'],'Gain','1/phi');
    add_block('simulink/Discontinuities/Saturation',[sub '/Sat'],'Position',[430,70,470,110]);
    set_param([sub '/Sat'],'UpperLimit','1','LowerLimit','-1');
    add_line(sub,'Surf/1','InvPhi/1'); add_line(sub,'InvPhi/1','Sat/1');

    add_block('simulink/Math Operations/Gain',[sub '/K'],'Position',[530,70,570,110]);
    set_param([sub '/K'],'Gain','K_smc');
    add_line(sub,'Sat/1','K/1'); add_line(sub,'K/1','u/1');

    set_param(sub,'MaskDisplay','disp(''SMC'');');
    set_param(sub,'MaskDescription','Sliding Mode Controller with boundary layer');
    finish_block('smc_mdl',sub,fn,{'lambda=@1;K_smc=@2;phi=@3',...
        {'lambda (slope)','K (gain)','phi (boundary)'},...
        {'10','3','0.1'},{'edit','edit','edit'},'lambda,K_smc,phi'});
end

%% ===== Lead-Lag =====
function build_LeadLag(root)
    out = fullfile(root,'blocks','controllers'); fn=fullfile(out,'LeadLag_Compensator.slx');
    setup_block(fn,'ll_mdl','LeadLag'); sub=['ll_mdl/LeadLag'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/in'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/out'],'Position',[450,50,470,70]);
    add_block('simulink/Continuous/Transfer Fcn',[sub '/TF'],'Position',[150,35,250,85]);
    set_param([sub '/TF'],'Numerator','K*[alpha*T 1]','Denominator','[T 1]');
    add_line(sub,'in/1','TF/1'); add_line(sub,'TF/1','out/1');

    set_param(sub,'MaskDisplay','disp(''LeadLag'');');
    set_param(sub,'MaskDescription','Lead-Lag Compensator: Lead(alpha>1), Lag(alpha<1)');
    finish_block('ll_mdl',sub,fn,{'K=@1;T=@2;alpha=@3',...
        {'Gain K','Time const T','alpha (>1=Lead)'},...
        {'1','0.1','3'},{'edit','edit','edit'},'K,T,alpha'});
end

%% ===== Luenberger Observer =====
function build_Luenberger(root)
    out = fullfile(root,'blocks','observers'); fn=fullfile(out,'Luenberger_Observer.slx');
    setup_block(fn,'luen_mdl','Observer'); sub=['luen_mdl/Observer'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/u'], 'Position',[30,40,50,60]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/y'], 'Position',[30,150,50,170]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/xhat'],'Position',[650,80,670,100]);

    add_block('simulink/Signal Routing/Mux',[sub '/Mux'],'Position',[150,70,180,120]);
    set_param([sub '/Mux'],'Inputs','2');
    add_block('simulink/Continuous/State-Space',[sub '/SS'],'Position',[300,60,420,140]);
    set_param([sub '/SS'],'A','A_obs','B','B_aug','C','C_obs','D','D_aug','X0','x0');

    add_line(sub,'u/1','Mux/1'); add_line(sub,'y/1','Mux/2');
    add_line(sub,'Mux/1','SS/1'); add_line(sub,'SS/1','xhat/1');

    set_param(sub,'MaskDisplay','disp(''Luenberger\nObserver'');');
    set_param(sub,'MaskDescription','Luenberger Observer: dxhat=A*xhat+[B,L]*[u;y]. B_aug=[B, L]');
    finish_block('luen_mdl',sub,fn,{'A_obs=@1;B_aug=@2;C_obs=@3;D_aug=@4;x0=@5',...
        {'A matrix','B_aug=[B L]','C matrix','D matrix','x0'},...
        {'[0 1;-10 -0.5]','[0 5;1 10]','[1 0;0 1]','[0 0;0 0]','[0;0]'},...
        {'edit','edit','edit','edit','edit'},'A_obs,B_aug,C_obs,D_aug,x0'});
end

%% ===== Kalman Filter =====
function build_Kalman(root)
    out = fullfile(root,'blocks','observers'); fn=fullfile(out,'Kalman_Filter.slx');
    setup_block(fn,'kf_mdl','Kalman'); sub=['kf_mdl/Kalman'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/u'], 'Position',[30,40,50,60]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/y'], 'Position',[30,150,50,170]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/xhat'],'Position',[650,80,670,100]);

    add_block('simulink/Signal Routing/Mux',[sub '/Mux'],'Position',[150,70,180,120]);
    set_param([sub '/Mux'],'Inputs','2');
    add_block('simulink/Continuous/State-Space',[sub '/SS'],'Position',[300,60,420,140]);
    set_param([sub '/SS'],'A','A_kf','B','B_aug','C','C_kf','D','D_aug','X0','x0');

    add_line(sub,'u/1','Mux/1'); add_line(sub,'y/1','Mux/2');
    add_line(sub,'Mux/1','SS/1'); add_line(sub,'SS/1','xhat/1');

    set_param(sub,'MaskDisplay','disp(''Kalman\nFilter'');');
    set_param(sub,'MaskDescription','Kalman Filter. B_aug=[B, K_kalman] for input [u; y]');
    finish_block('kf_mdl',sub,fn,{'A_kf=@1;B_aug=@2;C_kf=@3;D_aug=@4;x0=@5',...
        {'A matrix','B_aug=[B K]','C matrix','D matrix','x0'},...
        {'[0 1;-10 -0.5]','[0 3;1 5]','[1 0;0 1]','[0 0;0 0]','[0;0]'},...
        {'edit','edit','edit','edit','edit'},'A_kf,B_aug,C_kf,D_aug,x0'});
end

%% ===== Low-Pass Filter =====
function build_LowPass(root)
    out = fullfile(root,'blocks','filters'); fn=fullfile(out,'LowPass_Filter.slx');
    setup_block(fn,'lpf_mdl','LPF'); sub=['lpf_mdl/LPF'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/in'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/out'],'Position',[420,50,440,70]);
    add_block('simulink/Continuous/Transfer Fcn',[sub '/TF'],'Position',[150,30,250,90]);
    set_param([sub '/TF'],'Numerator','[1]','Denominator','[1/(wc*wc) sqrt(2)/wc 1]');
    add_line(sub,'in/1','TF/1'); add_line(sub,'TF/1','out/1');

    set_param(sub,'MaskDisplay','disp(''LPF'');');
    set_param(sub,'MaskDescription','2nd-order Butterworth Low-Pass Filter');
    finish_block('lpf_mdl',sub,fn,{'wc=@1',{'Cutoff wc (rad/s)'},{'628'},{'edit'},'wc'});
end

%% ===== Notch Filter =====
function build_Notch(root)
    out = fullfile(root,'blocks','filters'); fn=fullfile(out,'Notch_Filter.slx');
    setup_block(fn,'notch_mdl','Notch'); sub=['notch_mdl/Notch'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/in'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/out'],'Position',[420,50,440,70]);
    add_block('simulink/Continuous/Transfer Fcn',[sub '/TF'],'Position',[150,30,260,90]);
    set_param([sub '/TF'],'Numerator','[1/(wn*wn) 2*zeta1/wn 1]','Denominator','[1/(wn*wn) 2*zeta2/wn 1]');
    add_line(sub,'in/1','TF/1'); add_line(sub,'TF/1','out/1');

    set_param(sub,'MaskDisplay','disp(''Notch'');');
    set_param(sub,'MaskDescription','Notch Filter: attenuate a specific frequency');
    finish_block('notch_mdl',sub,fn,{'wn=@1;zeta1=@2;zeta2=@3',...
        {'wn (rad/s)','zeta1 (notch depth)','zeta2 (notch width)'},...
        {'628','0.01','0.5'},{'edit','edit','edit'},'wn,zeta1,zeta2'});
end

%% ===== Complementary Filter =====
function build_Complementary(root)
    out = fullfile(root,'blocks','filters'); fn=fullfile(out,'Complementary_Filter.slx');
    setup_block(fn,'comp_mdl','CompFilter'); sub=['comp_mdl/CompFilter'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/high'],'Position',[30,40,50,60]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/low'], 'Position',[30,130,50,150]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/fused'],'Position',[550,80,570,100]);

    add_block('simulink/Math Operations/Gain',[sub '/Alpha'],'Position',[200,30,250,70]);
    set_param([sub '/Alpha'],'Gain','alpha');
    add_block('simulink/Math Operations/Gain',[sub '/OneMinus'],'Position',[200,130,250,170]);
    set_param([sub '/OneMinus'],'Gain','1-alpha');
    add_block('simulink/Math Operations/Sum',[sub '/Sum'],'Position',[380,60,410,140]);
    set_param([sub '/Sum'],'Inputs','|++');

    add_line(sub,'high/1','Alpha/1'); add_line(sub,'low/1','OneMinus/1');
    add_line(sub,'Alpha/1','Sum/1'); add_line(sub,'OneMinus/1','Sum/2');
    add_line(sub,'Sum/1','fused/1');

    set_param(sub,'MaskDisplay','disp(''CompFilter'');');
    set_param(sub,'MaskDescription','Complementary Filter: alpha*high + (1-alpha)*low');
    finish_block('comp_mdl',sub,fn,{'alpha=@1',{'alpha (0~1)'},{'0.98'},{'edit'},'alpha'});
end

%% ===== Rate Limiter =====
function build_RateLimiter(root)
    out = fullfile(root,'blocks','utilities'); fn=fullfile(out,'Rate_Limiter.slx');
    setup_block(fn,'rl_mdl','RateLimit'); sub=['rl_mdl/RateLimit'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/in'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/out'],'Position',[650,50,670,70]);

    % Error = input - output (rate-limited feedback)
    add_block('simulink/Math Operations/Sum',[sub '/Err'],'Position',[120,40,150,100]);
    set_param([sub '/Err'],'Inputs','|+-');

    % Limit the gradient (rate of change)
    add_block('simulink/Discontinuities/Saturation',[sub '/Sat'],'Position',[240,40,280,110]);
    set_param([sub '/Sat'],'UpperLimit','R_up','LowerLimit','R_down');

    % Integrate limited gradient to get rate-limited output
    add_block('simulink/Continuous/Integrator',[sub '/Int'],'Position',[380,40,430,110]);

    add_line(sub,'in/1','Err/1');
    add_line(sub,'Err/1','Sat/1');
    add_line(sub,'Sat/1','Int/1');
    add_line(sub,'Int/1','out/1');
    add_line(sub,'Int/1','Err/2');

    set_param(sub,'MaskDisplay','disp(''RateLimit'');');
    set_param(sub,'MaskDescription','Rate Limiter: limits signal rate via derivative+saturation+integrator');
    finish_block('rl_mdl',sub,fn,{'R_up=@1;R_down=@2',...
        {'Rise rate (+)','Fall rate (-)'},{'1','-1'},{'edit','edit'},'R_up,R_down'});
end

%% ===== Anti-Windup =====
function build_AntiWindup(root)
    out = fullfile(root,'blocks','utilities'); fn=fullfile(out,'Anti_Windup.slx');
    setup_block(fn,'aw_mdl','AntiWindup'); sub=['aw_mdl/AntiWindup'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/u_raw'],'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/u_sat'],'Position',[30,140,50,160]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/fb'],'Position',[500,90,520,110]);

    add_block('simulink/Math Operations/Sum',[sub '/Err'],'Position',[150,80,180,130]);
    set_param([sub '/Err'],'Inputs','|+-');
    add_block('simulink/Math Operations/Gain',[sub '/Kaw'],'Position',[300,90,340,130]);
    set_param([sub '/Kaw'],'Gain','Kaw');

    add_line(sub,'u_raw/1','Err/1'); add_line(sub,'u_sat/1','Err/2');
    add_line(sub,'Err/1','Kaw/1'); add_line(sub,'Kaw/1','fb/1');

    set_param(sub,'MaskDisplay','disp(''AntiWindup'');');
    set_param(sub,'MaskDescription','Anti-Windup: Kaw*(u_sat-u_raw)');
    finish_block('aw_mdl',sub,fn,{'Kaw=@1',{'Kaw'},{'1'},{'edit'},'Kaw'});
end

%% ===== Dynamic Saturation =====
function build_Saturation(root)
    out = fullfile(root,'blocks','utilities'); fn=fullfile(out,'Dynamic_Saturation.slx');
    setup_block(fn,'ds_mdl','DynSat'); sub=['ds_mdl/DynSat'];

    add_block('simulink/Ports & Subsystems/In1',[sub '/in'], 'Position',[30,50,50,70]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/UL'], 'Position',[30,140,50,160]);
    add_block('simulink/Ports & Subsystems/In1',[sub '/LL'], 'Position',[30,230,50,250]);
    add_block('simulink/Ports & Subsystems/Out1',[sub '/out'],'Position',[550,120,570,140]);

    add_block('simulink/Math Operations/MinMax',[sub '/Min'],'Position',[250,60,290,100]);
    set_param([sub '/Min'],'Function','min','Inputs','2');
    add_block('simulink/Math Operations/MinMax',[sub '/Max'],'Position',[420,80,460,120]);
    set_param([sub '/Max'],'Function','max','Inputs','2');

    add_line(sub,'in/1','Min/1'); add_line(sub,'UL/1','Min/2');
    add_line(sub,'Min/1','Max/1'); add_line(sub,'LL/1','Max/2');
    add_line(sub,'Max/1','out/1');

    set_param(sub,'MaskDisplay','disp(''DynSat'');');
    set_param(sub,'MaskDescription','Dynamic Saturation with variable limits via input ports');
    finish_block('ds_mdl',sub,fn);
end
