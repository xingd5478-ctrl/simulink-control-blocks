function blk_path = block(name)
% BLOCK 快捷加载积木 — 一行代码拖进模型
%   block('PID')  → 返回 'PID_Controller/PID' 可直接 add_block
%   block('LQR')  → 返回 'LQR_Controller/LQR'
%
% 用法:
%   load_system(block('PID'));  % 预加载
%   add_block(block('PID'), 'mymodel/MyPID');

    root = fileparts(mfilename('fullpath'));
    blocks_dir = fullfile(root, 'blocks');

    map = containers.Map();
    map('PID')    = 'controllers/PID_Controller.slx|PID_Controller/PID';
    map('LQR')    = 'controllers/LQR_Controller.slx|LQR_Controller/LQR';
    map('SMC')    = 'controllers/SMC_Controller.slx|SMC_Controller/SMC';
    map('LeadLag')= 'controllers/LeadLag_Compensator.slx|LeadLag_Compensator/LeadLag';
    map('Luenberger') = 'observers/Luenberger_Observer.slx|Luenberger_Observer/Observer';
    map('Kalman') = 'observers/Kalman_Filter.slx|Kalman_Filter/Kalman';
    map('LowPass')= 'filters/LowPass_Filter.slx|LowPass_Filter/LPF';
    map('Notch')  = 'filters/Notch_Filter.slx|Notch_Filter/Notch';
    map('Complementary') = 'filters/Complementary_Filter.slx|Complementary_Filter/CompFilter';
    map('RateLimiter') = 'utilities/Rate_Limiter.slx|Rate_Limiter/RateLimit';
    map('AntiWindup')  = 'utilities/Anti_Windup.slx|Anti_Windup/AntiWindup';
    map('DynSat') = 'utilities/Dynamic_Saturation.slx|Dynamic_Saturation/DynSat';

    if ~map.isKey(name)
        error('Unknown block: %s. Available: %s', name, strjoin(keys(map), ', '));
    end

    parts = strsplit(map(name), '|');
    slx_file = fullfile(blocks_dir, parts{1});
    blk_path = parts{2};

    % Auto-load if not already loaded
    [~, mdl] = fileparts(slx_file);
    if ~bdIsLoaded(mdl)
        load_system(slx_file);
    end
end
