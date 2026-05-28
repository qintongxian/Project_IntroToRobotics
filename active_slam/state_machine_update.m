function [state, trigger_params] = state_machine_update(state, belief, trigger_params)
%STATE_MACHINE_UPDATE 主动SLAM状态机更新
%   状态: INIT -> REPLANNING -> EXECUTING -> (REPLANNING/LOOP_CLOSURE) -> TERMINATED
%
%   输入:
%     state          — 当前状态结构体 (.mode, .need_decision, .current_goal, ...)
%     belief         — 当前信念状态
%     trigger_params — 触发参数
%   输出:
%     state          — 更新后的状态
%     trigger_params — 更新后的触发参数（含reason）

    if ~isfield(state, 'mode')
        state.mode = 'INIT';
    end
    if ~isfield(state, 'need_decision')
        state.need_decision = false;
    end
    if ~isfield(state, 'loop_closure_completed')
        state.loop_closure_completed = false;
    end
    if ~isfield(state, 'exploration_complete')
        state.exploration_complete = false;
    end
    
    switch state.mode
        case 'INIT'
            % 等待SLAM收敛后首次触发决策
            if trigger_params.time_counter > 10
                state.mode = 'REPLANNING';
                state.need_decision = true;
                trigger_params.reason = 'init_first_decision';
            end
            
        case 'EXECUTING'
            % 高频SLAM持续运行，监控触发条件
            [should_replan, trigger_params] = trigger_conditions(belief, state, trigger_params);
            if should_replan
                state.mode = 'REPLANNING';
                state.need_decision = true;
            end
            
        case 'REPLANNING'
            % 决策完成后返回执行态
            if ~state.need_decision
                state.mode = 'EXECUTING';
            end
            
        case 'LOOP_CLOSURE'
            % 回环闭合模式
            if state.loop_closure_completed
                state.mode = 'REPLANNING';
                state.need_decision = true;
                state.loop_closure_completed = false;
            end
            
        case 'TERMINATED'
            % 探索完成，不再触发
            ;
    end
end
