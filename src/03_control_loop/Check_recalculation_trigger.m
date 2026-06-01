% DESCRIPTION: Discrete event trigger state machine for SP/SC recalculation.
%              Evaluates four trigger conditions: decision link entry, signal head
%              bypass, ETA falling-edge convergence, and periodic timeout.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_recalculate, out_trigger_type] = Check_recalculation_trigger(in_curr_ev_link, in_last_ev_link, in_curr_ev_pos, in_last_ev_pos, in_is_decision_link, in_sp_current, in_pos_sh, in_link_sh, in_curr_eta, in_prev_eta, in_t_despeje, in_sim_time, in_last_recalc_time, in_recalc_period, in_link_lookup, in_sh_lookup)
% CHECK_RECALCULATION_TRIGGER - Discrete events: Decision, Bypass, ETA, Periodic.
% in_link_lookup: Lookup vector link_id -> index (precomputed).
% in_sh_lookup: Lookup vector sh_id -> index (precomputed).
% Other parameters: scalars pre-computed by the caller.

out_recalculate = false;
out_trigger_type = '';

% EVENT 1: Entry into a decision link (bifurcation confirmation)
if Has_entered_decision_link(in_curr_ev_link, in_last_ev_link, in_is_decision_link, in_link_lookup)
    out_recalculate = true;
    out_trigger_type = 'DECISION';
    return;
end

% EVENT 2: Physical bypass of a prioritized signal head
if Has_passed_signal_head(in_curr_ev_link, in_curr_ev_pos, in_last_ev_pos, in_sp_current, in_pos_sh, in_link_sh, in_sh_lookup)
    out_recalculate = true;
    out_trigger_type = 'BYPASS';
    return;
end

% EVENT 3: ETA falling-edge convergence (ETA crosses clearance threshold downward)
if Has_ETA_converged(in_curr_eta, in_prev_eta, in_t_despeje)
    out_recalculate = true;
    out_trigger_type = 'ETA';
    return;
end

% EVENT 4: Periodic recalculation (temporal guarantee)
if (in_sim_time - in_last_recalc_time) >= in_recalc_period
    out_recalculate = true;
    out_trigger_type = 'PERIODIC';
    return;
end

end
