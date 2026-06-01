% DESCRIPTION: Pure falling-edge detector for ETA convergence. Fires only when the
%              ETA to the next signal head crosses the clearance threshold downward.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_converged = Has_ETA_converged(in_curr_eta, in_prev_eta, in_t_despeje)
% HAS_ETA_CONVERGED - Pure falling-edge detection.
% Fires only when ETA crosses the clearance threshold downward.
% in_curr_eta:  Current ETA to signal head (pre-computed by caller).
% in_prev_eta:  ETA from previous step (NaN if no valid previous data).
% in_t_despeje: Dynamic clearance threshold for the current link.

out_converged = false;
% Skip if there is no valid previous measurement
if isnan(in_prev_eta), return; end
% Detect the falling edge: previous ETA was above threshold, current is at or below
if in_prev_eta > in_t_despeje && in_curr_eta <= in_t_despeje
    out_converged = true;
end
end
