function [mag, phaserad] = iqmethod(sig, len, f0, fs, pperiod) %#codegen
% KEEP THIS IN MIND
% This calculates the phasor from a reference signal of unknown phase
% relative to the reference signal: this means that so long as we compute
% the phasor of the input AND output, we can obtain the phasor of the
% output relative to the input, which is effectively what we want.
    %% ---- GET IN PHASE AND QUADRATURE REF SIGNALS ---- %%
    t = 1 : len;
    ang_step = (2.0 * pi * t * f0) / ( fs) ;
    inphase_sig = sin(ang_step);
    quadrature_sig = cos(ang_step);

    %% ---- SUM UP ---- %%
    size(sig)
    size(inphase_sig)
    I_sig = inphase_sig .* sig;
    Q_sig = quadrature_sig .* sig;

    I = sum(I_sig) / len;
    Q = sum(Q_sig) / len;

    mag = sqrt(I^2 + Q^2);
    phaserad = atan2(Q, I);

end