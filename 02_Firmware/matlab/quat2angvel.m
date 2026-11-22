function omega = quat2angvel(q, Fs, varargin)
    p = inputParser;
    addRequired(p, 'q');
    addRequired(p, 'Fs', @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Method', 'relative', @(s) any(strcmpi(s, {'relative','derivative'})));
    addParameter(p, 'SmoothWindow', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    parse(p, q, Fs, varargin{:});
    method = lower(p.Results.Method);
    smoothWin = round(p.Results.SmoothWindow);

    qm = localToQuatArray(q);
    N = size(qm, 1);
    if N < 2
        error('quat2angvel:tooFewSamples', 'At least two quaternion required.');
    end

    qm = qm ./ vecnorm(qm, 2, 2);

    dt = 1 / Fs;
    omega = zeros(N, 3);

    switch method
        case 'relative'
            if N >= 3
                for k = 2:N-1
                    omega(k, :) = localRelOmega(qm(k-1, :), qm(k+1, :), 2*dt);
                end
            end
            omega(1, :)   = localRelOmega(qm(1, :),   qm(2, :), dt);
            omega(N, :)   = localRelOmega(qm(N-1, :), qm(N, :), dt);

        case 'derivative'
            qdot = gradient(qm', dt)';
            for k = 1:N
                qc = localQuatConj(qm(k, :));
                wq = localQuatMult(qc, qdot(k, :));
                omega(k, :) = 2 * wq(2:4);
            end
    end

    if smoothWin > 1
        omega = movmean(omega, smoothWin, 1);
    end
end

function omega = localRelOmega(qA, qB, dt)
    dq = localQuatMult(localQuatConj(qA), qB);
    if dq(1) < 0
        dq = -dq;
    end
    w = min(1, max(-1, dq(1)));
    theta = 2 * acos(w);
    if theta < 1e-9
        omega = [0 0 0];
    else
        axis = dq(2:4) / sin(theta/2);
        omega = axis * theta / dt;
    end
end

function q = localToQuatArray(q)
    if isa(q, 'quaternion')
        q = compact(q);
    elseif ~(isnumeric(q) && size(q, 2) == 4)
        error('quat2angvel:badInput', 'q should be Nx4 [w x y z] or quaternion.');
    end
end

function qout = localQuatMult(q1, q2)
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    qout = [ w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
             w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
             w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
             w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end

function qout = localQuatConj(q)
    qout = [q(1), -q(2), -q(3), -q(4)];
end
