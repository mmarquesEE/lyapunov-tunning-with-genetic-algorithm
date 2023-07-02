function fitness = Fitness_valuesLYAPUNOV(pop)
    tfin = 5;
    xg=3; yg=0;
    xini=[0;0;pi/2];
    y_ref = 0.15;
    u_ref = 0.8;

    %time_limit = 2;
    optionsODE=odeset('RelTol',1e-6);
    n = 4;
    m = 20-n;         % number bits for fraction part of your number
    %
    [popsize, len] = size(pop);
    len_Palavra = len/2;
    %
    gp = pop(:,1:len_Palavra);
    gi = pop(:,len_Palavra+1:2*len_Palavra);
    %
    Vobj =zeros(popsize,1);
    
    for i = 1:popsize
        gamma = gp(i,:)*pow2(n-1:-1:-m).';
        k = gi(i,:)*pow2(n-1:-1:-m).';
        [t,x]=ode45(@(t,x)robotDynamics(x,gamma,k),[0 tfin],xini,optionsODE);
        
        xp=x(:,1);
        yp=x(:,2);
        phi=x(:,3);

        deltaX = xg-xp; deltaY = yg-yp;
        e = sqrt(deltaX.^2 + deltaY.^2);
        alpha = atan2(deltaY,deltaX) - phi;
        
        % Find the time when distance first becomes less than 0.05
        reach_index = find(e < 0.05, 1);
        if isempty(reach_index)
           reach_index = numel(t); % Assign last value of t
        end
        
        %% Performance criteria
        u = k .* e .* cos(alpha);

        yp = yp .* ((1:numel(yp))' < reach_index);
        Vobj1(i) = abs(max(yp) - y_ref);

        u = u .* ((1:numel(u))' < reach_index);
        Vobj2(i) = abs(max(u) - u_ref);

    end
    %dist_error = Vobj';
    fitness1 = 1 ./ Vobj1;
    fitness2 = 1 ./ Vobj2;

    fitness = fitness1 + fitness2;
end

%% Define P3DX robot dynamics
function dx = robotDynamics(x, gamma, k)
    % Unpack the state variables
    xg=3; yg=0;
    xp=x(1,1);
    yp=x(2,1);
    fp=x(3,1);
    %% Goal
    deltaX = xg-xp; deltaY = yg-yp;
    phi = fp;

    e = sqrt(deltaX.^2 + deltaY.^2);
    alpha = atan2(deltaY,deltaX) - phi;
    % Compute control inputs
    u = gamma * e * cos(alpha);
    omega = k * alpha + gamma * cos(alpha) * sin(alpha);
    
    % Update the state variables
    dx = [u * cos(fp); u * sin(fp); omega];
end
