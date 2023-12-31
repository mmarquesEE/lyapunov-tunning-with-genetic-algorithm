clear
close
clc
%% Dynamic simulation setup
dini = 3; aini = pi;
yref = 0.1; uref = 0.8;
tfin = 5;

%% Genetic algorithm config
len=40;                    % The length of the genomes  
n = 1;
popSize=200;               % The size of the population (must be an even number)
maxGens=40;              % The maximum number of generations allowed in a run
probCrossover=1;           % The probability of crossing over. 
probMutation=0.003;        % The mutation probability (per bit)
sigmaScalingFlag=1;        % Sigma Scaling is described on pg 168 of M. Mitchell's
                           % GA book. It often improves GA performance.
sigmaScalingCoeff=1;       % Higher values => less fitness pressure 

SUSFlag=1;                 % 1 => Use Stochastic Universal Sampling (pg 168 of 
                           %      M. Mitchell's GA book)
                           % 0 => Do not use Stochastic Universal Sampling
                           %      Stochastic Universal Sampling almost always
                           %      improves performance

crossoverType=2;           % 0 => no crossover
                           % 1 => 1pt crossover
                           % 2 => uniform crossover

visualizationFlag=1;       % 0 => don't visualize bit frequencies
                           % 1 => visualize bit frequencies

verboseFlag=1;             % 1 => display details of each generation
                           % 0 => run quietly

useMaskRepositoriesFlag=1; % 1 => draw uniform crossover and mutation masks from 
                           %      a pregenerated repository of randomly generated bits. 
                           %      Significantly improves the speed of the code with
                           %      no apparent changes in the behavior of
                           %      the SGA
                           % 0 => generate uniform crossover and mutation
                           %      masks on the fly. Slower. 

% crossover masks to use if crossoverType==0.
mutationOnlycrossmasks=false(popSize,len);

maskReposFactor=5;
uniformCrossmaskRepos=rand(popSize/2,(len+1)*maskReposFactor)<0.5;
mutmaskRepos=rand(popSize,(len+1)*maskReposFactor)<probMutation;

avgFitnessHist=zeros(1,maxGens+1);
maxFitnessHist=zeros(1,maxGens+1);
    
eliteIndiv=zeros(maxGens+1,len);
eliteFitness=-realmax;


pop=rand(popSize,len)<.5;

for gen=0:maxGens
    fitnessVals=Fitness_valuesLYAPUNOV(pop,...
            tfin=tfin,...
            dini=dini,...
            aini=aini,...
            y_ref=yref,...
            u_ref=uref...
        );
     
    [maxFitnessHist(1,gen+1),maxIndex]=max(fitnessVals);   
    avgFitnessHist(1,gen+1)=mean(fitnessVals);
    if eliteFitness<maxFitnessHist(gen+1)
        eliteFitness=maxFitnessHist(gen+1);
        eliteIndiv(gen+1,:)=pop(maxIndex,:);
    end    
    
    % display the generation number, the average Fitness of the population,
    % and the maximum fitness of any individual in the population
    if verboseFlag
        display(['gen=' num2str(gen,'%.3d') '   avgFitness=' ...
            num2str(avgFitnessHist(1,gen+1),'%3.3f') '   maxFitness=' ...
            num2str(maxFitnessHist(1,gen+1),'%3.3f')]);
    end
    % Conditionally perform bit-frequency visualization
    if visualizationFlag
        figure(1)
        set (gcf, 'color', 'w');
        hold off
        bitFreqs=sum(pop)/popSize;
        plot(1:len,bitFreqs, '.');
        axis([0 len 0 1]);
        title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
        ylabel('Frequency of the Bit 1');
        xlabel('Locus');
        drawnow;
    end    

    % Conditionally perform sigma scaling 
    if sigmaScalingFlag
        sigma=std(fitnessVals);
        if sigma~=0;
            fitnessVals=1+(fitnessVals-mean(fitnessVals))/...
            (sigmaScalingCoeff*sigma);
            fitnessVals(fitnessVals<=0)=0;
        else
            fitnessVals=ones(popSize,1);
        end
    end        
    
    cumNormFitnessVals=cumsum(fitnessVals/sum(fitnessVals));

    if SUSFlag
        markers=rand(1,1)+[1:popSize]/popSize;
        markers(markers>1)=markers(markers>1)-1;
    else
        markers=rand(1,popSize);
    end
    [~,parentIndices]=histc(markers,[0 cumNormFitnessVals]);
    parentIndices=parentIndices(randperm(popSize));    

    firstParents=pop(parentIndices(1:popSize/2),:);
    secondParents=pop(parentIndices(popSize/2+1:end),:);
    
    % create crossover masks
    if crossoverType==0
        masks=mutationOnlycrossmasks;
    elseif crossoverType==1
        masks=false(popSize/2, len);
        temp=ceil(rand(popSize/2,1)*(len-1));
        for i=1:popSize/2
            masks(i,1:temp(i))=true;
        end
    else
        temp=floor(rand*len*(maskReposFactor-1));
        masks=uniformCrossmaskRepos(:,temp+1:temp+len);
    end
    
    % determine which parent pairs to leave uncrossed
    reprodIndices=rand(popSize/2,1)<1-probCrossover;
    masks(reprodIndices,:)=false;
    
    % implement crossover
    firstKids=firstParents;
    firstKids(masks)=secondParents(masks);
    secondKids=secondParents;
    secondKids(masks)=firstParents(masks);
    pop=[firstKids; secondKids];
    
    % implement mutation
    if useMaskRepositoriesFlag
        temp=floor(rand*len*(maskReposFactor-1));
        masks=mutmaskRepos(:,temp+1:temp+len);
    else
        masks=rand(popSize, len)<probMutation;
    end
    pop=xor(pop,masks);    
end
if verboseFlag
    figure(2)
    %set(gcf,'Color','w');
    hold off
    plot([0:maxGens],avgFitnessHist,'k-');
    hold on
    plot([0:maxGens],maxFitnessHist,'c-');
    title('Maximum and Average Fitness')
    xlabel('Generation')
    ylabel('Fitness')
end
[maxValue, maxIndex] = max(avgFitnessHist);
n = 4;
m = 20 - n;

[popsize, len] = size(pop);
len_Palavra = len/2;

gp = pop(:,1:len_Palavra);
gi = pop(:,len_Palavra+1:2*len_Palavra);

gamma = gp*pow2(n-1:-1:-m).';
k = gi*pow2(n-1:-1:-m).';

c = [gamma k];
gamma_10_in_10 = c(1:40:end,1);
kappa_10_in_10 = c(1:40:end,2);

gamma_end = c(end,1);
kappa_end = c(end,2);

gamma_10_in_10(end+1) = gamma_end;
kappa_10_in_10(end+1) = kappa_end;

tfin = 10;
xini=[0;0;aini];
optionsODE=odeset('RelTol',1e-6);

% Define line colors and styles
lineColors = lines(size(gamma_10_in_10, 1));
lineStyles = {'-', '--', ':', '-.'};

%% Create Figures for the state plots
figure(3);
%hold on;

% Perform the simulations using a for loop
for i = 1:size(gamma_10_in_10)
    if i == 1
        popu = 1;
    elseif i == 2
        popu = 40;
    elseif i == 3
        popu = 80;
    elseif i == 4
        popu = 120;
    elseif i == 5
        popu = 160;
    elseif i == 6
        popu = 200;
    end
    % Perform the ode45 simulation for the current gamma and kappa
    [t, x] = ode45(@(t, x) robotDynamics(x, gamma_10_in_10(i), kappa_10_in_10(i), dini),...
                                            [0 tfin], xini, optionsODE);
    
    % Plot the results for the current simulation in Figure 4
    plot(x(:, 1), x(:, 2), 'Color', lineColors(i,:), 'LineStyle', lineStyles{1}, 'LineWidth', 1.5, 'DisplayName', ['Pop: ' num2str(popu) ', \gamma: ' num2str(gamma_10_in_10(i)) ', \kappa: ' num2str(kappa_10_in_10(i))]);
    hold on;
end

% Add labels, legend, and title to Figure 4
xlabel('x, [m]');
ylabel('y, [m]');
axis equal;
title('y vesus x');
% Set legend and adjust its properties
legend('show', 'Interpreter', 'tex', 'FontSize', 10, 'Location', 'best');

% Customize the appearance of the plot
grid on;
box on;
set(gca, 'FontName', 'Arial', 'FontSize', 10);
set(gcf, 'Color', 'w');

% Adjust the figure size and position
set(gcf, 'Units', 'inches', 'Position', [1, 1, 6, 4]);
hold off;

%% Figuras
for i=1:3
    saveas(figure(i),"fig"+i+".png");
end

%%
function dx = robotDynamics(x, gamma, k, xg)
    
    xp=x(1,1);
    yp=x(2,1);
    fp=x(3,1);
    
    %% Goal
    deltaX = xg-xp; deltaY = 0-yp;
    phi = fp;

    e = sqrt(deltaX.^2 + deltaY.^2);
    alpha = atan2(deltaY,deltaX) - phi;

    % Compute control inputs
    % Adjust the control law based on the P3DX robot dynamics

    u = gamma * e * cos(alpha);
    omega = k * alpha + gamma * cos(alpha) * sin(alpha);
    
    % Update the state variables
    dx = [u * cos(fp); u * sin(fp); omega];
end



