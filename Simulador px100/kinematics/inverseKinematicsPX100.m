function [joints,info] = inverseKinematicsPX100(targetWristBase,links, ...
    minJointPos,maxJointPos,alpha3,shoulderHeight,elbowConfiguration)
% IK do PX100 para a posicao do wrist no referencial da base (metros)
% Orientacao da ferramenta: theta1 + theta2 + theta3 = alpha3

    if nargin < 5 || isempty(alpha3), alpha3 = 0; end
    if nargin < 6 || isempty(shoulderHeight), shoulderHeight = 0.0931; end
    if nargin < 7 || isempty(elbowConfiguration), elbowConfiguration = 'up'; end

    if numel(targetWristBase) ~= 3 || any(~isfinite(targetWristBase))
        error('targetWristBase tem de ser um vetor XYZ finito.');
    end
    if numel(links) < 4 || any(~isfinite(links(2:4)))
        error('links tem de conter L2, Lm e LB nas posicoes 2 a 4.');
    end
    if numel(minJointPos) < 4 || numel(maxJointPos) < 4
        error('Faltam os limites das quatro juntas.');
    end

    targetWristBase = double(targetWristBase(:)'); %converte para 1x3 indep. de onde vier
    LA = hypot(double(links(2)),double(links(3))); %L2 e LM
    LB = double(links(4));
    beta = atan2(double(links(3)),double(links(2)));
    psi = pi/2-beta;

    % Waist: orienta o plano do braco para o wrist.
    theta0 = atan2(targetWristBase(2),targetWristBase(1));

    % Posicao do wrist relativa ao ombro, no plano radial-vertical.
    Xed = hypot(targetWristBase(1),targetWristBase(2));
    Yed = targetWristBase(3)-shoulderHeight;
    R = hypot(Xed,Yed);
    minReach = abs(LA-LB);
    maxReach = LA+LB;
    if R < 1e-9 || R < minReach-1e-9 || R > maxReach+1e-9
        error('Wrist fora do alcance: R=%.4f m; intervalo [%.4f, %.4f] m.', ...
            R,minReach,maxReach);
    end

    % Triangulo LA-LB-R
    cosBeta1 = (LA^2+R^2-LB^2)/(2*LA*R);
    cosBeta2 = (LA^2+LB^2-R^2)/(2*LA*LB);
    cosBeta1 = max(-1,min(1,cosBeta1));
    cosBeta2 = max(-1,min(1,cosBeta2));
    beta1 = acos(cosBeta1);
    beta2 = acos(cosBeta2);
    gamma = atan2(Yed,Xed);

    switch lower(char(elbowConfiguration))
        case 'up'
            theta1 = pi/2-beta-beta1-gamma;
            theta2 = pi-psi-beta2;
        case 'down'
            theta1 = pi/2-(gamma-beta1+beta);
            theta2 = -pi+beta2-psi;
        otherwise
            error('elbowConfiguration deve ser ''up'' ou ''down''.');
    end

    %Impoe a orientacao pedida
    theta3 = alpha3-theta1-theta2;
    joints = [theta0,theta1,theta2,theta3];

    %Rejeita solucoes fora dos limites
    if any(joints < double(minJointPos(1:4))) || ...
            any(joints > double(maxJointPos(1:4)))
        error('A solucao IK excede os limites de uma ou mais juntas.');
    end

    % FK aqui devolve o wrist, igual ao ponto de entrada
    fkWrist = directKinematicsPX100(joints,links,shoulderHeight);
    info = struct('positionError',norm(fkWrist-targetWristBase));
end
