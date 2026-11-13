function wristBase = directKinematicsPX100(joints,links,shoulderHeight)
% FK do PX100: devolve apenas a posicao do wrist na base (metros).
% O elo LC e a garra ficam fora deste ponto final

    if nargin < 3 || isempty(shoulderHeight), shoulderHeight = 0.0931; end
    if numel(joints) ~= 4 || any(~isfinite(joints))
        error('joints tem de conter quatro angulos finitos em radianos.');
    end
    if numel(links) < 4 || any(~isfinite(links(2:4)))
        error('links tem de conter L2, Lm e LB nas posicoes 2 a 4.');
    end

    theta0 = double(joints(1));
    theta1 = double(joints(2));
    theta2 = double(joints(3));
    L2 = double(links(2));
    Lm = double(links(3));
    LB = double(links(4));
    LA = hypot(L2,Lm);
    beta = atan2(Lm,L2);

    % Posicao do wrist no plano radial-vertical.
    radial = LA*sin(theta1+beta)+LB*cos(theta1+theta2);
    height = LA*cos(theta1+beta)-LB*sin(theta1+theta2)+shoulderHeight;

    % Roda o plano radial para as coordenadas XYZ da base.
    wristBase = [radial*cos(theta0),radial*sin(theta0),height];
end
