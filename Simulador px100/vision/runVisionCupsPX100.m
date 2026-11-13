function objectInfo = runVisionCupsPX100(simInterface,showOutput,cupHeight)
% Deteta os copos, mostra os passos e calcula os alvos XYZ.

    if nargin < 2 || isempty(showOutput), showOutput = true; end
    if nargin < 3 || isempty(cupHeight), cupHeight = 0.070; end
    tableZ = 0.0;

    [image,~] = simInterface.get_camera_information();
    image = uint8(image);
    [cups,debug] = detectCupsPX100(image);

    for k = 1:numel(cups)
        % Pixel do centro do rebordo -> intersecao com o plano do topo.
        topCenter = calibrationPX100('projectplane', ...
            cups(k).centroid,tableZ+cupHeight,false);
        cups(k).topCenterBase = topCenter;

        % O X/Y mantem-se; Z desce meia altura para o centro do copo.
        cups(k).graspTargetBase = [topCenter(1:2), ...
            tableZ+cupHeight/2];
    end

    objectInfo = struct('image',image,'cups',cups, ...
        'cupHeight',cupHeight,'tableZ',tableZ);

    if showOutput
        showVisionSteps(image,cups,debug,tableZ+cupHeight);
        if ~isempty(cups)
            showProjectionGeometry(cups,tableZ+cupHeight);
        end
    end
end

function showVisionSteps(image,cups,debug,topZ)
    figure('Name','Detecao dos copos - passos da visao');
    tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

    ax = nexttile;
    imshow(image,'Parent',ax); hold(ax,'on');
    for k = 1:numel(cups)
        color = cupPlotColor(cups(k).color);
        rectangle(ax,'Position',cups(k).boundingBox, ...
            'EdgeColor',color,'LineWidth',1.5);
        angle = linspace(0,2*pi,100);
        plot(ax,cups(k).centroid(1)+cups(k).radius*cos(angle), ...
            cups(k).centroid(2)+cups(k).radius*sin(angle), ...
            'k-','LineWidth',1.5);
        plot(ax,cups(k).centroid(1),cups(k).centroid(2),'y+', ...
            'MarkerSize',12,'LineWidth',2);
        text(ax,cups(k).centroid(1)+6,cups(k).centroid(2), ...
            cups(k).color,'Color',color,'FontWeight','bold', ...
            'BackgroundColor','k','Margin',1);
    end
    title(ax,'1. Imagem e circulos aceites');
    hold(ax,'off');

    ax = nexttile; imshow(debug.redMask,'Parent',ax); title(ax,'2. Mascara vermelha');
    ax = nexttile; imshow(debug.blueMask,'Parent',ax); title(ax,'3. Mascara azul');
    ax = nexttile; imshow(debug.greenMask,'Parent',ax); title(ax,'4. Mascara verde');

    ax = nexttile;
    imshow(debug.blackMask,'Parent',ax); hold(ax,'on');
    for k = 1:numel(cups)
        plot(ax,cups(k).centroid(1),cups(k).centroid(2),'y+', ...
            'MarkerSize',12,'LineWidth',2);
    end
    title(ax,'5. Pixeis escuros e centros aceites');
    hold(ax,'off');

    ax = nexttile; hold(ax,'on');
    plot(ax,0,0,'k+','MarkerSize',10,'LineWidth',1.5);
    for k = 1:numel(cups)
        color = cupPlotColor(cups(k).color);
        xy = cups(k).topCenterBase(1:2);
        plot(ax,xy(1),xy(2),'o','Color',color, ...
            'MarkerFaceColor',color,'MarkerSize',8);
        text(ax,xy(1),xy(2),sprintf(' %s [%.3f, %.3f] m', ...
            cups(k).color,xy(1),xy(2)),'Color',color, ...
            'VerticalAlignment','bottom');
    end
    axis(ax,'equal'); grid(ax,'on');
    xlabel(ax,'X na base /px100 (m)'); ylabel(ax,'Y na base /px100 (m)');
    title(ax,sprintf('6. Centros projetados no plano z = %.3f m',topZ));
    hold(ax,'off');

    sgtitle(sprintf(['Detecao passo a passo | pixel do rebordo preto -> XY ' ...
        'no plano do topo (z = %.3f m)'],topZ));
end

function showProjectionGeometry(cups,topZ)
    calibrationFile = fullfile(fileparts(mfilename('fullpath')), ...
        'cameraCalibration.mat');
    data = load(calibrationFile,'T_base_camera','cameraToPinhole');
    T_base_pinhole = double(data.T_base_camera)/double(data.cameraToPinhole);
    cameraOrigin = T_base_pinhole(1:3,4);
    points = vertcat(cups.topCenterBase);

    padding = 0.05;
    xLimits = [min([cameraOrigin(1);points(:,1)])-padding, ...
        max([cameraOrigin(1);points(:,1)])+padding];
    yLimits = [min([cameraOrigin(2);points(:,2)])-padding, ...
        max([cameraOrigin(2);points(:,2)])+padding];
    if diff(xLimits) < 0.10, xLimits = xLimits+[-0.05 0.05]; end
    if diff(yLimits) < 0.10, yLimits = yLimits+[-0.05 0.05]; end

    figure('Name','Projecao geometrica: raio da camera e plano do copo');
    hold on;
    patch([xLimits(1) xLimits(2) xLimits(2) xLimits(1)], ...
        [yLimits(1) yLimits(1) yLimits(2) yLimits(2)], ...
        topZ*ones(1,4),[0.65 0.82 1.0], ...
        'FaceAlpha',0.20,'EdgeColor',[0.3 0.5 0.8]);
    plot3(cameraOrigin(1),cameraOrigin(2),cameraOrigin(3), ...
        'ks','MarkerFaceColor','k','MarkerSize',8);
    text(cameraOrigin(1),cameraOrigin(2),cameraOrigin(3), ...
        ' Camera/pinhole','VerticalAlignment','bottom');

    for k = 1:numel(cups)
        point = cups(k).topCenterBase;
        color = cupPlotColor(cups(k).color);
        plot3([cameraOrigin(1),point(1)], ...
            [cameraOrigin(2),point(2)], ...
            [cameraOrigin(3),point(3)],'--','Color',color,'LineWidth',1.5);
        plot3(point(1),point(2),point(3),'o', ...
            'Color',color,'MarkerFaceColor',color,'MarkerSize',7);
        text(point(1),point(2),point(3),[' ',cups(k).color]);
    end
    xlabel('X /px100 (m)'); ylabel('Y /px100 (m)'); zlabel('Z /px100 (m)');
    title('Cada pixel define um raio; o plano conhecido fixa a profundidade');
    axis equal; grid on; view(3); hold off;
end

function color = cupPlotColor(name)
    switch name
        case 'red'
            color = [1 0 0];
        case 'blue'
            color = [0 0.5 1];
        case 'green'
            color = [0 1 0];
        otherwise
            color = [1 1 1];
    end
end
