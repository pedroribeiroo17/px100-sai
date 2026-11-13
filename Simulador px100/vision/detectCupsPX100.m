function [cups,debug] = detectCupsPX100(image)
% Deteta copos vermelhos, azuis e verdes pelo corpo e pelo rebordo preto.
% centroid devolve o centro do circulo preto em pixeis [x y].

    if size(image,3) ~= 3
        error('A imagem da camera tem de ser RGB.');
    end

    rgb = im2uint8(image);
    hsvImage = rgb2hsv(rgb);
    hue = hsvImage(:,:,1);
    saturation = hsvImage(:,:,2);
    value = hsvImage(:,:,3);
    gray = im2gray(rgb);
    blackMask = value < 0.24;
    debug = struct('redMask',false(size(value)), ...
        'blueMask',false(size(value)),'greenMask',false(size(value)), ...
        'blackMask',blackMask);

    colors = {'red','blue','green'};
    template = struct('color','','centroid',[],'bodyCentroid',[], ...
        'boundingBox',[],'area',0,'radius',0,'score',0, ...
        'topCenterBase',[],'graspTargetBase',[]);
    cups = repmat(template,0,1);

    for colorIndex = 1:numel(colors)
        switch colors{colorIndex}
            case 'red'
                colorMask = (hue <= 0.055 | hue >= 0.94) & ...
                    saturation >= 0.30 & value >= 0.12;
            case 'blue'
                colorMask = hue >= 0.50 & hue <= 0.75 & ...
                    saturation >= 0.30 & value >= 0.12;
            case 'green'
                colorMask = hue >= 0.18 & hue <= 0.46 & ...
                    saturation >= 0.30 & value >= 0.12;
        end

        colorMask = imopen(colorMask,strel('disk',2,0));
        colorMask = imclose(colorMask,strel('disk',4,0));
        colorMask = imfill(colorMask,'holes');
        colorMask = bwareaopen(colorMask,40);
        debug.([colors{colorIndex},'Mask']) = colorMask;
        components = bwconncomp(colorMask);
        stats = regionprops(components,'Area','Centroid','BoundingBox');
        [~,componentOrder] = sort([stats.Area],'descend');

        % Comeca pela maior mancha desta cor e so aceita uma ROI com rebordo.
        for orderIndex = 1:numel(componentOrder)
            componentIndex = componentOrder(orderIndex);
            box = double(stats(componentIndex).BoundingBox);
            width = box(3);
            height = box(4);
            smallSide = min(width,height);
            aspect = width/max(height,eps);
            if stats(componentIndex).Area < 40 || smallSide < 10 || ...
                    aspect < 0.40 || aspect > 2.5
                continue
            end

            [center,radius,score,found] = findBlackCircle( ...
                gray,blackMask,box,double(stats(componentIndex).Centroid));
            if ~found
                continue
            end

            cup = template;
            cup.color = colors{colorIndex};
            cup.centroid = center;
            cup.bodyCentroid = double(stats(componentIndex).Centroid);
            cup.boundingBox = box;
            cup.area = double(stats(componentIndex).Area);
            cup.radius = radius;
            cup.score = score;
            cups(end+1,1) = cup; %#ok<AGROW>
            break
        end
    end
end

function [bestCenter,bestRadius,bestScore,found] = findBlackCircle( ...
    gray,blackMask,box,bodyCenter)

    imageHeight = size(gray,1);
    imageWidth = size(gray,2);
    pad = 0.30*max(box(3),box(4));
    x1 = max(1,floor(box(1)-pad));
    y1 = max(1,floor(box(2)-pad));
    x2 = min(imageWidth,ceil(box(1)+box(3)+pad));
    y2 = min(imageHeight,ceil(box(2)+box(4)+pad));
    grayCrop = gray(y1:y2,x1:x2);
    blackCrop = blackMask(y1:y2,x1:x2);
    bodyCenterCrop = bodyCenter-[x1-1,y1-1];

    smallSide = min(box(3),box(4));
    radiusMin = max(4,floor(0.22*smallSide));
    radiusMax = max(radiusMin+2,ceil(0.75*smallSide));
    radiusMax = min(radiusMax,floor(min(size(grayCrop))/2));
    if radiusMax <= radiusMin
        bestCenter = []; bestRadius = 0; bestScore = 0; found = false;
        return
    end

    [centers,radii,metrics] = imfindcircles(grayCrop, ...
        [radiusMin radiusMax],'ObjectPolarity','dark', ...
        'Sensitivity',0.92,'EdgeThreshold',0.10);

    bestCenter = [];
    bestRadius = 0;
    bestScore = -inf;
    found = false;
    maxCenterDistance = 0.50*max(box(3),box(4));

    for k = 1:size(centers,1)
        centerCrop = centers(k,:);
        distanceToBody = norm(centerCrop-bodyCenterCrop);
        if distanceToBody > maxCenterDistance
            continue
        end
        blackCoverage = circleBlackCoverage(blackCrop,centerCrop,radii(k));
        if blackCoverage < 0.12
            continue
        end
        circleConfidence = min(max(double(metrics(k)),0),1);
        centerConfidence = 1-distanceToBody/max(maxCenterDistance,1);
        score = 0.45*circleConfidence+0.40*blackCoverage+ ...
            0.15*centerConfidence;
        if score > bestScore
            bestCenter = centerCrop+[x1-1,y1-1];
            bestRadius = double(radii(k));
            bestScore = score;
            found = true;
        end
    end

    % Se a Hough nao encontrar o aro, usa o centroide dos pixeis pretos
    % dentro da caixa do copo como alternativa simples.
    if ~found
        closedBlack = imclose(blackCrop,strel('disk',2,0));
        components = bwconncomp(closedBlack);
        blackStats = regionprops(components,'Area','Centroid','BoundingBox');
        bestFallbackScore = -inf;
        for k = 1:numel(blackStats)
            blackBox = double(blackStats(k).BoundingBox);
            blackWidth = blackBox(3);
            blackHeight = blackBox(4);
            blackAspect = min(blackWidth,blackHeight)/max(blackWidth,blackHeight);
            centerCrop = double(blackStats(k).Centroid);
            distanceToBody = norm(centerCrop-bodyCenterCrop);
            if blackStats(k).Area < 8 || blackAspect < 0.45 || ...
                    distanceToBody > maxCenterDistance
                continue
            end
            centerConfidence = 1-distanceToBody/max(maxCenterDistance,1);
            areaFraction = double(blackStats(k).Area)/ ...
                max(blackWidth*blackHeight,1);
            areaConfidence = min(areaFraction/0.30,1);
            score = 0.45*blackAspect+0.35*centerConfidence+ ...
                0.20*areaConfidence;
            if score > bestFallbackScore
                bestCenter = centerCrop+[x1-1,y1-1];
                bestRadius = mean([blackWidth,blackHeight])/2;
                bestScore = score;
                bestFallbackScore = score;
                found = true;
            end
        end
    end
end

function coverage = circleBlackCoverage(blackMask,center,radius)
    [x,y] = meshgrid(1:size(blackMask,2),1:size(blackMask,1));
    distance = hypot(x-center(1),y-center(2));
    band = abs(distance-radius) <= max(2,0.15*radius);
    coverage = nnz(blackMask & band)/max(nnz(band),1);
end
