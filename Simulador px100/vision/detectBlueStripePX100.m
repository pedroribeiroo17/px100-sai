function stripe = detectBlueStripePX100(image)
% Segmenta a faixa azul e estima os dois cantos da aresta inferior.
% Devolve coordenadas de pixeis [x y], origem no canto superior esquerdo.

    hsvImage = rgb2hsv(image);
    hue = hsvImage(:,:,1);
    saturation = hsvImage(:,:,2);
    value = hsvImage(:,:,3);
    mask = hue>=0.52 & hue<=0.72 & saturation>=0.35 & value>=0.20;
    mask = bwareaopen(mask,10);
    mask = imclose(mask,strel('rectangle',[3 15]));
    mask = bwareafilt(mask,1);
    if ~any(mask,'all')
        stripe = [];
        return
    end

    [y,x] = find(mask);
    xy = [x,y];
    centered = xy-mean(xy,1);
    [vectors,~] = eig(cov(centered));
    [~,order] = sort(var(centered*vectors,0,1),'descend');
    longAxis = vectors(:,order(1));
    shortAxis = [-longAxis(2);longAxis(1)];
    axesMatrix = [longAxis,shortAxis];
    projected = centered*axesMatrix;
    lower = min(projected,[],1);
    upper = max(projected,[],1);
    localCorners = [lower(1),lower(2); upper(1),lower(2); ...
        upper(1),upper(2); lower(1),upper(2)];
    imageCorners = localCorners*axesMatrix'+mean(xy,1);
    edgeA = imageCorners(1:2,:);
    edgeB = imageCorners(3:4,:);
    if mean(edgeA(:,2))>mean(edgeB(:,2)), corners=edgeA; else, corners=edgeB; end
    [~,leftToRight] = sort(corners(:,1));
    stripe.mask = mask;
    stripe.lowerCorners = corners(leftToRight,:);
end
