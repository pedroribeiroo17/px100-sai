function varargout = calibrationPX100(action, varargin)
% Ferramentas de calibracao e geometria da camera do PX100.
% Comandos: capture, calibrate, transform, validate, inspect, project,
% projectplane, originpixel.

    if nargin == 0 || isempty(action)
        error(['Indica um comando: capture, calibrate, transform, ' ...
            'validate, inspect, project, projectplane ou originpixel.']);
    end
    switch lower(char(action))
        case 'capture'
            captureImages(varargin{:});
            outputs = {};
        case 'calibrate'
            [cameraParams, estimationErrors] = calibrateIntrinsics(varargin{:});
            outputs = {cameraParams, estimationErrors};
        case 'transform'
            T_base_camera = getCameraTransform(varargin{:});
            outputs = {T_base_camera};
        case 'validate'
            validation = validateCalibration(varargin{:});
            outputs = {validation};
        case 'inspect'
            objects = inspectScene(varargin{:});
            outputs = {objects};
        case 'project'
            pointsBase = projectPixelsToObjectTop(varargin{:});
            outputs = {pointsBase};
        case 'projectplane'
            pointsBase = projectPixelsToHorizontalPlane(varargin{:});
            outputs = {pointsBase};
        case 'originpixel'
            originPixel = projectBaseOrigin(varargin{:});
            outputs = {originPixel};
        otherwise
            error('Comando desconhecido: %s', char(action));
    end
    if nargout > 0
        varargout = outputs(1:min(nargout, numel(outputs)));
    end
end


function pointsBase = projectPixelsToHorizontalPlane(pixelPoints,planeZ,verbose)
% Projeta pixeis para um plano horizontal z=planeZ, na base /px100.
% Usa apenas a calibracao guardada, sem consultar a pose de um objeto na cena.
    if nargin < 3 || isempty(verbose), verbose = true; end
    if size(pixelPoints,2) ~= 2 || isempty(pixelPoints)
        error('pixelPoints deve ser uma matriz N x 2 [x y].');
    end
    if ~isscalar(planeZ) || ~isfinite(planeZ)
        error('planeZ deve ser uma coordenada Z finita em metros.');
    end
    [calibrationFolder,~] = projectFolders();
    data = load(fullfile(calibrationFolder,'cameraCalibration.mat'));
    required = {'cameraParams','T_base_camera','cameraToPinhole'};
    if ~all(isfield(data,required))
        error('Executa primeiro calibrate, transform e validate.');
    end

    T_base_pinhole = double(data.T_base_camera)/double(data.cameraToPinhole);
    cameraOrigin = T_base_pinhole(1:3,4);
    cameraRotation = T_base_pinhole(1:3,1:3);
    undistorted = undistortPoints(double(pixelPoints),data.cameraParams);
    intrinsics = data.cameraParams.Intrinsics;
    focal = double(intrinsics.FocalLength);
    principal = double(intrinsics.PrincipalPoint);
    rays = [(undistorted(:,1)-principal(1))/focal(1), ...
        (undistorted(:,2)-principal(2))/focal(2),ones(size(pixelPoints,1),1)];

    pointsBase = zeros(size(pixelPoints,1),3);
    for k = 1:size(pixelPoints,1)
        direction = cameraRotation*rays(k,:)';
        if abs(direction(3)) < 1e-10
            error('O raio do pixel [%g %g] e paralelo ao plano z=%.4f.', ...
                pixelPoints(k,1),pixelPoints(k,2),planeZ);
        end
        distance = (planeZ-cameraOrigin(3))/direction(3);
        if distance <= 0
            error('O plano z=%.4f esta atras da camera para o pixel [%g %g].', ...
                planeZ,pixelPoints(k,1),pixelPoints(k,2));
        end
        pointsBase(k,:) = (cameraOrigin+distance*direction)';
        pointsBase(k,3) = planeZ;
    end
    if verbose
        fprintf('XYZ no referencial /px100 (m), plano horizontal z=%.4f:\n',planeZ);
        disp(pointsBase);
    end
end


function captureImages(numberOfImages, outputFolder)
    [calibrationFolder, projectFolder] = projectFolders();
    if nargin < 1 || isempty(numberOfImages), numberOfImages = 15; end
    if nargin < 2 || isempty(outputFolder)
        outputFolder = fullfile(calibrationFolder, 'images');
    end
    if numberOfImages < 3 || numberOfImages > 15 || numberOfImages ~= floor(numberOfImages)
        error('numberOfImages deve ser um inteiro entre 3 e 15.');
    end
    if ~exist(outputFolder, 'dir'), mkdir(outputFolder); end

    [client, sim] = connectRemote(projectFolder);
    chessHandle = sim.getObject('/chess');
    cameraHandle = sim.getObject('/Vision_sensor');
    originalPosition = asRow(sim.getObjectPosition(chessHandle, sim.handle_world));
    originalOrientation = asRow(sim.getObjectOrientation(chessHandle, sim.handle_world));

    % Cada linha: [dx dy dz dAlpha dBeta dGamma], metros e graus.
    poseOffsets = [ ...
         0.00  0.00 0.000   0    0    0; ...
        -0.02  0.00 0.055   0    8    0; ...
         0.02  0.00 0.055   0   -8    0; ...
         0.00 -0.02 0.055   8    0    0; ...
         0.00  0.02 0.055  -8    0    0; ...
        -0.02 -0.02 0.065   7    7    0; ...
         0.02  0.02 0.065  -7   -7    0; ...
        -0.02  0.01 0.065   5   -5   10; ...
         0.02 -0.01 0.065  -5    5  -10; ...
         0.00  0.01 0.070  10    0    5; ...
         0.00 -0.01 0.070 -10    0   -5; ...
         0.01  0.00 0.075   0   10   10; ...
        -0.01  0.00 0.075   0  -10  -10; ...
         0.00  0.01 0.090  10  -10   15; ...
         0.00 -0.01 0.090 -10   10  -15];

    cleanup = onCleanup(@()restoreScene(sim, chessHandle, ...
        originalPosition, originalOrientation));
    sim.setStepping(true);
    sim.startSimulation();
    fprintf('A capturar %d imagens do tabuleiro /chess.\n', numberOfImages);
    for k = 1:numberOfImages
        offset = poseOffsets(k,:);
        position = originalPosition + offset(1:3);
        orientation = originalOrientation + deg2rad(offset(4:6));
        sim.setObjectPosition(chessHandle, position, sim.handle_world);
        sim.setObjectOrientation(chessHandle, orientation, sim.handle_world);
        sim.step();
        [imageData, resolution] = sim.getVisionSensorImg(cameraHandle);
        image = formatCameraImage(imageData, resolution);
        filename = fullfile(outputFolder, sprintf('calib_%02d.png', k));
        imwrite(image, filename);
        fprintf('%02d/%02d guardada: %s\n', k, numberOfImages, filename);
        imshow(image);
        title(sprintf('Calibracao: imagem %d/%d', k, numberOfImages));
        drawnow;
    end
    clear cleanup
    clear sim client
    disp('Capturas concluidas. Executa calibrationPX100(''calibrate'').');
end


function [cameraParams, estimationErrors] = calibrateIntrinsics(imageFolder, squareSize)
    [calibrationFolder, ~] = projectFolders();
    if nargin < 1 || isempty(imageFolder), imageFolder = fullfile(calibrationFolder, 'images'); end
    if nargin < 2 || isempty(squareSize), squareSize = 0.0625; end
    if ~isfolder(imageFolder), error('Pasta nao encontrada: %s', imageFolder); end

    extensions = {'*.png','*.jpg','*.jpeg','*.tif','*.bmp'};
    files = struct([]);
    for k = 1:numel(extensions)
        files = [files; dir(fullfile(imageFolder, extensions{k}))]; %#ok<AGROW>
    end
    if isempty(files), error('Nao foram encontradas imagens em %s.', imageFolder); end
    names = fullfile({files.folder}, {files.name});
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(names);
    if nnz(imagesUsed) < 3
        error('O MATLAB detetou o tabuleiro em menos de 3 imagens.');
    elseif nnz(imagesUsed) < 10
        warning('Foram usadas apenas %d imagens; recomenda-se 10 a 20.', nnz(imagesUsed));
    end

    firstImage = imread(names{1});
    imageSize = [size(firstImage,1), size(firstImage,2)];
    squareSizeMillimeters = squareSize * 1000;
    worldPoints = generateCheckerboardPoints(boardSize, squareSizeMillimeters);
    [cameraParams, ~, estimationErrors] = estimateCameraParameters( ...
        imagePoints, worldPoints, 'ImageSize', imageSize, 'WorldUnits', 'mm');
    usedFiles = names(imagesUsed);
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');
    save(calibrationFile, 'cameraParams', 'estimationErrors', 'imagePoints', ...
        'worldPoints', 'boardSize', 'imagesUsed', 'usedFiles', ...
        'squareSize', 'squareSizeMillimeters', 'imageSize');
    fprintf('Calibracao guardada em %s\n', calibrationFile);
    fprintf('Imagens usadas: %d de %d\n', nnz(imagesUsed), numel(names));
    fprintf('Erro medio de reprojecao: %.4f px\n', ...
        mean(cameraParams.MeanReprojectionError));
    figure('Name','Erros da calibracao'); showReprojectionErrors(cameraParams);
    figure('Name','Extrinsecos por imagem'); showExtrinsics(cameraParams,'CameraCentric');
end


function T_base_camera = getCameraTransform(basePath, cameraPath)
    [calibrationFolder, projectFolder] = projectFolders();
    if nargin < 1 || isempty(basePath), basePath = '/px100'; end
    if nargin < 2 || isempty(cameraPath), cameraPath = '/Vision_sensor'; end
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');
    if ~isfile(calibrationFile), error('Executa primeiro o comando calibrate.'); end
    [client, sim] = connectRemote(projectFolder);
    baseHandle = sim.getObject(basePath);
    cameraHandle = sim.getObject(cameraPath);
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle, baseHandle));
    cameraPositionInBase = T_base_camera(1:3,4)';
    save(calibrationFile, 'T_base_camera', 'cameraPositionInBase', ...
        'basePath', 'cameraPath', '-append');
    fprintf('Transformacao camera -> base guardada em %s\n', calibrationFile);
    fprintf('Posicao [x y z] (m): [%.6f %.6f %.6f]\n', cameraPositionInBase);
    disp(T_base_camera);
    clear sim client
end


function validation = validateCalibration(image, simInterface)
    [calibrationFolder, ~] = projectFolders();
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');
    data = load(calibrationFile);
    [imagePoints, boardSize] = detectCheckerboardPoints(image);
    if isempty(imagePoints)
        validation = struct('valid',false,'errorPixels',NaN);
        disp('Tabuleiro nao detetado.'); return
    end
    if numel(boardSize) ~= 2 || any(boardSize < 4) || ...
            size(imagePoints,1) ~= prod(boardSize-1)
        error('Dimensoes inesperadas: [%d %d], pontos=%d.', ...
            boardSize(1),boardSize(2),size(imagePoints,1));
    end

    client = simInterface.get_connection();
    sim = client.require('sim');
    baseHandle = sim.getObject('/px100');
    chessHandle = sim.getObject('/chess');
    cameraHandle = sim.getObject('/Vision_sensor');
    currentCameraMatrix = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));
    difference = max(abs(currentCameraMatrix(:)-double(data.T_base_camera(:))));
    if difference > 1e-4
        fprintf('Pose da camera mudou (%.6g); atualizo T_base_camera.\n',difference);
        T_base_camera = currentCameraMatrix;
        cameraPositionInBase = currentCameraMatrix(1:3,4)';
        save(calibrationFile,'T_base_camera','cameraPositionInBase','-append');
    end
    T_base_chess = toHomogeneous(sim.getObjectMatrix(chessHandle,baseHandle));

    boardSide = 0.50;
    squareX = boardSide / boardSize(2);
    squareY = boardSide / boardSize(1);
    innerCols = boardSize(2)-1; innerRows = boardSize(1)-1;
    x = ((1:innerCols)-(innerCols+1)/2)*squareX;
    y = ((1:innerRows)-(innerRows+1)/2)*squareY;
    [X,Y] = meshgrid(x,y);
    localPoints = [X(:),Y(:),zeros(numel(X),1),ones(numel(X),1)]';
    basePoints = (T_base_chess*localPoints)'; basePoints = basePoints(:,1:3);

    T_camera_base = inv(double(currentCameraMatrix));
    axisMaps = cat(3,[1 0;0 1],[1 0;0 -1],[-1 0;0 1],[-1 0;0 -1], ...
        [0 1;1 0],[0 1;-1 0],[0 -1;1 0],[0 -1;-1 0]);
    bestError = inf; projected = []; bestAxisMap = [];
    for k = 1:size(axisMaps,3)
        map3 = zeros(3); map3(1:2,1:2) = axisMaps(:,:,k);
        map3(3,3) = det(axisMaps(:,:,k));
        T_pinhole_base = [map3,zeros(3,1);0 0 0 1]*T_camera_base;
        R = T_pinhole_base(1:3,1:3);
        [U,~,V] = svd(R); R = U*diag([1 1 det(U*V')])*V';
        pointsCamera = (R*basePoints'+T_pinhole_base(1:3,4))';
        if any(pointsCamera(:,3)<=0), continue; end
        candidate = worldToImage(data.cameraParams,R',T_pinhole_base(1:3,4)', ...
            basePoints,'ApplyDistortion',true);
        if any(~isfinite(candidate),'all'), continue; end
        d = hypot(candidate(:,1)-imagePoints(:,1)', ...
            candidate(:,2)-imagePoints(:,2)');
        err = sqrt(mean([min(d,[],1)'; min(d,[],2)].^2));
        if err < bestError
            bestError = err; projected = candidate; bestAxisMap = axisMaps(:,:,k);
        end
    end
    if isempty(projected), error('Nao foi possivel projetar os cantos do tabuleiro.'); end

    cameraToPinhole = eye(4);
    cameraToPinhole(1:2,1:2) = bestAxisMap;
    cameraToPinhole(3,3) = det(bestAxisMap);
    save(calibrationFile,'cameraToPinhole','bestAxisMap','-append');
    validation = struct('valid',true,'pass',bestError<2, ...
        'errorPixels',bestError,'boardSize',boardSize,'axisMap',bestAxisMap, ...
        'detectedPoints',imagePoints,'projectedPoints',projected);
    fprintf('Tabuleiro: %d x %d quadrados (%d x %d cantos internos)\n', ...
        boardSize(1),boardSize(2),innerRows,innerCols);
    fprintf('Espacamento inferido [x y]: [%.5f %.5f] m\n',squareX,squareY);
    fprintf('Erro medio de validacao: %.3f px\n',bestError);
    disp('Conversao de eixos do sensor para imagem:'); disp(bestAxisMap);
    if validation.pass, disp('Validacao passou (<2 px).');
    else, disp('Validacao acima de 2 px.'); end
    figure('Name','Validacao camera-base com tabuleiro'); imshow(image); hold on
    plot(imagePoints(:,1),imagePoints(:,2),'go','MarkerSize',7,'LineWidth',1.5);
    plot(projected(:,1),projected(:,2),'r+','MarkerSize',8,'LineWidth',1.5);
    legend('Cantos detetados','Cantos projetados');
    title(sprintf('Erro medio: %.3f px',bestError)); hold off
end


function objects = inspectScene(simInterface)
    [~,projectFolder] = projectFolders();
    if nargin < 1 || isempty(simInterface)
        [client,sim] = connectRemote(projectFolder);
    else
        client = simInterface.get_connection(); sim = client.require('sim');
    end
    baseHandle = sim.getObject('/px100');
    handles = sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0);
    if iscell(handles), handles = cell2mat(handles); end
    objects = repmat(struct('path','','positionBase',[NaN NaN NaN], ...
        'orientationBase',[NaN NaN NaN],'bboxMin',[NaN NaN NaN], ...
        'bboxMax',[NaN NaN NaN]),numel(handles),1);
    fprintf('%-48s %27s %27s %27s\n','Objeto','Origem [x y z] base (m)', ...
        'BBox min local (m)','BBox max local (m)');
    for k = 1:numel(handles)
        h = handles(k); path = sim.getObjectAlias(h,1);
        pos = asRow(sim.getObjectPosition(h,baseHandle));
        ori = asRow(sim.getObjectOrientation(h,baseHandle));
        mn = [NaN NaN NaN]; mx = mn;
        try
            mn = [getFloat(sim,h,sim.objfloatparam_objbbox_min_x), ...
                getFloat(sim,h,sim.objfloatparam_objbbox_min_y), ...
                getFloat(sim,h,sim.objfloatparam_objbbox_min_z)];
            mx = [getFloat(sim,h,sim.objfloatparam_objbbox_max_x), ...
                getFloat(sim,h,sim.objfloatparam_objbbox_max_y), ...
                getFloat(sim,h,sim.objfloatparam_objbbox_max_z)];
        catch
        end
        if iscell(path), path = path{1}; end
        objects(k) = struct('path',char(path),'positionBase',double(pos), ...
            'orientationBase',double(ori),'bboxMin',double(mn),'bboxMax',double(mx));
        fprintf('%-48s [%8.4f %8.4f %8.4f] [%8.4f %8.4f %8.4f] [%8.4f %8.4f %8.4f]\n', ...
            objects(k).path,objects(k).positionBase,objects(k).bboxMin,objects(k).bboxMax);
    end
    clear sim client
end


function pointsBase = projectPixelsToObjectTop(pixelPoints,objectPath,simInterface)
    if size(pixelPoints,2) ~= 2 || isempty(pixelPoints)
        error('pixelPoints deve ser uma matriz N x 2 [x y].');
    end
    [calibrationFolder,~] = projectFolders();
    calibrationFile = fullfile(calibrationFolder,'cameraCalibration.mat');
    data = load(calibrationFile);
    required = {'cameraParams','T_base_camera','cameraToPinhole'};
    if ~all(isfield(data,required))
        error('Executa primeiro os comandos calibrate, transform e validate.');
    end
    client = simInterface.get_connection(); sim = client.require('sim');
    baseHandle = sim.getObject('/px100');
    cameraHandle = sim.getObject('/Vision_sensor');
    objectHandle = sim.getObject(objectPath);
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));
    T_base_object = toHomogeneous(sim.getObjectMatrix(objectHandle,baseHandle));
    cameraPositionInBase = T_base_camera(1:3,4)';
    save(calibrationFile,'T_base_camera','cameraPositionInBase','-append');

    maxZ = getFloat(sim,objectHandle,sim.objfloatparam_objbbox_max_z);
    R_object = T_base_object(1:3,1:3);
    planePoint = T_base_object(1:3,4)+R_object(:,3)*maxZ;
    planeNormal = R_object(:,3);
    T_base_pinhole = double(T_base_camera)/double(data.cameraToPinhole);
    cameraOrigin = T_base_pinhole(1:3,4);
    cameraRotation = T_base_pinhole(1:3,1:3);

    uv = undistortPoints(double(pixelPoints),data.cameraParams);
    intrinsics = data.cameraParams.Intrinsics;
    focal = double(intrinsics.FocalLength);
    principal = double(intrinsics.PrincipalPoint);
    rays = [(uv(:,1)-principal(1))/focal(1), ...
        (uv(:,2)-principal(2))/focal(2),ones(size(uv,1),1)];
    pointsBase = zeros(size(pixelPoints,1),3);
    for k = 1:size(pixelPoints,1)
        direction = cameraRotation*rays(k,:)';
        denominator = dot(planeNormal,direction);
        if abs(denominator)<1e-10, error('Raio paralelo ao plano superior de %s.',objectPath); end
        distance = dot(planeNormal,planePoint-cameraOrigin)/denominator;
        if distance<=0, error('Plano de %s atras da camera.',objectPath); end
        pointsBase(k,:) = (cameraOrigin+distance*direction)';
    end
    fprintf('XYZ no referencial /px100 (m), plano maxZ de %s:\n',objectPath);
    disp(pointsBase);
end


function originPixel = projectBaseOrigin(simInterface)
    [calibrationFolder,~] = projectFolders();
    data = load(fullfile(calibrationFolder,'cameraCalibration.mat'));
    required = {'cameraParams','cameraToPinhole'};
    if ~all(isfield(data,required))
        error('Executa primeiro os comandos calibrate e validate.');
    end
    client = simInterface.get_connection(); sim = client.require('sim');
    baseHandle = sim.getObject('/px100');
    cameraHandle = sim.getObject('/Vision_sensor');
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));
    T_base_pinhole = double(T_base_camera)/double(data.cameraToPinhole);
    T_pinhole_base = inv(T_base_pinhole);
    R = T_pinhole_base(1:3,1:3);
    [U,~,V] = svd(R); R = U*diag([1 1 det(U*V')])*V';
    t = T_pinhole_base(1:3,4)';
    originCamera = R*[0;0;0]+t';
    if originCamera(3) <= 0
        error('A origem da base esta atras da camera e nao pode ser projetada.');
    end
    originPixel = worldToImage(data.cameraParams,R',t,[0 0 0], ...
        'ApplyDistortion',true);
end


function [client,sim] = connectRemote(projectFolder)
    addpath(projectFolder);
    jarPath = fullfile(projectFolder,'jeromq-0.5.2.jar');
    if ~isfile(jarPath), error('Biblioteca ZeroMQ nao encontrada: %s',jarPath); end
    try
        java.lang.Class.forName('org.zeromq.ZMsg');
    catch
        javaaddpath(jarPath);
    end
    client = RemoteAPIClient();
    sim = client.require('sim');
end


function [calibrationFolder,projectFolder] = projectFolders()
    calibrationFolder = fileparts(mfilename('fullpath'));
    projectFolder = fileparts(calibrationFolder);
end


function restoreScene(sim,chessHandle,position,orientation)
    try
        sim.setObjectPosition(chessHandle,position,sim.handle_world);
        sim.setObjectOrientation(chessHandle,orientation,sim.handle_world);
        sim.stopSimulation();
    catch
    end
end


function image = formatCameraImage(imageData,resolution)
    width = resolution{1}; height = resolution{2};
    image = flip(permute(reshape(imageData,3,width,height),[3 2 1]),1);
    image = normalizeImage(image);
end


function image = normalizeImage(image)
    if isa(image,'uint8'), return; end
    image = double(image);
    if ~isempty(image) && max(image(:))<=1, image = image*255; end
    image = uint8(min(max(round(image),0),255));
end


function T = toHomogeneous(matrixData)
    if iscell(matrixData), matrixData = cell2mat(matrixData); end
    T = [reshape(double(matrixData),4,3)';0 0 0 1];
end


function row = asRow(value)
    if iscell(value), value = cell2mat(value); end
    row = double(value(:)');
end


function value = getFloat(sim,handle,parameter)
    value = sim.getObjectFloatParam(handle,parameter);
    if iscell(value), value = cell2mat(value); end
    value = double(value);
end
