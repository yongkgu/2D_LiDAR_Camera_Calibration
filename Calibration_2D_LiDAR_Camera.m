% *Connect Camera and RPLiDAR*

webcamlist
%%
% Connect Camera Sensor
cam = webcam("Intel(R) RealSense(TM) Depth Camera 415  RGB");
cam.Resolution = "1280x720"; % 640x480
preview(cam);
%%
% Connect LiDAR Sensor
hardwarex_init;
pRPLIDAR = CreateRPLIDAR();

[result]  = ConnectRPLIDAR(pRPLIDAR, 'RPLIDAR0.txt');
disp(result)
% *Create Checkerboard*

checkerboard_img = checkerboard(246,5,9);
checkerboard_img = checkerboard_img(:,1:2214);
imshow(checkerboard_img)
% *Read Data*

img = snapshot(cam);

Point_number = 2000;
alldistances = zeros([1 Point_number]);
allangles = zeros([1 Point_number]);

for count=1:Point_number
    [~, distance, angle, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
    alldistances(count) = distance;
    allangles(count) = angle;
end

scan = lidarScan(alldistances, allangles);
plot(scan);

% plot(scan.Cartesian(:,1),scan.Cartesian(:,2));
% *Load Data*

% The number of data is 19.
load("CalibrationData.mat");
load("CameraParams.mat");
load("pts_.mat");
% *Estimate Camera Parameters*

%image dataStore
imgDS = imageDatastore("C:\Users\Admin\Desktop\YONG\LiDAR\RPLidar\Calibration\Images\","IncludeSubfolders",true);

numImages = 19;
files = cell(1, numImages);
for i = 1:numImages
    files{i} = imgDS.Files{i};
end

% Display one of the calibration images
magnification = 100;
I = imread(files{1});
figure; imshow(I, 'InitialMagnification', magnification);
title('One of the Calibration Images');
%%
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 42; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);

% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');
%%
imOrig = imread("im02.bmp");
figure; imshow(imOrig);
title("Input Image","FontSize",15);

[im, newOrigin] = undistortImage(imOrig, cameraParams, OutputView = "full");
figure; imshow(im);
title("Undistorted Image","FontSize",15);
% *Scan Data Pre-processing*

scan = scan20;

figure(1);plot(scan);
title('All Data')
idx = find(scan.Ranges > 1.1 & scan.Ranges < 1.95 & ...
    scan.Angles > deg2rad(-20) & scan.Angles < deg2rad(32) );

% On the Checkerboard Data // Pf = [X,Z,1]'
pts20 =  [scan.Cartesian(idx,1),scan.Cartesian(idx,2),ones(numel(idx),1)]';

plot(scan)
hold on
plot(scan.Cartesian(idx,1),scan.Cartesian(idx,2),'.r')
% legend('All data','On the Checkerboard Data','Location','southeast')
title('Pre-Processed Data')
hold off
%%


%% step1. get camera calibration data and get Norm vector about each plane
cameraPlanes=[];

for i=1:19
     rc = cameraParams.PatternExtrinsics(i,1).R;
     tc = cameraParams.PatternExtrinsics(i,1).Translation;

     plane = -rc(:,3) * dot(rc(:,3)', tc); % = N 
     plane = plane./1000; % in mm not m and from camera to plane not the other way around -PLANE???
     cameraPlanes=[cameraPlanes,plane];


end
Nci = cameraPlanes;

%% step2. read laserdata and get initial estimation
stringlsBase='pts';
Nc = [];
Lpts = [];

for i = 1:19
    newls = eval([stringlsBase, num2str(i)]);
     newls(3,:) = newls(1,:);
     newls(1,:) = -newls(2,:);
     newls(2,:) = newls(3,:);
     newls(3,:) = 1;
    newmatchCP = repmat(Nci(:,i),1,size(newls,2)); % camera planes
    Lpts = [Lpts, newls];
    Nc = [Nc, newmatchCP];
end

[delta,phi] =getinitest(Lpts,Nc)



%% step5. Laser points into Image


% get rotation vector
phiinv=inv(phi);
 
% Get points
for i=1:19
stringlsBase='pts';
pts = eval([stringlsBase, num2str(i)]);
% pts(3,:) = pts(2,:);
% pts(2,:) = 0;
%
pts(3,:) = pts(1,:);
pts(1,:) = -pts(2,:);
pts(2,:) = 0;

Lpts_test= pts;

% change to mm (camera parameters in mm)
Lpts_test=Lpts_test.*1000;
Delta=delta.*1000;
% Delta=[0.3651, +450 , 0.12]';

% apply laser to camera transformation
%Cpts=phiinv*Lpts_test+repmat(Delta,1,size(Lpts_test,2)); 
Cpts = phi\Lpts_test + repmat(-Delta,1,size(Lpts_test,2));
xc=Cpts(1,:);
yc=Cpts(2,:);
zc=Cpts(3,:);

%normalise over Z (in this frame);
a=xc./zc;
b=yc./zc;
% add distortion
r = sqrt(a.^2 + b.^2);
k = [cameraParams.RadialDistortion cameraParams.TangentialDistortion 0];
alpha = cameraParams.Skew;
ad = a.*(1 + k(1).*r.^2 + k(2).*r.^4 + k(5).*r.^6) +  2.*k(3).*a.*b + k(4).*(r.^2 + 2.*a.^2);
bd = b.*(1 + k(1).*r.^2 + k(2).*r.^4 + k(5).*r.^6) +  k(3).*(r.^2 + 2.*b.^2) + 2.*k(4).*a.*b;

% image coordinates
f = cameraParams.FocalLength;
c = cameraParams.PrincipalPoint;
x = f(1).*(ad + alpha.*bd) + c(1); % add 1 for matlab coords
y = f(2).*bd + c(2); % add 1 for matlab coords

% img = imread('result.png');
stringlsBase='im';
img = imread([stringlsBase, num2str(i),'.bmp']);
figure;
imshow(img);
hold on 
plot(round(x(1,:)),round(y(1,:)),'.r','MarkerSize',20)

hold off
end
%%
% PHI_ANS =[[0.9995, 0.0165, -0.0268]; [0.0154,-0.9990,-0.0421]; [-0.0275,0.0417,-0.9988]]
% PHI_ANS_M = -PHI_ANS
% DELTA_ANS = [-27.3456;-24.4341;-100.7541]
% H3 = PHI_ANS_M\DELTA_ANS
% 
% DEL = PHI_ANS_M*H3

h_ANS = [0.9995;0.0275;24.9401327624321;0.0165;0.0417;-19.7592977939675;-0.0268; -0.9988;-102.386780505582]
