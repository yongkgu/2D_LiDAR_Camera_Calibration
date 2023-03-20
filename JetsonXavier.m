% CheckGpuInstall
gpucoderSetup
%% Connect Jetson AGX Xavier
clear; clc;
obj = jetson('192.168.0.81','wecar','wecar');
load('pretrainedYOLOv3Detector.mat');
matFile = 'pretrainedYOLOv3Detector.mat';

 
% obj.workspaceDir
envCfg = coder.gpuEnvConfig('jetson');
envCfg.BasicCodegen = 1;
envCfg.Quiet = 1;
envCfg.HardwareObject = obj;
coder.checkGpuInstall(envCfg);

% *Generate CUDA Code for the Target Using GPU Coder*

cfg = coder.gpuConfig('exe');
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.Hardware.BuildDir = '~/remoteBuildDir';
cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
cfg.BuildConfiguration = 'Faster Builds';
cfg.DeepLearningConfig = coder.DeepLearningConfig('TargetLibrary','cudnn');

disp('Target Check')
%% generate CUDA code

% inputArgs = {ones(608,608,3,'uint8'),coder.Constant(matFile)};
inputArgs = {coder.Constant(matFile)};

codegen -config cfg yolov3Detect -args inputArgs -report
% hwobj.putFile('gpio_test.py', hwobj.workspaceDir);

%% run

% exe = [obj.workspaceDir '/yolov3Detect.elf'];
exe = ['/home/wecar/remoteBuildDir/MATLAB_ws/R2022b/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/yolov3Detect.elf'];
pid = obj.runExecutable(exe);
%% 
tic;
getFile(obj,'/home/wecar/remoteBuildDir/MATLAB_ws/R2022b/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/idx.bin')
fid = fopen('idx.bin', 'rb');
data = fread(fid,'uint8');
disp(data)
fclose(fid);
toc
%% stop

killApplication(obj,exe)
killProcess(obj,pid)
%% 

cam = webcam(obj,1,'640x480')

%% 
% detector = yolov3ObjectDetector('darknet53-coco');
% matFile = 'pretrainedYOLOv3Detector.mat';
% save(matFile,'detector');
%% 
if size(bboxes) == [0 4]
    disp(1)
else
    disp(2)
end
