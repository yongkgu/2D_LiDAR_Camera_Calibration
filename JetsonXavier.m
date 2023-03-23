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
% codegen -config cfg Detect_test -args inputArgs -report
codegen -config cfg yolov3Detect -args inputArgs -report


%% run

exe = [obj.workspaceDir '/yolov3Detect.elf'];
% exe = ['/home/wecar/remoteBuildDir/MATLAB_ws/R2023a/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/yolov3Detect.elf'];
pid = obj.runExecutable(exe);
%% run Detect_test

% exe = [obj.workspaceDir '/Detect_test.elf'];
exe = ['/home/wecar/remoteBuildDir/MATLAB_ws/R2023a/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/Detect_test.elf'];
pid = obj.runExecutable(exe);
%% stop

killApplication(obj,exe)  
killProcess(obj,pid)

%% modbus read
m = modbus('tcpip','192.168.0.81',1478)
%% sample rate = 90 Hz
 
tic;
data = zeros(3000,4);
count=1;
while toc < 10
data(count,1) = read(m, 'holdingregs',31);
data(count,2) =read(m, 'holdingregs',32);
data(count,3) =read(m, 'holdingregs',33);
data(count,4) =read(m, 'holdingregs',34);
% disp(data)
count=count+1;

end

%% Generate File and Read File
tic;
getFile(obj,'/home/wecar/remoteBuildDir/MATLAB_ws/R2022b/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/idx.bin')
fid = fopen('idx.bin', 'rb');
data = fread(fid,'uint8');
disp(data)
fclose(fid);
toc


%% Modbus test
% codegen -config cfg Modbus_Client
codegen -config cfg Modbus
%% Modbus run
exe = ['/home/wecar/remoteBuildDir/MATLAB_ws/R2023a/C/Users/Admin/Desktop/YONG/LiDAR/RPLidar/Jetson_AGX_Xavier/Modbus.elf'];
runApplication(obj,'Modbus');
%% stop
killApplication(obj,'Modbus');