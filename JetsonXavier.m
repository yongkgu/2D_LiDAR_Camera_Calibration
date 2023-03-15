obj = jetson('192.168.0.81','wecar','wecar');
%% 
cam = webcam(obj);
%% 
envCfg = coder.gpuEnvConfig('jetson');
envCfg.BasicCodegen = 1;
envCfg.Quiet = 1;
envCfg.HardwareObject = obj;
coder.checkGpuInstall(envCfg);

%% 
count=1;
tic;
while toc < 100
% for count=1:13

    img = snapshot(cam);
    
    count = count+1;
  
end 
%% 
tic;
img = snapshot(cam);
toc