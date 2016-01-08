%% Initialization
clear ; close all; clc

imageL1=imread('im0.png');
imageR1=imread('im1.png');

imageL=rgb2gray(imageL1);
imageR=rgb2gray(imageR1);

cam0=[999.421 0 294.182; 0 999.421 252.932; 0 0 1];
cam1=[999.421 0 326.96; 0 999.421 252.932; 0 0 1];
doffs=32.778;
baseline=193.001;
width=741;
height=497;
ndisp=70;
neighbourhood=1;

dmin=disparity(imageL,imageR,'DisparityRange',[0 80],'BlockSize',5);

% COST_MAX=256*256*(2*neighbourhood+1)*(2*neighbourhood+1)*3.0;
% dmin=zeros(height,width);
% costMin=ones(height,width)*COST_MAX;
%
% for d=0:ndisp
%     fprintf('%d',d);
%     costM=double(imageR(:,1:width-d))-double(imageL(:,d+1:width));
%     costM=costM.*costM;
%
%     for i=neighbourhood+1:height-neighbourhood-d
%         for j=neighbourhood+1:width-neighbourhood-d
%             startX=j-neighbourhood;
%             startY=i-neighbourhood;
%             endX=j+neighbourhood;
%             endY=i+neighbourhood;
%
%             cost=double(0);
%             for y=startY:endY
%                 for x=startX:endX
%                     cost=cost+costM(y,x);
%                 end
%             end
%
%             if(cost<costMin(i,j))
%                 costMin(i,j)=cost;
%                 dmin(i,j)=d;
%             end
%         end
%     end
% end

z=cam0(1,1)*baseline./(dmin+doffs);

% Pose estimation

initialX=120;
x=initialX;
% cost=zeros(300,1);
% grad=zeros(300,1);
%
% for i=1:300;
%     [cost(i), grad(i)] = costFunction(i, z, imageL, imageR);
% end

[cost, grad] = costFunction(initialX, z, imageL, imageR);

fprintf('Cost at initial theta (zeros): %f\n', cost);
fprintf('Gradient at initial theta (zeros): \n');
fprintf(' %f \n', grad);

numIter=300;
alpha=0.3;
Jhist=zeros(numIter,1);
Jhist(1)=cost;
% x=x+1;
x=x+sqrt(cost)/grad;

[cost, grad] = costFunction(x, z, imageL, imageR);
% for i=1:numIter
i=2;
while(cost>0.6 && i<=numIter)
    [cost, grad] = costFunction(x, z, imageL, imageR);
    % grad=cost-Jhist(i-1);
    % x=x-alpha*grad;
    x=x+sqrt(cost)/grad;
    Jhist(i)=cost;
    i=i+1;
end

x

% zNew=cam0(1,1)*x./(dmin+doffs);
% err=z-zNew;
% err=err.*err;
% sum(sum(err))

dmin=dmin/ndisp;

% Display the left image
subplot(2, 2, 1);
imshow(imageL);
title('Left');

% Display the right image
subplot(2, 2, 2);
imshow(imageR);
title('Right');

subplot(2, 2, 3);
%imagesc(dmin);
imshow(dmin);
title('Depth');

subplot(2, 2, 4);
plot(1:numIter,Jhist);
% plot(initialX:300,grad(initialX:300));

pause;