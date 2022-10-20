clear all;
rosshutdown
rosinit

%% Get image source
i = 0;
wherepath = what('DobotCameraImgs');  %find folder path 
saveimgpath = wherepath.path;
rgbSub = rossubscriber('/camera/color/image_raw');           % subscribe to usb_cam raw image. Change whatever is in brackets to directory found through rostopic list
% rgbSub = rossubscriber('/usb_cam/image_raw');
pause(2);

%% Take photo
figure(2);
imgIn = rgbSub.LatestMessage;
im = readImage(imgIn);
file_name = sprintf('test%d.png', i);
fullFileName = fullfile(saveimgpath, file_name);
imgName = [saveimgpath,'/test_',num2str(i),'.png'];
fprintf("test image %d saved \n",i)
pause(1);
%% Detect specific color

R = im(:,:,1) > 55;
G = im(:,:,2) < 35;
B = im(:,:,3) < 35;

RedPixels = R & G & B;

subplot(2,3,2);
imshow(R);
title('R'); 
subplot(3,3,3);
imshow(RedPixels);
title('Red'); 
subplot(2,1,2);
imshow(im);
title('im'); 
%% Get location of white area

% I1 = convertRGBtoGS(RedPixels); 
% I2 = convertGStoBW(I1,0.35); 
I2= RedPixels;
[y1,x1] = find(I2==0) ;   % get black pixels to reduce white background 
I = im(min(y1):max(y1),min(x1):max(x1),:) ;  % Get the only black part of the image 
I1 = convertRGBtoGS(I); 
I2 = convertGStoBW(I1,0.35); 
[y,x] = find(I2) ;   % get white pixels in image I 
x = x+min(x1) ; y = y+min(y1) ;   % white pxiels in original image 
imshow(im)
hold on
plot(x,y,'.r')

objectX = ((max(x) - min(x))/2)+min(x);
objectY = (max(y) - min(y))/2+min(y);

hold on
plot(objectX,objectY,'.b')



% Get the size of the input image
[rows, cols, channels] = size(im)

imgX = cols/2;
imgY = rows/2;

difX = -(imgX - objectX)
difY = imgY - objectY

hold on
plot(imgX,imgY,'.y')



%% RGB to GS

function [imgGS] = convertRGBtoGS(imgRGB)

% Get the size of the input image
[rows, cols, channels] = size(imgRGB)

% Create an empty matrix for the new greyscale image
imgGS = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        % greyscale value = 0.2989 * R + 0.5870 * G + 0.1140 * B
        imgGS(i,j) = 0.2989 * imgRGB(i,j,1) + 0.5870 * imgRGB(i,j,2) + 0.1140 * imgRGB(i,j,3);
    end
end

imgGS = uint8(imgGS);
imshow(imgGS);

end
%% GS to BW

function [imgBW] = convertGStoBW(imgGS, threshold)

% Get the size of the input image
[rows, cols, channels] = size(imgGS);

%create an empty matrix for the binary image
imgBW = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        if imgGS(i,j) <= threshold*256
            imgBW(i,j) = 1;
        end
    end
end

imgBW = logical(imgBW);
imshow(imgBW);

end