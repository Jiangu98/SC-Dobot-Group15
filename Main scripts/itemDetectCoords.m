   function itemcoords = itemDetectCoords()
    %Setup
    workspace;

    donotmasktopthreshold = 90;
    maskbottom = 160;
    deleteitemsize = 100;
    framewidth = 0.19; % in Metres
    dobotoutsideframe = 0.24; % in Metres, distance out of top of frame

    % Get image from camera
    i = 0;
    wherepath = what('DobotCameraImgs');  %find folder path 
    saveimgpath = wherepath.path;
    rgbSub = rossubscriber('/camera/color/image_raw');           % subscribe to usb_cam raw image. Change whatever is in brackets to directory found through rostopic list
    pause(2);

    figure(2);
    imgIn = rgbSub.LatestMessage;
    im = readImage(imgIn);
    image_h = imshow(readImage(rgbSub.LatestMessage)); 
    image_h.CData = readImage(rgbSub.LatestMessage);
    [rgb_image, ~] = readImage(imgIn);

    [rows, cols, ~] = size(rgb_image);
    frame_realworld_height = framewidth * (rows / cols);

    % Separate the RGB bands
    redarea = rgb_image(:, :, 1);
    greenarea = rgb_image(:, :, 2);
    bluearea = rgb_image(:, :, 3);
    
    % Display each band of colour
    subplot(3, 3, 1);
    imshow(redarea);
    title('Reds');
    subplot(3, 3, 2);
    imshow(greenarea);
    title('Greens');
    subplot(3, 3, 3);
    imshow(bluearea);
    title('Blues');

    % Apply colour band masks based on thresholds 
    notredmask = (redarea >= 0) & (redarea <= donotmasktopthreshold);
    notgreenmask = (greenarea >= 0) & (greenarea <= donotmasktopthreshold);
    notbluemask = (bluearea >= 0) & (bluearea <= donotmasktopthreshold);

    redmask = (redarea >= maskbottom) & (redarea <= 255);
    greenmask = (greenarea >= maskbottom) & (greenarea <= 255);
    bluemask = (bluearea >= maskbottom) & (bluearea <= 255);

    % Combine !masks with masks to increase accuracy
    notredmask = uint8(notredmask | greenmask | bluemask);
    notgreenmask = uint8(notgreenmask | redmask | bluemask);
    notbluemask = uint8(notbluemask | redmask | greenmask);

    % Eliminate objects smaller than x number of pixels
    notredmask = uint8(bwareaopen(notredmask, deleteitemsize));
    notgreenmask = uint8(bwareaopen(notgreenmask, deleteitemsize));
    notbluemask = uint8(bwareaopen(notbluemask, deleteitemsize));

    % Calculate the inclusive masks and imshow
    justreditems = uint8(notgreenmask & notbluemask & not(notredmask));% show just red
    justreditems = uint8(bwareaopen(justreditems, deleteitemsize)); % more filtering
    subplot(3, 3, 4);
    imshow(justreditems, []); %show img
    title('Red objects only');

    justgreenitems = uint8(notredmask & notbluemask & not(notgreenmask));% show just green
    justgreenitems = uint8(bwareaopen(justgreenitems, deleteitemsize)); % more filtering
    subplot(3, 3, 5);
    imshow(justgreenitems, []); %show img
    title('Green Objects only');

    justblueitems = uint8(notredmask & notgreenmask & not(notbluemask)); % show just blue
    justblueitems = uint8(bwareaopen(justblueitems, deleteitemsize)); % more filtering
    subplot(3, 3, 6);
    imshow(justblueitems, []); %show img
    title('Blue Objects only');
    
    % Get the centre postion and average intensity of all red items
    
    figure(3);
    red_components = bwconncomp(justreditems); % Finds connected components in binary image
    item_properties = regionprops(red_components, (redarea - bluearea - 
        ... greenarea) .* justblueitems, {'MeanIntensity', 'Centroid'}); % measures the properties of each region
    centroids = [item_properties.Centroid];
    mid_x = centroids(1:2:end - 1);
    mid_y = centroids(2:2:end);

    [~, idx] = max([item_properties.MeanIntensity]);

    redx = mid_x(idx)
    redy = mid_y(idx);

    % Show the reddest object centre
    subplot(3, 1, 1);
    imshow(redarea);
    hold on;
    plot(redx, redy, 'or')
    title('Most Red Object')

    % Get the centre postion and average intensity of all green items
    green_components = bwconncomp(justgreenitems); 
    item_properties = regionprops(green_components, 
       ...(greenarea - redarea - bluearea) .* justblueitems, {'MeanIntensity', 'Centroid'});
    centroids = [item_properties.Centroid];
    mid_x = centroids(1:2:end - 1);
    mid_y = centroids(2:2:end);

    [~, idx] = max([item_properties.MeanIntensity]);

    greenx = mid_x(idx);
    greeny = mid_y(idx);

    % Show the greenest object centre
    subplot(3, 1, 2);
    imshow(greenarea);
    hold on;
    plot(greenx, greeny, 'or')
    title('Most Green Object')

    % Get the centre postion and average intensity of all blue items
    blue_components = bwconncomp(justblueitems);
    item_properties = regionprops(blue_components, (bluearea - redarea - 
        ...greenarea) .* justblueitems, {'MeanIntensity', 'Centroid'});
    centroids = [item_properties.Centroid];
    mid_x = centroids(1:2:end - 1);
    mid_y = centroids(2:2:end);

    [~, idx] = max([item_properties.MeanIntensity]);

    bluex = mid_x(idx);
    bluey = mid_y(idx);

    % Show the bluest object centre
    subplot(3, 1, 3);
    imshow(bluearea);
    hold on;
    plot(bluex, bluey, 'or');
    title('Most Blue Object');

    %% Get position of objects in frame in realworld distance relative to top left
    
    [rows, cols, ~] = size(im);
    imgX = cols/2
    imgY = rows/2

    difX = imgX - redx - 300
    difY = imgY - redy - 300

    % -ve x value means to the right of the robot, +ve = left
    realredx = (difX - (framewidth * (redx / columns)));
    realredy = difY + (frame_realworld_height * (redy / rows));

    realgreenx = (difX - framewidth * (greenx / columns));
    realgreeny = difY + (frame_realworld_height * (greeny / rows));

    realbluex = (difX - framewidth * (bluex / columns));
    realbluey = difY + (frame_realworld_height * (bluey / rows));

    itemcoords = [realredx, realredy; realgreenx, realgreeny; realbluex, realbluey];
    
