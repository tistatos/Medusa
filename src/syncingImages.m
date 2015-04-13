
%% bajsbajsbajs
RGB_points = [85.191849, 65.543503; 99.306999, 65.570381; 113.41929, 65.701408; 127.57812, 65.952141; 142.442, 66.115028; 157.11238, 66.372688; 172.39476, 66.402; 84.835304, 79.573997; 98.943726, 79.86071; 113.16443, 80.217537; 127.46212, 80.441177; 142.18559, 80.488312; 157.08153, 80.496101; 172.36249, 80.683937; 84.567085, 93.735725; 98.637367, 94.193321; 112.78877, 94.437958; 127.37285, 94.525764; 142.02054, 94.964317; 156.85295, 95.492477; 172.11006, 95.593262; 84.313446, 108.27714; 98.51609, 108.39649; 112.47041, 108.52999; 127.29625, 109.30547; 141.72577, 109.55335; 156.75957, 110.28821; 172.32007, 110.52209; 83.707726, 122.60802; 98.220612, 123.22353; 112.4814, 123.48775; 127.06523, 123.88881; 141.77043, 124.48642; 156.7505, 125.29935; 172.44151, 125.64103];
IR_points = [92.867851, 64.565132; 108.95816, 65.266777; 124.25905, 65.458427; 140.69386, 66.725693; 156.56677, 66.779732; 172.7496, 67.357849; 188.5, 68; 92.421478, 80.872887; 108.30152, 81.56089; 123.52887, 82.728516; 143.5, 82; 155.92503, 82.826958; 171.67743, 83.96804; 188.08772, 84.42514; 92.188217, 96.671806; 107.56848, 97.70179; 122.77666, 98.482162; 139.42776, 99.591599; 155.54652, 99.46376; 172.20107, 100.19595; 187.3669, 100.48555; 91.10183, 113.29824; 107.14317, 113.9165; 122.79343, 114.59597; 139.36778, 115.2536; 154.43375, 115.6627; 171.0791, 116.55158; 187.30707, 117.64093; 90.383041, 129.02365; 106.07482, 130.35121; 121.90832, 130.71802; 138.15852, 131.66658; 154.59003, 132.10577; 170.82159, 133.26283; 187.12767, 133.78418];
RGB_pic = imread('testrgb.jpg');
IR_pic = imread('testir.jpg');

point_diff = RGB_points - IR_points;
diff_rounded = round(point_diff);
p_one = diff_rounded(20,:);
p_one = p_one * (-1);
IR_pic_shifted = circshift(IR_pic, p_one);

% image(:,:,1) = RGB_pic(:,:,1);
% image(:,:,2) = IR_pic_shifted(:,:,2);
% image(:,:,3) = zeros(160,213);
% figure;
% imshow(image);

IR_points = round(IR_points);
markerInserter = vision.MarkerInserter('Shape','X-mark','BorderColor','Custom','CustomBorderColor',uint8([0 0 255]));
Pts = int32([IR_points(1,:); IR_points(7,:); IR_points(29,:); IR_points(35,:)]);
J = step(markerInserter, IR_pic, Pts);
imshow(J);

%%
[GX,GY] = ginput(M);

% display selected dots
hold on;
for k = 1:M
    text(GX(k),GY(k),num2str(k));
end
hold off;
%% C select points H

figure;
imshow(Him);
[HX,HY] = ginput(M);

% display selected dots
hold on;
for k = 1:M
    text(HX(k),HY(k),num2str(k));
end
hold off;

%% D collect coordinates

GC = [GX GY];
HC = [HX HY];

GC1 = [GC ones(M,1)];
HC1 = [HC ones(M,1)];

%% E find transformation matrix

A = mldivide(GC1,HC1);

%% F,G Gim -> Him

% This is an attempt to get it working without imwarp. Not working.
% warpedimage = zeros(size(Gim));
% [sizeX, sizeY] = size(Him);
% for X = 1:sizeX;
%     for Y = 1:sizeY;
%         % T = [newX, newY, n]
%         T = [X Y 1] * inv(A);
%         T = T/T(3);
%         T = round(T);
%         if T(1) > 0 && T(1) < sizeX+1 && T(2) > 0 && T(2) < sizeY+1
%             warpedimage(X, Y) = Him(T(1), T(2));
%         end
%     end
% end

A(:,3) = [0 0 1];
At = affine2d(A);

warpedimage = imwarp(Gim, At, 'OutputView', imref2d(size(Him)));

imshowpair(Him, warpedimage);