% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 693.151150703778853 ; 690.663051950498357 ];

%-- Principal point:
cc = [ 285.679711434935427 ; 244.430189832313260 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.104655986276354 ; -0.244819572083185 ; -0.000021277404980 ; 0.000417356947810 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.958275590576347 ; 1.928508566516935 ];

%-- Principal point uncertainty:
cc_error = [ 3.383274465109320 ; 2.551883667563774 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.014330721443249 ; 0.075529408765575 ; 0.001534086998006 ; 0.002152732908718 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.077094e+00 ; 2.167435e+00 ; -1.528609e-01 ];
Tc_1  = [ -7.152876e+01 ; -8.810707e+01 ; 3.905725e+02 ];
omc_error_1 = [ 3.125482e-03 ; 3.597960e-03 ; 7.162579e-03 ];
Tc_error_1  = [ 1.923224e+00 ; 1.439307e+00 ; 1.248098e+00 ];

%-- Image #2:
omc_2 = [ 2.175365e+00 ; 2.247393e+00 ; 1.755193e-01 ];
Tc_2  = [ -8.893200e+01 ; -5.594813e+01 ; 4.373708e+02 ];
omc_error_2 = [ 4.083032e-03 ; 3.565053e-03 ; 8.452015e-03 ];
Tc_error_2  = [ 2.150168e+00 ; 1.632764e+00 ; 1.474418e+00 ];

%-- Image #3:
omc_3 = [ 1.224390e-01 ; -2.785798e+00 ; 5.989411e-02 ];
Tc_3  = [ 1.322757e+02 ; -7.180351e+01 ; 3.610570e+02 ];
omc_error_3 = [ 1.031613e-03 ; 5.083609e-03 ; 5.640055e-03 ];
Tc_error_3  = [ 1.803069e+00 ; 1.373433e+00 ; 1.305514e+00 ];

%-- Image #4:
omc_4 = [ 4.277653e-02 ; 2.948855e+00 ; -1.026643e+00 ];
Tc_4  = [ 1.170751e+02 ; -4.826489e+01 ; 4.814927e+02 ];
omc_error_4 = [ 2.417290e-03 ; 4.898722e-03 ; 5.879795e-03 ];
Tc_error_4  = [ 2.347535e+00 ; 1.791940e+00 ; 1.361140e+00 ];

%-- Image #5:
omc_5 = [ 1.981014e+00 ; 1.701463e+00 ; 2.168173e-01 ];
Tc_5  = [ -5.528710e+01 ; -8.101089e+01 ; 4.261025e+02 ];
omc_error_5 = [ 3.669901e-03 ; 3.385809e-03 ; 6.082746e-03 ];
Tc_error_5  = [ 2.097905e+00 ; 1.566755e+00 ; 1.402510e+00 ];

%-- Image #6:
omc_6 = [ -1.842138e+00 ; -1.905756e+00 ; -4.672355e-01 ];
Tc_6  = [ -7.019889e+01 ; -8.572508e+01 ; 3.660956e+02 ];
omc_error_6 = [ 2.422482e-03 ; 3.898402e-03 ; 6.164327e-03 ];
Tc_error_6  = [ 1.814053e+00 ; 1.360621e+00 ; 1.275524e+00 ];

%-- Image #7:
omc_7 = [ 1.630983e+00 ; 1.511886e+00 ; -1.308399e-02 ];
Tc_7  = [ -1.077306e+02 ; -6.642790e+01 ; 4.691642e+02 ];
omc_error_7 = [ 3.165995e-03 ; 3.899347e-03 ; 5.011004e-03 ];
Tc_error_7  = [ 2.304303e+00 ; 1.742671e+00 ; 1.562966e+00 ];

%-- Image #8:
omc_8 = [ 1.778470e+00 ; 2.188540e+00 ; -1.194733e+00 ];
Tc_8  = [ -7.412444e+01 ; -8.995313e+01 ; 5.188068e+02 ];
omc_error_8 = [ 2.153838e-03 ; 4.613540e-03 ; 6.829228e-03 ];
Tc_error_8  = [ 2.554628e+00 ; 1.927464e+00 ; 1.354058e+00 ];

%-- Image #9:
omc_9 = [ -1.338243e+00 ; -1.817653e+00 ; 6.342840e-01 ];
Tc_9  = [ 1.171063e+01 ; -1.299014e+02 ; 4.972677e+02 ];
omc_error_9 = [ 3.480976e-03 ; 3.807511e-03 ; 4.852207e-03 ];
Tc_error_9  = [ 2.456810e+00 ; 1.832993e+00 ; 1.356319e+00 ];

%-- Image #10:
omc_10 = [ 1.849305e+00 ; 1.368420e+00 ; 6.711226e-01 ];
Tc_10  = [ -8.632571e+01 ; -5.857609e+01 ; 4.795453e+02 ];
omc_error_10 = [ 4.036454e-03 ; 3.321292e-03 ; 5.310190e-03 ];
Tc_error_10  = [ 2.369374e+00 ; 1.778768e+00 ; 1.708069e+00 ];

