## Line Reconstruction
According to real position and orientation of camera, compute the 3D position of line features in images. Then plot in MATLAB.
### Dependence
MATLAB R2016b; OpenCV 3.1.0; <br>
ps: This project also depends on Eigen, G2O, and Sophus because I want it to be an odometry step by step.
### TORUN
1.Run extract_data to track line features and save result in 3 "txt" files: idx.txt, sp.txt, ep.txt.<br>
2.Run read_line.m to read data from previous 3 files and save in lineTracks.mat.<br>
3.Run lineTrack.m to calculate 3D lines and plot.<br>
