clear;
L = csvread("./build/nis_laser.csv");
R = csvread("./build/nis_radar.csv");
RMSE = csvread("./build/rmse.csv");
mean(RMSE)
