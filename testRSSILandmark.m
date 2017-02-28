% function [observed_LL, output_landmark_list]= getLandmarkRSSI(landmark_list)
clc;
clear all;
close all;
fclose(instrfind);

% RSSI Landmark Struct
s = getNodeStruct();