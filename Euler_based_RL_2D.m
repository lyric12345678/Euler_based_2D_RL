clear
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Objective: Provide a proof of the Appendix A of paper: "Robot-to-Robot Relative
%Pose Estimation based on Semidefinite Relaxation Optimization" 
%Author: Ming Li
%Date: Feb./09/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Euler Angle Representation
syms d_0 theta phi x_1l y_l1 x_21 y_21 d_meas
p = d_0*[cos(theta),sin(theta)].';
C = [cos(phi), -sin(phi); sin(phi), cos(phi)];
q_l1 =[x_1l y_l1].';
q_l2 =[x_21 y_21].';
w = p + C*q_l2 - q_l1;
residual=w.'*w-d_meas^2;
residual_poly=expand(residual);
residual_simp=simplify(residual_poly)
x=[cos(theta),sin(theta),cos(phi),sin(phi),cos(phi-theta),sin(phi-theta)];
x_represent_poly=collect(residual_simp,x);
[coeffcients,variable]=coeffs(x_represent_poly,x);
variable                                                                   % Note that in our paper, the variable is diffent, which is x=[cos(theta),sin(theta),cos(phi),sin(phi),cos(theta-phi),sin(theta-phi)];
coeffcients.'                                                              % The sixth coefficent would be change to 2*d_0*y_21 since sin(phi-theta)= sin(theta-phi)