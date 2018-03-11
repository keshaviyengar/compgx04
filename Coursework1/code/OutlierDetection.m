%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%Outlier Detection Function
%inputs: H matrix (measurement matrix) and d_z (measurement innovation vector)
%
function outlier_index = OutlierDetection(H,d_z)

num_meas = size(H,1);
%a compute residuals vector
v = (H/(H'*H)*H' - eye(num_meas))*d_z;

%b compute residual covariance
sigma_p = 5;
C_v = (eye(num_meas) - H/(H'*H)*H')*sigma_p^2;

%c compute normalized residuals and compare to threshold
T = 6;
outlier_index = zeros(1,num_meas);
for j=1:size(H,1)
    if abs(v(j)) > sqrt(C_v(j,j))*T
        outlier_index(j) = v(j);
    end
end
end