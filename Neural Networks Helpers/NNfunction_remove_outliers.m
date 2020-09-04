%this function removes outliers from both the input and teh output data
%simultaniously
%
%the data (variables X and Y) needs to be vertical arrays
%
%isoutlier_ThresholdFactor - parameter of the isoutlier outlier detection
%algorithm
function [fixed_X, fixed_Y] = NNfunction_remove_outliers(X, Y, isoutlier_ThresholdFactor)

if nargin < 3
    isoutlier_ThresholdFactor = 3;
end

TFX = isoutlier(X, 'ThresholdFactor', isoutlier_ThresholdFactor);
TFY = isoutlier(Y, 'ThresholdFactor', isoutlier_ThresholdFactor);
TF = [TFX, TFY];

n = size(TF, 1); 
m = size(TF, 2); mx = size(TFX, 2); my = size(TFY, 2);
isoutlier_res = false(n, 1);
for j = 1:m
    %we want to find rows where at least one element is an outlier
    %the outliers are true, the rest are false (see isoutlier docs)
    isoutlier_res = or(isoutlier_res, TF(:, j));
end

%the code below can be expanded as follows:
%
%logical_indexes = repmat(isoutlier_res, 1, m);
%data_without_outliers_vector = data(~logical_indexes);
%data_without_outliers_matrix = reshape(data_without_outliers_vector, [], m);
%
%the code is squashed like this because the size of data might be huge, so
%it would be desirable to avoid generating variables of similar sizes.
fixed_X = reshape(X(~repmat(isoutlier_res, 1, mx)), [], mx);
fixed_Y = reshape(Y(~repmat(isoutlier_res, 1, my)), [], my);
end