%this function generates a scaling for the input data
%scaled_data' = D*data' + c
%
%the input data (variable named data) needs to be a vertical array where
%columns need to be scaled individually
%
%isoutlier_ThresholdFactor - parameter of the isoutlier outlier detection
%algorithm
%if isoutlier_ThresholdFactor == 0, the outliers are not eliminated

function [D, c, scaled_data] = NNfunction_generate_scaling(data, isoutlier_ThresholdFactor)

if nargin < 2
    isoutlier_ThresholdFactor = 0;
end

m = size(data, 2);

if isoutlier_ThresholdFactor ~= 0
    TF = isoutlier(data, 'ThresholdFactor', isoutlier_ThresholdFactor);
    
    n = size(TF, 1); 
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
    scaled_data = reshape(data(~repmat(isoutlier_res, 1, m)), [], m);
    
    %clear from memory
    clear isoutlier_res TF;
else
    scaled_data = data;
end

max_val = max(scaled_data);
min_val = min(scaled_data);

D = diag(max_val-min_val) \ eye(m);
c = -D*min_val';

% scaled_data = (D*scaled_data' + c)';
scaled_data = scaled_data*D + c';
end