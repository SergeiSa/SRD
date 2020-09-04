%the functions returns a square gain matrix Q of the correct size
%desired_size. If input is correct, it is returned. If input is a scalar,
%it is multiplied by an identity matrix of correct size.
function Q = OHfunction_GetCorrectWeights(input, desired_size)

if size(input, 1) == desired_size
    Q = input;
else
    Q = input * eye(desired_size);
end

end