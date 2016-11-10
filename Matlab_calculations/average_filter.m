function filtered = average_filter(data, period)
% AVERAGE_FILTER    calculates the average period of a periodical series of
% data
%   filtered = AVERAGE_FILTER(data, period) outputs a sequence of data of
%   length 'period' with values that correspond to the average value of the
%   data in different periods. period = 0 will return the unfiltered data.

if period < length(data) && period > 0
    filtered = zeros(period,1);

    for i=1:length(filtered)
        sum = 0;        % tracks the sum of the same value across different periods
        nb_periods = 0; % the amount of periods currently viewed
        index = i;      % index of the element currently viewed
        
        while nb_periods+1 < length(data)/period
            sum = sum + data(index);
            nb_periods = nb_periods + 1;
            index = i + nb_periods*period;            
        end
        
        filtered(i) = sum/nb_periods;
    end
    
else
    % There isn't any periodicity present in the data
    filtered = data;
end

end