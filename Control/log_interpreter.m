filename = 'log_2019-03-06 11:17:36.026708.log';
data = dlmread(filename);

time = data(:,1);
data = data(:, 2:end);