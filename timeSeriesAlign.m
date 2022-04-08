function [new_aligned_value] = timeSeriesAlign(standard,alignment, alignment_value)
%TIMESERIESALIGN 此处显示有关此函数的摘要
%   此处显示详细说明
size_standard = length(standard);%8100, dt = 0.037
size_alignment = length(alignment);%8950 dt = 0.033
new_aligned_value = zeros(size_standard, 3);
new_aligned_value(1) = new_aligned_value(1);

for index = 2:1:size_standard
    key = standard(index);
    j = 2;
    
    while alignment(j) < key && j < size_alignment
        j = j + 1;
    end
    
    left_key = alignment(j-1);
    right_key = alignment(j);
    
    left_point = alignment_value(j-1,:);
    right_point = alignment_value(j,:);
    
    factor = 1/abs(left_key-key)/(1/abs(left_key-key)+1/abs(right_key-key));
    new_aligned_value(index,:) = factor*left_point + (1-factor)*right_point;
end

