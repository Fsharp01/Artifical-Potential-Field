function uniqueCoordinates = selectUniqueValues(matrix)
    uniqueValues = unique(sprintf('%.15e_%.15e\n', matrix'), 'rows');
    uniqueCoordinates = cellfun(@(x) str2double(strsplit(x, '_')), cellstr(uniqueValues), 'UniformOutput', false);
    uniqueCoordinates = cell2mat(uniqueCoordinates);
end