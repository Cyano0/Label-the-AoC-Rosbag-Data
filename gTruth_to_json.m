% Load gTruth from workspace (or from a .mat file)
% Uncomment if loading from a .mat file:
% load('gTruth.mat');

% ✅ Extract available topics from gTruth.ROILabelData
sourceNames = fieldnames(gTruth.ROILabelData);
disp('Available Data Sources:');
disp(sourceNames);

% ✅ Extract label definitions (filtering for 2D image labels only)
labelDefinitions = gTruth.LabelDefinitions;
numLabels = numel(labelDefinitions.Name); % First half are 2D image labels, second half are 3D labels
imageLabels = unique(labelDefinitions.Name(1:numLabels)); % Take only the 2D labels

% ✅ Verify extracted label names
disp('🔍 Extracted Image Labels:');
disp(imageLabels);

% ✅ Process each topic separately
for s = 1:numel(sourceNames)
    topicName = sourceNames{s};
    labelTable = gTruth.ROILabelData.(topicName); % Extract label data

    % ✅ Define timestamp file location
    timestampFile = fullfile(pwd, topicName, 'timestamps.mat');

    % ✅ Load timestamps if available
    if isfile(timestampFile)
        timestampStruct = load(timestampFile);
        timestamps = sort(timestampStruct.timestamps); % Sort timestamps
        disp(['✅ Loaded & Sorted timestamps from ', timestampFile]);
    else
        warning(['⚠️ Missing timestamp file: ', timestampFile]);
        timestamps = linspace(0, 1, height(labelTable)); % Placeholder timestamps
    end

    % ✅ Initialize labeled data for this topic
    numFrames = height(labelTable);
    labeledData = repmat(struct('Timestamp', NaN, 'File', '', 'Labels', struct([])), numFrames, 1);

    % ✅ Get label names dynamically from gTruth.LabelDefinitions (image labels)
    labelNames = imageLabels; % Use ordered labels from LabelDefinitions

    for i = 1:numFrames
        % ✅ Assign timestamps sequentially (sorted order)
        timestamp = timestamps(i);
        file = sprintf('%f.png', timestamp); % Generate correct filename

        % ✅ Use a dictionary to prevent duplicate labels per frame
        labelMap = containers.Map;

        % ✅ Extract bounding boxes for labels
        for j = 1:numel(labelNames)
            labelName = labelNames{j};

            if ismember(labelName, labelTable.Properties.VariableNames) % Check if label exists in the table
                boundingBox = labelTable.(labelName){i}; % Extract bounding box
                
                % ✅ Convert cell to numeric array if necessary
                if iscell(boundingBox)
                    boundingBox = cell2mat(boundingBox);
                end
                
                % ✅ Ensure bounding box exists before adding
                if ~isempty(boundingBox) && all(~isnan(boundingBox(:)))
                    if ~isKey(labelMap, labelName) % Prevent duplicate labels
                        labelMap(labelName) = boundingBox;
                    end
                end
            end
        end

        % ✅ Convert dictionary to struct array
        labels = struct([]);
        keys = labelMap.keys;
        for k = 1:numel(keys)
            labels(end+1).Class = keys{k};
            labels(end).BoundingBoxes = labelMap(keys{k});
        end

        % ✅ Store structured data
        labeledData(i).Timestamp = timestamp;
        labeledData(i).File = file;
        labeledData(i).Labels = labels;

        % ✅ Debugging: Print stored labels per frame
        disp(['📝 Frame ', num2str(i), ' in ', topicName, ': ', jsonencode(labels)]);
    end

    % ✅ Convert structured data to JSON
    jsonData = jsonencode(labeledData, 'PrettyPrint', true);

    % ✅ Save JSON file per topic
    jsonFile = fullfile(pwd, topicName, 'labeled_data.json');
    fid = fopen(jsonFile, 'w');
    fwrite(fid, jsonData, 'char');
    fclose(fid);

    disp(['✅ Labeled data saved to ', jsonFile]);
end
