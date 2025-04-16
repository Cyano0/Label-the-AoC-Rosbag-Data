% ✅ Check Operating System
if ispc
    osType = 'Windows';
elseif isunix
    osType = 'Unix/Linux';
elseif ismac
    osType = 'Mac';
else
    osType = 'Unknown';
end
disp(['Operating System: ', osType]);

% Create a separate folder for JSON files (fullfile handles OS differences)
outputFolder = fullfile(pwd, 'json_files');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

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

% Process each topic separately
for s = 1:numel(sourceNames)
    topicName = sourceNames{s};
    labelTable = gTruth.ROILabelData.(topicName); % Extract label data

    % Define timestamp file location and load timestamps if available
    timestampFile = fullfile(pwd, topicName, 'timestamps.mat');
    if isfile(timestampFile)
        timestampStruct = load(timestampFile);
        timestamps = sort(timestampStruct.timestamps); % Sort timestamps
        disp(['✅ Loaded & Sorted timestamps from ', timestampFile]);
    else
        warning(['⚠️ Missing timestamp file: ', timestampFile]);
        timestamps = linspace(0, 1, height(labelTable)); % Placeholder timestamps
    end

    % Initialize structured data for this topic
    numFrames = height(labelTable);
    labeledData = repmat(struct('Timestamp', NaN, 'File', '', 'Labels', []), numFrames, 1);

    % Get label names dynamically from gTruth.LabelDefinitions (only image labels)
    labelNames = imageLabels; % for example, {'human', ...}

    % Process each frame
    for i = 1:numFrames
        timestamp = timestamps(i);
        file = sprintf('%f.png', timestamp); % generate filename for the frame

        % Initialize an empty array of label entries for this frame.
        labels = struct('Class', {}, 'BoundingBoxes', {});

        % Process each label defined in the data
        for j = 1:numel(labelNames)
            labelName = labelNames{j};

            if ismember(labelName, labelTable.Properties.VariableNames)
                entry = labelTable.(labelName){i}; % Get the stored data from the cell

                if strcmp(labelName, 'human')
                    % Special processing for the "human" label.
                    % Check that the cell contains a struct array with fields 'id' and 'Position'
                    if isstruct(entry) && isfield(entry, 'id') && isfield(entry, 'Position')
                        % Process each ROI separately
                        for roi_idx = 1:numel(entry)
                            roi_entry = entry(roi_idx);
                            % Convert the 'id' field (a string like '1') into a numeric value.
                            id_value = str2double(roi_entry.id);
                            if ~isnan(id_value) && ismember(id_value, 0:5)
                                newLabelName = sprintf('human%d', id_value);
                            else
                                newLabelName = 'human';
                            end
                            % Extract the bounding box using the 'Position' field.
                            bbox = roi_entry.Position;
                            if iscell(bbox)
                                bbox = cell2mat(bbox);
                            end
                            % Validate bbox if it is numeric
                            if ~isempty(bbox)
                                if isnumeric(bbox)
                                    valid = all(~isnan(bbox(:)));
                                else
                                    valid = true;
                                end
                                if valid
                                    labels(end+1).Class = newLabelName;      %#ok<SAGROW>
                                    labels(end).BoundingBoxes = bbox;
                                end
                            end
                        end
                        % Since we have processed all "human" ROIs individually, move on.
                        continue;
                    else
                        % Fallback: if entry isn’t in the expected format, process as a generic label.
                        newLabelName = 'human';
                        bbox = entry;
                    end
                else
                    % For labels other than "human", use the original label name.
                    newLabelName = labelName;
                    bbox = entry;
                end

                % If the bounding box is stored in a cell, convert it to a numeric array.
                if iscell(bbox)
                    bbox = cell2mat(bbox);
                end
                if ~isempty(bbox)
                    if isnumeric(bbox)
                        valid = all(~isnan(bbox(:)));
                    else
                        valid = true;
                    end
                    if valid
                        labels(end+1).Class = newLabelName;      %#ok<SAGROW>
                        labels(end).BoundingBoxes = bbox;
                    end
                end
            end
        end

        % Store the frame data.
        labeledData(i).Timestamp = timestamp;
        labeledData(i).File = file;
        labeledData(i).Labels = labels;

        % Optional: Print labels for debugging.
        disp(['📝 Frame ', num2str(i), ' in ', topicName, ': ', jsonencode(labels)]);
    end

    % Convert the data for this topic to JSON.
    jsonData = jsonencode(labeledData, 'PrettyPrint', true);

    % Save the JSON file in the designated output folder.
    jsonFile = fullfile(outputFolder, sprintf('labeled_data_%s.json', topicName));
    fid = fopen(jsonFile, 'w');
    fwrite(fid, jsonData, 'char');
    fclose(fid);
    disp(['✅ Labeled data saved to ', jsonFile]);
end
