%% ✅ Check Operating System
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

%% ✅ Create output folder for JSON files
outputFolder = fullfile(pwd, 'json_files');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% ✅ Load gTruth (uncomment if you need to load from disk)
% load('gTruth.mat');   % assumes variable gTruth appears in the file

%% ✅ Extract topics and label definitions
sourceNames      = fieldnames(gTruth.ROILabelData);
labelDefinitions = gTruth.LabelDefinitions;
numLabels        = numel(labelDefinitions.Name);
% Only take the first half (2D image labels)
imageLabels      = unique(labelDefinitions.Name(1:numLabels));

disp('Available Data Sources:');
disp(sourceNames);
disp('🔍 Extracted Image Labels:');
disp(imageLabels);

%% ✅ Process each topic separately
for s = 1:numel(sourceNames)
    topicName  = sourceNames{s};
    labelTable = gTruth.ROILabelData.(topicName);
    
    % — Load or fallback timestamps —
    timestampFile = fullfile(pwd, topicName, 'timestamps.mat');
    if isfile(timestampFile)
        tsStruct   = load(timestampFile);
        timestamps = sort(tsStruct.timestamps);
        disp(['✅ Loaded & Sorted timestamps from ', timestampFile]);
    else
        warning(['⚠️ Missing timestamp file: ', timestampFile]);
        timestamps = linspace(0,1,height(labelTable));  % placeholder
    end
    
    numFrames   = height(labelTable);
    % Preallocate struct array for JSON export
    labeledData = repmat(struct('Timestamp', NaN, 'File', '', 'Labels', []), ...
                         numFrames, 1);

    %% — Loop over frames —
    for i = 1:numFrames
        ts   = timestamps(i);
        file = sprintf('%f.png', ts);
        
        % Dynamic array of label structs for this frame
        labels = struct('Class', {}, 'BoundingBoxes', {});
        
        %% — Loop over each defined label name —
        for j = 1:numel(imageLabels)
            lbl = imageLabels{j};
            if ~ismember(lbl, labelTable.Properties.VariableNames)
                continue;
            end
            
            entry = labelTable.(lbl){i};
            
            % —— Special "human" handling ——
            if strcmp(lbl,'human') && isstruct(entry) ...
                    && isfield(entry,'id') && isfield(entry,'Position')
                
                % Each element of the struct array is one ROI
                for k = 1:numel(entry)
                    roi = entry(k);
                    % Convert ID from char to number
                    idNum = str2double(roi.id);
                    if ~isnan(idNum) && ismember(idNum,0:5)
                        className = sprintf('human%d', idNum);
                    else
                        className = 'human';
                    end
                    % Extract Position field
                    bbox = roi.Position;
                    if iscell(bbox)
                        bbox = cell2mat(bbox);
                    end
                    % Validate numeric bbox or accept non‑numeric
                    if ~isempty(bbox) && ( ~isnumeric(bbox) || all(~isnan(bbox(:))) )
                        labels(end+1).Class          = className;   %#ok<SAGROW>
                        labels(end).BoundingBoxes    = bbox;
                    end
                end
                continue;  % done with "human"
            end
            
            % —— Generic label handling ——
            className = lbl;
            bbox      = entry;
            
            % — NEW: only unwrap scalar structs to Position/position —
            if isstruct(bbox) && numel(bbox)==1
                if isfield(bbox,'Position')
                    bbox = bbox.Position;
                elseif isfield(bbox,'position')
                    bbox = bbox.position;
                end
            end
            
            % Convert cell to numeric
            if iscell(bbox)
                bbox = cell2mat(bbox);
            end
            
            % Validate and append
            if ~isempty(bbox) && ( ~isnumeric(bbox) || all(~isnan(bbox(:))) )
                labels(end+1).Class       = className;   %#ok<SAGROW>
                labels(end).BoundingBoxes = bbox;
            end
        end
        
        % — Store frame data —
        labeledData(i).Timestamp = ts;
        labeledData(i).File      = file;
        labeledData(i).Labels    = labels;
        
        % Optional debugging
        disp(['📝 Frame ', num2str(i), ' in ', topicName, ': ', jsonencode(labels)]);
    end
    
    %% ✅ Save this topic’s data as a pretty‑printed JSON
    jsonStr = jsonencode(labeledData, 'PrettyPrint', true);
    outFile = fullfile(outputFolder, sprintf('labeled_data_%s.json', topicName));
    fid     = fopen(outFile,'w');
    fwrite(fid, jsonStr, 'char');
    fclose(fid);
    disp(['✅ Labeled data saved to ', outFile]);
end
