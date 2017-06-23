% imd = read_disparity(filename)
% Reads a disparity image from disk. Corrects endianness issues. Replaces
% 'nan_value' values with NaNs. 
% assumes 640x480 - change width, height, length(imd) values for different resolutions


function imd = read_disparity(filename, nan_value)

if(nargin < 2)
  nan_value = 2047;
    width = 640;
  height = 480;
end

use_custom_read = false;

%dat file, can read directly using fread
[fid,msg] = fopen(filename,'r');
if(fid < 0)
    error('realsense_toolbox:read_disparity:fopen',strrep([filename ':' msg],'\','\\'));
end
imd = fread(fid, '*uint16');%fscanf(fid,'%c',2);
fclose(fid);
  
%We need to reshape to get depth image.
if length(imd) == 307200
    imd = reshape(imd, width, height);

    % Rotate and flip to get an image aligned with usualRGB directions
    imd = fliplr(rot90(rot90(rot90(imd))));
else
      disp(filename);
      fprintf('Error: image read failed. Vector length %d \n', length(imd));
      
end


%Check channel count
if(size(imd,3) > 1)
  warning('read_disparity:channels','Disparity image has multiple channels, taking only first channel.');
  imd = imd(:,:,1);
end


% RealSense uses 0 to represent infinity/unresolvable pixels. So Set invalid depths to NaN
  imd = double(imd);
  imd(imd==0) = NaN;
