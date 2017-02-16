
function blkStruct = slblocks
		% This function specifies that the library should appear
		% in the Library Browser
		% and be cached in the browser repository

		%Browser.Library = 'cube_lib';
        	Browser.Library = 'cubex_simulation';
		% 'mylib' is the name of the library

		%Browser.Name = 'Cubex 2D';
        Browser.Name = 'Cubex simulation';
		% 'My Library' is the library name that appears in the Library Browser
        
		blkStruct.Browser = Browser; 
     



% %At the MATLAB® command prompt, enter this command to enable the model property EnableLBRepository. 
% Your library can appear in the browser only if this property is on when you save your library.
%
% Comment: Seems that the library must be open when doing this 
% %set_param(gcs,'EnableLBRepository','on');
% 
% 
