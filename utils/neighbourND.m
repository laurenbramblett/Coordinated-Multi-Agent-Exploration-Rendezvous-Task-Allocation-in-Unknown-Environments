function [Iadj , Radj, Nfound ] = neighbourND( index, sizeA, res )
% function  [Iadj , Radj, Nfound] = neighbour3D( index,  sizeA, res )
% Calculate the linear indices for neighboring points in a matrix 
% Second output is and array of distances based on an input resolution vector
% This resolution vector defaults to ones(1,ndims)
% The output Nfound reports the number of neighbours found in within the
% matrix. For 2D we expect up to 8, for 3D up to 26 etc...
% 
% Example 1:
% A is a 128x128x16 image data matrix with a spatial resolution of
% 0.1x 0.25x .375 mm^3 
% to get the neighbouring point linear indices for point 456 we do
% sizeA = [128 128 16]
% [ Iadj , Radj, Nfound] = neighbourND( 456, sizeA, [ .10 .25 .375] )
%
% NEW: now index can be a column array with linear indices
% Output Iadj will be Nx8 (2D) or Nx26 (3D) etc and Radj will be 
% a row array 1x8 or 1x26 etc... 
%
% Example 2:
% create points near the center of a 144x192x16 matrix
% spatial resolution .3 x .3x 5 mm^3
% idx = (-6:1:6)+((144*192*3)+144*96+76)
%[ Iadj , Radj, Nfound] = neighbourND( idx , [144,192, 32] , [.3, 0.3, 5])
% Results in 11x26 matrix Iadj, 
% 26 distances in Radj and Nfound is 26
%
% The neighbour indices outside the matrix will be zero!
% when a single index is entered the outside points are still removed so a
% point in a 3D matrix at the edge can sill return 17 neighbours or even less
% when it is a corner.
%==============================================

%==============================================
% Ronald Ouwerkerk 2010 NIH/NIDDK 
% New version: Now handles arrays of indices
% This script is made available on Matlab file exchange by the author 
% for use by other Matlab programmers.
% This script is not intended for commercial use.
% If used for published work a reference or acknowledgement is greatly 
% appreciated.
% The function was tested for several 1D(col and row), 2D, 3D and 4D cases
% I cannot be sure that it really works for all dimensionalities. 
% Let me know if you find a bug (and feel free to squash it for me)
%==============================================

%% Set defaults and process input parameters
% first two are arbitary values for a demo
if nargin <1
    % default index [7,6,2]
    index = 128*128+ 128*5+7
end

if nargin < 2
    % default size 128x128xN with N big enough for the index
    i3 = floor( index /128/128);
    disp( 'Demo mode')
    sizeA =[128, 128, i3+2]
end

% Get dimensionality
ndimA = length( sizeA );

%Set default resolution to isotropic distances
if nargin < 3
    res =ones(1, length( sizeA) );
else
    if length(res) < ndimA;
        errstr = sprintf('\nError in %s.\n The length of the resolution array (%d) must equal the number of matrix dimensions (%d)\n', ...
                                         mfilename,                                           length(res)  ,                                                         ndimA  );
        disp(errstr)
        help( mfilename)
        return
    else
        % reduce the resolution array, last digit is probably slice
        % thickness, irrelevant if we have one slice only
        res = res( 1:ndimA );
    end
end

%% explicit version of ind2sub 
% ind2sub requires multiple output arguments, one for each dimension
ilin = index(:);
np = length( ilin );
imat = ones( np, ndimA);

for di = ndimA:-1:2    
    blocksize = prod( sizeA( 1:(di-1)  ) );
    ndi = 1+ floor( ( ilin-1) / blocksize );
    ilin = ilin- (ndi -1) *blocksize;
    imat(:,di) = ndi;
end
imat(:,1) = ilin;

%% Find the indices of neighbours
% Get all the index permutations for neighbours ( -1, +1) over all
% dimensions. The total number of neighbours should be three  to the power Ndim
% minus one if we discard the original point itself

% initialize the shift index array
nneighb = 3^ndimA;
nbi = zeros( nneighb, ndimA);

di = ndimA;
while ( di ) 
    N = 3^(di-1);
    ni = 1:N;
    while( ni(end) < nneighb+1 )
        for val=[-1, 0, 1]
              nbi( ni ,di ) = val;
              ni = ni+ N;
        end
    end
    di = di-1;
end

%% Create distance matrix
d = ones(nneighb, 1) * res;
d = d.*abs( nbi );
% create a row vector with distances
dvec = sqrt( sum( d.^2, 2))';
% Get index to exclude the original point: distance = 0
notorig = logical( dvec > 0 );

%% Add the input index array to nbi to get all neighbours
% set up the array for neighbour indices
nd = length( index);
Iadj = zeros( nd, nneighb );
kdo = notorig(ones(nd,1), : ); 

for di = 1:ndimA
    indices = imat( :, di );
    shifts = nbi( :, di )';
    neighbindices = indices( :, ones( 1,nneighb)) +shifts( ones(nd, 1), : ) ;
    maxmat = sizeA( di );
    % set up mask matrix to keep indices within limits and excllude the original point
    s = logical( neighbindices <= maxmat );
    s =logical( neighbindices > 0 ) & s;
    kdo = kdo & s;
    % Calculate the linear index
    if di == 1       
        Iadj( kdo ) =  neighbindices( kdo );
    else
        blocksize = prod( sizeA( 1:(di-1)  ) );
        m = neighbindices-1;
        Iadj(kdo )  = Iadj(kdo )+ m(kdo)*blocksize;
    end
end

%% Select only the sensible points for the neighbour index and distances matrices
% Remove columns that have no valid indices anywhere at all (e.g. origin)
% for shorter index lists with  all points near the edges more may be
% removed.
if nd == 1
    allkdo = any( kdo, 1);
    Iadj = Iadj( :, allkdo);
    Radj = dvec( allkdo );
    Nfound = length(  find( allkdo ) );
else
    Nfound = nneighb-1;
    Radj = dvec;
    iself = (Radj == 0);
    Iadj = Iadj(:,~iself);
    Radj = Radj(~iself);
end

%END



