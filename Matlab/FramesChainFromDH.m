%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FramesChainFromDH( DH,q,Convention ) builds the kinematic chain of a    %
% robot given its D-H table.                                              %
% The format of the D-H table have to be as in the following example:     %
% D-H table format example:                                               %
%  joint type | d_i | theta_i | a_i  | alpha_i                            %
%       1     | d1  |    q1   |  0   |   pi/2                             %
%                                                                         %
% joint type: 1 'revolute', 2 'prismatic'                                 %
% Convention: 'classic' (default) | 'modified'                            %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [ framesChain ] = FramesChainFromDH(DH,q,Convention )

if nargin < 3
   Convention = 'classic';  % default 'classic D-H convention'
else
    if ~strcmp(Convention,'modified')  % 'modified D-H convention'
        Convention = 'classic';
    end
end

framesChain = zeros(4,4,size(DH,1));


q(1) = DH(1,3)+q(1);
if strcmp(Convention,'classic')
    framesChain(:,:,1) =  [cos(q(1)), -cos(DH(1,5))*sin(q(1)),  sin(DH(1,5))*sin(q(1)),DH(1,4)*cos(q(1));
                           sin(q(1)),  cos(DH(1,5))*cos(q(1)), -sin(DH(1,5))*cos(q(1)),DH(1,4)*sin(q(1));
                                 0     ,            sin(DH(1,5))   ,          cos(DH(1,5))     ,     DH(1,2)        ;
                                 0     ,                 0         ,              0            ,         1         ];
else
    framesChain(:,:,1) =  [     cos(q(1))        ,         -sin(q(1))     ,      0       ,      DH(1,4);
                           cos(DH(1,5))*sin(q(1)),  cos(DH(1,5))*cos(q(1)), -sin(DH(1,5)),-sin(DH(1,5))*DH(1,2);
                           sin(DH(1,5))*sin(q(1)),  sin(DH(1,5))*cos(q(1)), cos(DH(1,5)) , cos(DH(1,5))*DH(1,2);
                                    0               ,                 0         ,      0       ,         1         ];
end

for i=2:size(DH,1)
    q(i) = DH(i,3)+q(i);
    if strcmp(Convention,'classic')
        framesChain(:,:,i) =  framesChain(:,:,i-1)*[cos(q(i)), -cos(DH(i,5))*sin(q(i)),  sin(DH(i,5))*sin(q(i)),DH(i,4)*cos(q(i));
                                                    sin(q(i)),  cos(DH(i,5))*cos(q(i)), -sin(DH(i,5))*cos(q(i)),DH(i,4)*sin(q(i));
                                                          0     ,            sin(DH(i,5))   ,          cos(DH(i,5))     ,     DH(i,2)        ;
                                                          0     ,                 0         ,              0            ,         1         ];
    else
        framesChain(:,:,i) =  framesChain(:,:,i-1)*[     cos(q(i))        ,         -sin(q(i))     ,      0       ,      DH(i,4);
                                                    cos(DH(i,5))*sin(q(i)),  cos(DH(i,5))*cos(q(i)), -sin(DH(i,5)),-sin(DH(i,5))*DH(i,2);
                                                    sin(DH(i,5))*sin(q(i)),  sin(DH(i,5))*cos(q(i)), cos(DH(i,5)) , cos(DH(i,5))*DH(i,2);
                                                             0               ,                 0         ,      0       ,         1         ];
    end
end


end