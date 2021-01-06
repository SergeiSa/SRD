%   This function returns LinkArray generated using URDF file path
%   ParseSTL models of URDF are parsed or not according to the ParseSTL 
%   parameter

function result = SRD_GetLinkArrayFromURDF(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'SRD_GetLinkArrayFromURDF';
    Parser.addOptional('UrdfFilePath', []);
    Parser.addOptional('ParseSTL', false);
    Parser.parse(varargin{:});
   
    if isempty(Parser.Results.UrdfFilePath)
        error('Please pass URDF file path')
    else
        if exist(Parser.Results.UrdfFilePath,'file')
            UrdfFilePath = Parser.Results.UrdfFilePath;
            ParseSTL = Parser.Results.ParseSTL;
        else
            error('URDF path does not exist')
        end
    end
    
    robot = importrobot(UrdfFilePath);

    robot = robot(1);
    
    xml = xmlread(UrdfFilePath);
    links = xml.getElementsByTagName('link');
    
    %Create ground link
    result = [SRD_get_Link_Ground()];
    joints = [];
    result(1).RelativeFollower = [];
    result(1).AbsoluteFollower = [];
    %Creating links
    body_count = robot.NumBodies;
    body_structs = [];

    ground_link_alias = robot.BaseName;
        
    coord_index = 0;
    for body_idx=1:body_count
        body = robot.Bodies(body_idx);
        body = body{1};
        parent_link = body.Parent;
        RelativeCoM = reshape(body.CenterOfMass,[3,1]); 
        Mass = body.Mass;
        Name = body.Name;
        MeshPath = '';
        MeshScale = [1,1,1];
        if ~isempty(body.Visuals)
            
            if ParseSTL
                [MeshPath,MeshScale] = SRDhelper_GetMeshInfoFromLinkName('LinkXMLNodes', links,...
                    'UrdfFilePath',UrdfFilePath,'LinkName',Name);
            else
                
            end
        end

        Inertia = UPH_GetInertiaMatrix('LinkXMLNodes', links,'LinkName',Name);

        RelativeFollower = [];
        RelativeBase = [0; 0; 0];


        if ~isempty(parent_link)
            joint_info = body.Joint;

            joint_Type = joint_info.Type;
            parent_name = parent_link.Name;

            %Searching for the parent struct in the list
            name_matcher = @(i) strcmp(result(i).Name,parent_name);
            match = arrayfun(name_matcher, 1:numel(result));
            parent_index = find(match);
            parent_obj = result(parent_index);
            
            if strcmp(parent_name,ground_link_alias)
                parent_obj = result(1);
            end
            
            joint_name = "none";
            
            if strcmp(joint_Type,'floating')   
                    joint_name = 'FloatingBase_6dof';
            else

                if strcmp(joint_Type,'planar')
                    joint_name = 'planar';
                end

                if strcmp(joint_Type,'pivot') || strcmp(joint_Type,'revolute')
                    joint_name = 'pivot';
                end

                if strcmp(joint_Type,'prismatic')
                    joint_name = 'prismatic';
                end

                if ~isnan(joint_info.JointAxis)
                    if joint_info.JointAxis(1)~=0 && joint_info.JointAxis(2)~=0 && joint_info.JointAxis(3)~=0
                        joint_name = [joint_name,'_XYZ'];
                    else
                        if joint_info.JointAxis(1)~=0
                            joint_name = [joint_name,'X'];
                        end
                        if joint_info.JointAxis(2)~=0
                            joint_name = [joint_name,'Y'];
                        end
                        if joint_info.JointAxis(3)~=0
                            joint_name = [joint_name,'Z'];
                        end
                    end
                end
            end
            
            %orientation_matrix =eye(3);
            
            %Extracting the child frame offset from the parent frame
            transform_to_joint = getTransform(robot, homeConfiguration(robot), body.Name, parent_name);
            joint_origin = transform_to_joint*[0;0;0;1];
            
            parent_obj.RelativeFollower = [parent_obj.RelativeFollower,joint_origin(1:3)];
            parent_obj.AbsoluteFollower = [parent_obj.AbsoluteFollower,joint_origin(1:3)];

            orientation_matrix = transform_to_joint(1:3,1:3);
   
            if strcmp(joint_Type,'fixed')
                joint_name = 'fixed';
            end
            
            joint_urdf_type = split(joint_info.Name,'_')
            
            if strcmp(joint_urdf_type(end),'floating')
                joint_name = 'floating';
            end
            
%             body_obj = SRDLinkWithJoint('JointType',,'Order', body_idx,'FileName', [],...
%             'LinkParametersStructure', body_struct, 'ParentLink', parent_obj,'ParentFollowerNumber',
%             size(parent_obj.RelativeFollower,2));%!!!
%             
            
            body_obj = SRD_get_Link(...
                'Order', body_idx, ...
                'Name', Name, ...
                'RelativeBase', RelativeBase, ...
                'RelativeFollower', RelativeFollower, ...
                'RelativeCoM', RelativeCoM, ...
                'Mass', Mass, ...
                'Inertia', Inertia, ...
                'ToDisplay', true, ...
                'Color', 'r', ...
                'StlPath', MeshPath);

            body_obj.StlPath = MeshPath;
            
            if ~isempty(MeshPath)
                body_obj.Mesh = matlab.internal.meshio.stlread(MeshPath);
                body_obj.Mesh.Vertices = body_obj.Mesh.Vertices.*MeshScale;
            end
            
            current_gen_coords = [];
            if ~strcmp(joint_name,'fixed')
                coord_index_new = coord_index + GetJointInputsRequirements(joint_name);
                current_gen_coords = (coord_index+1):coord_index_new;
                coord_index = coord_index_new;
            end
            

             new_joint = SRD_get_Joint(...
                    'Name', [Name,'Joint'], ...
                    'Type', joint_name, ...
                    'ChildLink',  body_obj, ...
                    'ParentLink', parent_obj, ...
                    'ParentFollowerNumber', size(parent_obj.RelativeFollower,2), ...
                    'UsedGeneralizedCoordinates', current_gen_coords, ...
                    'UsedControlInputs', current_gen_coords, ...
                    'DefaultJointOrientation', orientation_matrix);
            
            
            result = [result; body_obj];
            joints = [joints; new_joint];
        end
    end



    
    for i = 1:length(result)
        body =  result(i);
        if isempty(body.RelativeFollower)
            body.RelativeFollower = [0.5;0.5;0.5];
        end
    end

end

function ProperSize = GetJointInputsRequirements(JointType)
    switch JointType
        case {'none', 'fixed'}
            ProperSize = 0;
        case {'FloatingBase_6dof', 'FloatingBase_6dof_ZYX','floating'}
            ProperSize = 6;
        case {'abs_spherical', 'prismatic_XYZ', ...
              'planarX', 'planarY', 'planarZ', ...
              'abs_planarX', 'abs_planarY', 'abs_planarZ'}
            ProperSize = 3;
        case {'pivotXY', 'pivotYZ', 'pivotZX','pivotXZ'}
            ProperSize = 2;
        case {'pivotX', 'pivotY', 'pivotZ', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ', ...
                'prismaticX', 'prismaticY', 'prismaticZ'}
            ProperSize = 1;
        otherwise
            warning('Invalid joint type');
    end            
end

