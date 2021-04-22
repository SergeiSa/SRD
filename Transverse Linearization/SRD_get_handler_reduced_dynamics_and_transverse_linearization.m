function Handler_reduced_dynamics_and_transverse_linearization = SRD_get_handler_reduced_dynamics_and_transverse_linearization(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__reduced_dynamics_and_transverse_linearization';
Parser.addOptional('description', []);
Parser.addOptional('UsePinv', false);
Parser.parse(varargin{:});


% if isfield(Parser.Results.description, 'Casadi_cfile_name')
%     Casadi = true;
% else
%     Casadi = false;
% end

Handler_reduced_dynamics_and_transverse_linearization = SRDHandler_reduced_dynamics_and_transverse_linearization();
Handler_reduced_dynamics_and_transverse_linearization.State.description = Parser.Results.description;

Handler_reduced_dynamics_and_transverse_linearization.N_dof = Parser.Results.description.N_dof;
Handler_reduced_dynamics_and_transverse_linearization.c0 = Parser.Results.description.c0;
Handler_reduced_dynamics_and_transverse_linearization.H0 = Parser.Results.description.H0;
    

Handler_reduced_dynamics_and_transverse_linearization.SerializationPrepNeeded = true;
Handler_reduced_dynamics_and_transverse_linearization.PreSerializationPrepFunction = @PreSerializationPrepFunction;
Handler_reduced_dynamics_and_transverse_linearization.PostSerializationPrepFunction = @PostSerializationPrepFunction_m;



    function PostSerializationPrepFunction_m(Handler_reduced_dynamics_and_transverse_linearization)
        
        description = Handler_reduced_dynamics_and_transverse_linearization.State.description;
        
        if ~isempty(description.Path)
            current_dir = pwd;
            cd(description.Path);
        end
        
        %H*ddq + C*dq + g = T*u
        Handler_reduced_dynamics_and_transverse_linearization.get_alpha = str2func(description.FunctionName_alpha);
        Handler_reduced_dynamics_and_transverse_linearization.get_beta = str2func(description.FunctionName_beta);
        Handler_reduced_dynamics_and_transverse_linearization.get_gamma = str2func(description.FunctionName_gamma);
        Handler_reduced_dynamics_and_transverse_linearization.get_Uff = str2func(description.FunctionName_Uff);
        Handler_reduced_dynamics_and_transverse_linearization.get_Nff = str2func(description.FunctionName_Nff);
        Handler_reduced_dynamics_and_transverse_linearization.get_A = str2func(description.FunctionName_A);
        Handler_reduced_dynamics_and_transverse_linearization.get_B = str2func(description.FunctionName_B);
        
        if ~isempty(description.Path)
            cd(current_dir);
        end
    end

%     function PostSerializationPrepFunction_casadi(Handler_reduced_dynamics_and_transverse_linearization)
%         disp('TBD')
%     end


    function PreSerializationPrepFunction(Handler_reduced_dynamics_and_transverse_linearization)
        Handler_reduced_dynamics_and_transverse_linearization.get_alpha = [];
        Handler_reduced_dynamics_and_transverse_linearization.get_beta = [];
        Handler_reduced_dynamics_and_transverse_linearization.get_gamma = [];
        Handler_reduced_dynamics_and_transverse_linearization.get_Uff = [];
        Handler_reduced_dynamics_and_transverse_linearization.get_Nff = [];
        Handler_reduced_dynamics_and_transverse_linearization.get_A = [];
    end

end