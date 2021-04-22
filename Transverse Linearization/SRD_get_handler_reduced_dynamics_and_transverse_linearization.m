function Handler_reduced_dynamics_and_transverse_linearization = SRD_get_handler_reduced_dynamics_and_transverse_linearization(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__reduced_dynamics_and_transverse_linearization';
Parser.addOptional('description', []);
Parser.addOptional('UsePinv', false);
Parser.parse(varargin{:});

description = Parser.Results.description;

if isfield(description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_reduced_dynamics_and_transverse_linearization = SRDHandler_reduced_dynamics_and_transverse_linearization();
Handler_reduced_dynamics_and_transverse_linearization.State.description = description;

Handler_reduced_dynamics_and_transverse_linearization.N_dof = description.N_dof;
Handler_reduced_dynamics_and_transverse_linearization.c0 = description.c0;
Handler_reduced_dynamics_and_transverse_linearization.H0 = description.H0;
    
if Casadi

else
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

end