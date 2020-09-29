function Handler_IK_task = SRD_get_handler__IK_task__splines(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_IK_task_handler';
Parser.addOptional('ZeroOrderDerivativeNodes', []);
Parser.addOptional('FirstOrderDerivativeNodes', []);
Parser.addOptional('SecondOrderDerivativeNodes', []);
Parser.addOptional('NodeTimes', []);

Parser.addOptional('OutOfBoundariesBehaviour', 'LastValue');

Parser.parse(varargin{:});

Spline = TPSplineConstructorUI();
Spline.OutOfBoundariesBehaviour = Parser.Results.OutOfBoundariesBehaviour;
Spline.GenerateSplines(Parser.Results.NodeTimes, ...
    Parser.Results.ZeroOrderDerivativeNodes, ...
    Parser.Results.FirstOrderDerivativeNodes, ...
    Parser.Results.SecondOrderDerivativeNodes);

Handler_IK_task = SRDHandler_IK_task;

Handler_IK_task.State.Spline = Spline;
Handler_IK_task.TimeStart = Parser.Results.NodeTimes(1);
Handler_IK_task.TimeExpiration = Parser.Results.NodeTimes(end);

Handler_IK_task.get_Task = Handler_IK_task.State.Spline.get_EvaluateQ_handle;
Handler_IK_task.get_Task_derivative = Handler_IK_task.State.Spline.get_EvaluateV_handle;
Handler_IK_task.get_Task_second_derivative = Handler_IK_task.State.Spline.get_EvaluateA_handle;
end