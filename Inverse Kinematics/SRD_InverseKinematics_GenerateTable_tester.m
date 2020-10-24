function SRD_InverseKinematics_GenerateTable_tester(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_InverseKinematics_GenerateTable';
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('TimeTable', []);
Parser.addOptional('IK_Table', []);
Parser.addOptional('tol', 10^(-5));

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);

condition_number_tape = zeros(Count, 1);
rank_tape = zeros(Count, 1);

for i = 1:Count
    
    q = reshape( Parser.Results.IK_Table(i, :), [], 1);
    J = Parser.Results.Handler_IK_Model.get_Jacobian(q);
    
    %condition_number_tape(i) = cond(J*J');
    condition_number_tape(i) = cond(J);
    rank_tape(i) = rank(J, Parser.Results.tol);
end

figure('Color', 'w', 'Name', 'GenerateTable tester 1')
subplot(2, 2, 1:2)
SRDgraphic_PlotGeneric(Parser.Results.TimeTable, condition_number_tape, ...
    'NewFigure', false, ...
    'Title', 'condition number J', ...
    'LableVariable', 'cond');
subplot(2, 2, 3:4)
SRDgraphic_PlotGeneric(Parser.Results.TimeTable, rank_tape, ...
    'NewFigure', false, ...
    'Title', 'rank J', ...
    'LableVariable', 'rank');

drawnow;

SRDgraphic_PlotGeneric(Parser.Results.TimeTable, Parser.Results.IK_Table, ...
    'NewFigure', true, ...
    'FigureName', 'GenerateTable tester 2', ...
    'Title', 'IK solution', ...
    'LableVariable', 'q');
end