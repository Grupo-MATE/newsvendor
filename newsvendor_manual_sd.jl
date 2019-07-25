using Distributions, JuMP, Gurobi

p = 1.0 #day ahead price
q = 2.0 #same day price

#array de vectores de cuts. Arranca en la lower bound
cuts = [[0.0;0.0]];

#condicion inicial de stock
x0=0.0;

for l=1:100
    #resuelvo el primer paso
    model = JuMP.Model(with_optimizer(Gurobi.Optimizer))

    @variable(model,reserve>=0);
    @variable(model,stock>=0);
    @variable(model,z);

    for k=1:length(cuts)
        cut=cuts[k]
        @constraint(model,z>=cut[1]+cut[2]*(stock+reserve));
    end

    fix_x = @constraint(model,stock==x0);

    @objective(model,Min,p*reserve+z);

    optimize!(model)

    x=value(stock)+value(reserve);

    #resuelvo para todos un ruido en el segundo paso y agrego un corte
    demand=rand(DiscreteUniform(0,100));

    local_cuts = [];

    model = JuMP.Model(with_optimizer(Gurobi.Optimizer))

    @variable(model,shortage>=0);
    @variable(model,stock>=0);

    @constraint(model,stock-demand+shortage>=0);

    fix_x = @constraint(model,stock==x);

    @objective(model,Min,q*shortage);

    optimize!(model)

    beta = objective_value(model);
    lambda = dual(fix_x)

    new_cut = [beta-lambda*x;lambda]

    #update all cuts for averaging

    for j=1:length(cuts)
        cuts[j] = 0.9 * cuts[j];
    end

    push!(cuts,new_cut);

end


#resuelvo una vez mas para hallar el costo optimo
model = JuMP.Model(with_optimizer(Gurobi.Optimizer))

@variable(model,reserve>=0);
@variable(model,stock>=0);
@variable(model,z);

for i=1:length(cuts)
    cut=cuts[i]
    @constraint(model,z>=cut[1]+cut[2]*(stock+reserve));
end

fix_x = @constraint(model,stock==x0);

@objective(model,Min,p*reserve+z);

optimize!(model)

x=value(stock)+value(reserve);
costo = objective_value(model);
