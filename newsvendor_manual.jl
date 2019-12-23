using Distributions, JuMP, Gurobi

gurobi_env = Gurobi.Env()

p = 1.0 #day ahead price
q = 1.5 #same day price

#array de vectores de cuts. Arranca en la lower bound
cuts = [[0.0;0.0]];

#condicion inicial de stock
x0=0.0;

for i=1:100
    #resuelvo el primer paso
    model = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

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

#demand=rand(DiscreteUniform(0,100));
    #resuelvo para todos los ruidos el segundo paso y agrego un corte
    local_cuts = [];

    for demand=1:100


        model = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

        @variable(model,shortage>=0);
        @variable(model,stock>=0);

        fix_x = @constraint(model,x-demand+shortage>=0);

#        fix_x = @constraint(model,stock==x);

        @objective(model,Min,q*shortage);

        optimize!(model)

        beta = objective_value(model);
        lambda = -dual(fix_x)

        push!(local_cuts,[beta-lambda*x;lambda])

    end

    new_cut = 1/100*[sum([c[1] for c in local_cuts]);sum([c[2] for c in local_cuts])];
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
