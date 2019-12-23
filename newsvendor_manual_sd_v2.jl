using Distributions, JuMP, Gurobi, ProgressMeter

gurobi_env = Gurobi.Env()

p = 1.0 #day ahead price
q = 1.5 #same day price

#array de vectores de cuts. Arranca en la lower bound
cuts = [[0.0;0.0]];

#aca guardo solo los multiplicadores
#duals = Set{Float64}();

#aca guardo los ruidos que fueron saliendo
noises = Array{Float64}(undef,0);
objective_values = Array{Float64}(undef,0);
explored_states = Array{Float64}(undef,0);
duals = Array{Float64}(undef,0);


#condicion inicial de stock
x0=0.0;

@showprogress 1 "Computing..." for l=1:1000
    global cuts

    #resuelvo el primer paso
    model = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

    @variable(model,reserve>=0);
    @variable(model,stock>=0);
    @variable(model,z);

    cut_constraints = [];
    for k=1:length(cuts)
        cut=cuts[k]
        c=@constraint(model,z>=cut[1]+cut[2]*(stock+reserve));
        push!(cut_constraints,c);
    end

    fix_x = @constraint(model,stock==x0);

    @objective(model,Min,p*reserve+z);

    optimize!(model)

    #purge cuts!
    #multipliers = [dual(c) for c in cut_constraints];
    #idx = filter(k->multipliers[k]!=0,(1:length(cuts)));
    #cuts = cuts[idx];


    x=value(stock)+value(reserve);
    push!(explored_states,x)

    #resuelvo para un ruido en el segundo paso y agrego un corte
    demand=rand(DiscreteUniform(0,100));
    push!(noises,demand)


    model = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

    @variable(model,shortage>=0);
    @variable(model,stock_in>=0);

    @variable(model,stock_out>=0);

    @constraint(model,stock_in-demand+shortage==stock_out);

    fix_x = @constraint(model,x==stock_in);

    @objective(model,Min,q*shortage);

    optimize!(model)

    beta = objective_value(model);

    push!(objective_values,beta)

    #cambio el signo del multiplicador por que es >= en la constraint
    lambda = dual(fix_x)

    push!(duals,lambda)


    ##problema auxiliar para hallar el beta estimado de los cortes anteriores.
    aux = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

    @variable(aux, z2>=0)
    @variable(aux, x2>=0)

    cut_constraints = [];

    for k=1:length(cuts)
        cut=cuts[k]
        c=@constraint(aux,z2>=cut[1]+cut[2]*(x2));
        push!(cut_constraints,c);
    end
    @objective(aux,Min,z2)

    fix_x2 = @constraint(aux,x==x2)

    optimize!(aux)

    beta_est = objective_value(aux)
    lambda_est = dual.(fix_x2)



    #update all previous cuts for averaging
    for j=1:length(cuts)
        cuts[j] = (l-1)/(l) * cuts[j];
    end




    new_cut = 1/(l)*[beta+lambda*x;-lambda]+(l-1)/(l)*[beta_est+lambda_est*x;-lambda_est]

    push!(cuts,new_cut);
end

#
# #resuelvo una vez mas para hallar el costo optimo
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
