using Distributions, JuMP, Gurobi, ProgressMeter

gurobi_env = Gurobi.Env()

p = 1.0 #day ahead price
q = 1.5 #same day price

#array de vectores de cuts. Arranca en la lower bound
cuts = [[0.0;0.0]];

#aca guardo solo los multiplicadores
duals = Set{Float64}();

#aca guardo los ruidos que fueron saliendo
noises = Array{Float64}(undef,0);

#condicion inicial de stock
x0=0.0;

@showprogress 1 "Computing..." for l=1:100
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

    #resuelvo para un ruido en el segundo paso y agrego un corte
    demand=rand(DiscreteUniform(0,100));
    push!(noises,demand)


    model = JuMP.Model(with_optimizer(Gurobi.Optimizer,OutputFlag=0,gurobi_env))

    @variable(model,shortage>=0);
    @variable(model,stock>=0);

    fix_x = @constraint(model,x-demand+shortage>=0);

#    fix_x = @constraint(model,stock==x);

    @objective(model,Min,q*shortage);

    optimize!(model)

    beta = objective_value(model);
    #cambio el signo del multiplicador por que es >= en la constraint
    lambda = dual(fix_x)

    push!(duals,lambda)

    #update all previous cuts for averaging
    for j=1:length(cuts)
        cuts[j] = (l-1)/l * cuts[j];
    end

    pik = similar(noises);
    #genero el new cut recorriendo todos los ruidos
    for j=1:length(noises)
        aux = [(pi*(noises[j]-x),pi) for pi in duals]
        _, ix = findmax([a[1] for a in aux])
        pik[j] = aux[ix][2];
    end

    new_cut  = 1/l * [ sum(pik.*noises); -sum(pik) ];

    push!(cuts,new_cut);
    println(cuts)
    println(x)

end


#resuelvo una vez mas para hallar el costo optimo
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
costo = objective_value(model);
