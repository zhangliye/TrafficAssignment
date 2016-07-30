#############################################################################
#  Copyright 2016, Liye Zhang and contributors
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at http://mozilla.org/MPL/2.0/.
#############################################################################
# Frank-Wolfe Methods
# CFW and BFW in Mitradijieva and Lindberg (2013)
# 
#############################################################################



# required packages: Graphs, Optim
using LightGraphs, Optim
import Base.LinAlg.gradient

include("misc.jl")
include("demand.jl")
include("network.jl")

"""
    create network according to user type
        change capacity and free travel time
	return: {name: Network, ...}
"""
function create_networks( network )

   networks = Dict{AbstractString, Network}()

   # network for PV, 1 for EV, else 0
   #
   network1 = deepcopy( network )
   for k in 1:length(network1.capacity)
     if contains(network1.link_permission[k], "1")
       network1.capacity[k] = 0
       network1.free_flow_time[k] = Inf
     end
   end

   # for EV
   network2 = deepcopy( network )
   return ( network1, network2 )
end

"""
BPR( x, net )

Calculate the cost of every link

# Parameters
* `x`: [ flow1, flow2... flow_link_index ]
* `net`: Network object

# Return: costs[ cost1, cost2, ..., cost_link_index ]
"""
function BPR( x, net )
    # travel_time = free_flow_time.* ( 1.0 + B .* (x./capacity).^power )
    # generalized_cost = travel_time + toll_factor *toll + distance_factor * link_length
    # return generalized_cost

    bpr = similar( x )
    for i=1:length( bpr )
        bpr[i] = net.free_flow_time[i] * ( 1.0 + net.B[i] * (x[i]/net.capacity[i])^net.power[i] )
        #bpr[i] += net.toll_factor *net.toll[i] + net.distance_factor * net.link_length[i]
    end
    return bpr
end

"""
# Paramters
* `x`: total traffic flow
* `net`: must be the ev users
"""
function objective( x1, net1, x2, net2 )
    # value = free_flow_time .* ( x + B.* ( x.^(power+1)) ./ (capacity.^power) ./ (power+1))
    # return sum(value)

    sum = 0
    x = x1
    net = net1
    for i=1:length( x )
        sum += net.free_flow_time[i] * ( x[i] + net.B[i]* ( x[i]^(net.power[i]+1)) / (net.capacity[i]^net.power[i]) / (net.power[i]+1))
        sum += net.toll_factor *net.toll[i] + net.distance_factor * net.link_length[i]
    end

    x = x2
    net = net2
    for i=1:length( x )
        sum += net.free_flow_time[i] * ( x[i] + net.B[i]* ( x[i]^(net.power[i]+1)) / (net.capacity[i]^net.power[i]) / (net.power[i]+1))
        sum += net.toll_factor *net.toll[i] + net.distance_factor * net.link_length[i]
    end
    return sum
end

function objective( x, net )
    # value = free_flow_time .* ( x + B.* ( x.^(power+1)) ./ (capacity.^power) ./ (power+1))
    # return sum(value)

    sum = 0
    for i=1:length( x )
        sum += net.free_flow_time[i] * ( x[i] + net.B[i]* ( x[i]^(net.power[i]+1)) / (net.capacity[i]^net.power[i]) / (net.power[i]+1))
        sum += net.toll_factor *net.toll[i] + net.distance_factor * net.link_length[i]
    end
    return sum
end

function gradient(x, net)
    return BPR(x, net)
end

function hessian( x, net )
    no_arc = Base.length( net.start_nodes )

    h = zeros( no_arc, no_arc )
    h_diag = hessian_diag( x )

    for i=1:no_arc
        h[i,i] = h_diag[i]
    end

    return h

    #Link travel time = free flow time * ( 1 + B * (flow/capacity)^Power ).
end

function hessian_diag(x, net)
    h_diag = Array(Float64, size(x))
    for i=1:length(x)
        if net.power[i] >= 1.0
            h_diag[i] = net.free_flow_time[i] * net.B[i] * net.power[i] * (x[i]^(net.power[i]-1)) / (net.capacity[i]^net.power[i])
        else
            h_diag[i] = 0 # Some cases, power is zero.
        end
    end
    # h_diag = free_flow_time .* B .* power .* (x.^(power-1)) ./ (capacity.^power)

    return h_diag
    #Link travel time = free flow time * ( 1 + B * (flow/capacity)^Power ).
end

"""
# Arguments
* `travel_time`: [cost1, cost2, ...cost_link_index]

# Return: Array( link_number ), traffic flow on the links, traffic flow index 1:link_number
"""
function all_or_nothing_single( travel_time, net, demand, graph, link_dic )
    state = []
    path = []
    x = zeros( size(start_nodes(net)) )

    for r=1 : size(demand.travel_demand)[1]
        # for each origin node r, find shortest paths to all destination nodes
        state = TA_dijkstra_shortest_paths(graph, travel_time, r, start_nodes(net), end_nodes(net))

        for s=1 : size( demand.travel_demand )[2]
            # for each destination node s, find the shortest-path vector
            # load travel demand
            x = x + demand.travel_demand[r,s] * get_vector(state, r, s, link_dic)
        end
    end

    return x
end

"""
all_or_nothing_parallel( travel_time )

parallel computing version
"""
function all_or_nothing_parallel( travel_time )
    state = []
    path = []
    vv = zeros(size(start_nodes))
    x = zeros(size(start_nodes))

    x = x + @parallel (+) for r=1:size(travel_demand)[1]
    # for each origin node r, find shortest paths to all destination nodes
    # if there is any travel demand starting from node r.
    vv = zeros( size(start_nodes) )

        if sum(travel_demand, 2)[r] > 0.0
            state = TA_dijkstra_shortest_paths(graph, travel_time, r, start_nodes, end_nodes)

            for s=1:size(travel_demand)[2]
                # for each destination node s, find the shortest-path vector
                # v = get_vector(state, r, s, start_nodes, end_nodes)

                if travel_demand[r,s] > 0.0
                    # load travel demand
                    vv = vv + travel_demand[r,s] * get_vector(state, r, s, link_dic)
                end
            end

        end

        vv
    end

    return x
end

"""
all_or_nothing( travel_time )

0-1 traffic assignment

# Arguments
* `travel_time`: [cost1, cost2, ...cost_link_index]

# Return: Array( link_number ), traffic flow on the links, traffic flow index 1:link_number
"""
function all_or_nothing( travel_time, network, demand, graph, link_dic )
	if nprocs() > 1 # if multiple CPU processes are available
	    all_or_nothing_parallel( travel_time )
	else
	    all_or_nothing_single( travel_time, network, demand, graph, link_dic )
	    # when nprocs()==1, using @parallel just adds unnecessary setup time. I guess.
	end
end


"""
ta_frank_wolfe(network, od_demands; method=:bfw, max_iter_no=2000, step=:exact, log=:off, tol=1e-3)

Assign OD matrix to route traffic

# Arguments
* `network`: TA_Data object
* `od_demands`: od_demands::Dict{ASCIIString, Demand}
* `max_iter_no`: max iteration number
* `tol`: convergence bound

# Returns
* `xk`: Array( link_number ), traffic flow on the links, traffic flow index 1:link_number
* `travel_time`: [cost1, cost2, ...cost_link_index], link real travel time calculated with BPR function
* `objective`: Float, total cost including time and money
"""
function ta_frank_wolfe(network, network1, network2, od_demands; method=:fw, max_iter_no=50000, step=:exact, log=:off, tol=1e-3)

    setup_time = time()

    if log==:on
        println("-------------------------------------")
        println("Network Name: $(network1.network_name)")
        println("Method: $method")
        println("Line Search Step: $step")
        println("Maximum Interation Number: $max_iter_no")
        println("Tolerance for AEC: $tol")
        println("Number of processors: ", nprocs())
    end

    # demand data
    demand1 = od_demands["1"]
    demand2 = od_demands["2"]

    ###########################################################################
    # preparing a graph
    #
    graph1 = create_graph( start_nodes(network1), end_nodes(network1) )
    link_dic1 = sparse( start_nodes( network1 ), end_nodes( network1), 1:get_link_number(network1) )  # sparse matrix, [from, to] = 1, 2, ...number_of_links

    graph2 = create_graph( start_nodes(network2), end_nodes(network2) )
    link_dic2 = sparse( start_nodes(network2), end_nodes(network2), 1 : get_link_number(network2) )  # sparse matrix, [from, to] = 1, 2, ...number_of_links

    setup_time = time() - setup_time
    if log==:on
        println("Setup time = $setup_time seconds")
    end

    spp_total = 0
    vvv_total = 0

    iteration_time = time()

    ################ step 0, All-or-nothing assignment #########################
    # Finding a starting feasible solution
    number_of_links = get_link_number( network1 )
    travel_time1 = BPR( zeros(number_of_links), network1 )
    travel_time2 = BPR( zeros(number_of_links), network2 )

    x0_1 = all_or_nothing( travel_time1, network1, demand1, graph1, link_dic1 )  # Array( link_number ), traffic flow on the links
    x0_2 = all_or_nothing( travel_time2, network2, demand2, graph2, link_dic2 )  # Array( link_number ), traffic flow on the links

    # Initializing variables
    xk = x0_1 + x0_2   # traffic flow of links, Array{Float64}(link_number)
    xk1 = x0_1
    xk2 = x0_2
    tauk = 0           # alpha for FW direction
    yk_FW = xk         # auxiliary flow, Array{Float64}(link_number), for finding direction, step 2

    sk_CFW = yk_FW
    Hk_diag = []

    dk_FW = []
    dk_bar = []
    dk_CFW = []
    dk = []
    dk1 = []       # x on G1
    dk2 = []       # x on G2

    alphak = 0
    Nk = 0
    Dk = 0

    tauk = 0
    is_first_iteration = false
    is_second_iteration = false

    sk_BFW = yk_FW
    sk_BFW_old = yk_FW

    dk_bbar = []
    muk = []
    nuk = []
    beta0 = 0
    beta1 = 0
    beat2 = 0

    """ linear function to find direction of FW
    """
    function fk( tau )
        #value = objective( xk1 + tau*dk1, network1, xk2 + tau*dk2, network2 )      # all the traffic flow are calculated accroding the EV network
        value = objective( xk + tau*dk, network )   # must be EV network, or ev lanes will be 0
        return value
    end

    function lower_bound_k( x, xk )
        value = objective(xk) + dot( BPR(xk), ( x - xk) )
    end

    for k=1:max_iter_no
        ####################### step 1, update link time ##############################
        # Finding yk
        travel_time1 = BPR( xk, network1 )
        travel_time2 = BPR( xk, network2 )

        ####################### step 2, Find auxiliary flow ###########################
        yk_FW1 = all_or_nothing(travel_time1, network1, demand1, graph1, link_dic1)   # Array( link_number ), traffic flow on the links
        yk_FW2 = all_or_nothing(travel_time2, network2, demand2, graph2, link_dic2)   # Array( link_number ), traffic flow on the links

        # merge X and update X for two networks
        yk_FW = yk_FW1 + yk_FW2

        ####################### step 3, Line search, Find alpha_n ######################
        # Basic Frank-Wolfe Direction
        dk_FW1 = yk_FW1 - xk1
        dk_FW2 = yk_FW2 - xk2
        dk_FW = yk_FW - xk

        # Finding a feasible direction
        if method == :fw # Original Frank-Wolfe
            dk1 = dk_FW1
            dk2 = dk_FW2
            dk = dk_FW
        else
            error("The type of Frank-Wolfe method is specified incorrectly.")
        end
        # dk is now identified.

        # Line Search from xk in the direction dk
        optk = optimize(fk, 0.0, 1.0, method = GoldenSection() )
        tauk = optk.minimum

        # Average Excess Cost
        travel_demand = demand1.travel_demand + demand2.travel_demand
        average_excess_cost = ( dot(xk, travel_time2) - dot(yk_FW, travel_time2) ) / sum( travel_demand )
        if log==:on
            println("k=$k,\ttauk=$tauk,\tobjective=$(objective(xk, network2)),\taec=$average_excess_cost")
        end

        # rel_gap = ( objective(xk) - best_objective ) / best_objective

        # Convergence Test
        if average_excess_cost < tol
        # if rel_gap < tol
            break
        end

        # Update x
        xk1 = xk1 + tauk*dk1
        xk2 = xk2 + tauk*dk2

        new_x = xk + tauk*dk
        xk = new_x

        @assert minimum(xk) >= 0

    end

    iteration_time = time() - iteration_time

    if log==:on
        println("Iteration time = $iteration_time seconds")
    end

    println( "private vehicle (mins): ", dot(xk1, travel_time2) )
    println( "emergency vehicle (mins): ", dot(xk2, travel_time2) )

    return xk, travel_time2, objective(xk, network2), network2

end
