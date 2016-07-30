#############################################################################
#  Copyright 2016, Liye Zhang and contributors
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at http://mozilla.org/MPL/2.0/.
#############################################################################
# traffic assignment test
#
#############################################################################

include("network.jl")
include("demand.jl")
include("frank_wolfe_multiuser.jl")
using Debug
using GraphPlot
import Graphs

function test()
	base_dir = "H:\\NUS_DISK_64G\\python_code\\julia_code\\traffic_network_design\\data"
	net_path = base_dir * "\\demo_net.csv"
	trip_path1 = base_dir * "\\demo_trips_pv.csv"
	trip_path2 = base_dir * "\\demo_trips_ev.csv"
	trip_paths = [trip_path1, trip_path2]

	network = Network( net_path )
	##########################################################
    # create network
    networks = create_networks( network )

    network1 = networks[1]   # PV
    network2 = networks[2]   # EV

    # test network
    #set_link_ev( network1, 4, 5 )
    #set_link_pv( network1, 4, 5 )
    set_link_ev( network1, 4, 5 )
    reverse_link( network2, 4, 5 )
    reverse_link( network1, 4, 5 )

    # prepare demand
    demand1 = Demand( trip_path1 )
	demand2 = Demand( trip_path2 )
	demands = Dict{Any, Any}( "1"=> demand1, "2" => demand2 )

	setup_time = time()

	xk, travel_time, obj, net = ta_frank_wolfe( network, network1, network2, demands, log=:on )

	run_time = time() - setup_time
	println("\nNetwork size:")
	println("Network TZ number: ", net.number_of_zones)
	println("Network node number: ", net.number_of_nodes)
	println("Total cost: ",  @sprintf("%.2f", obj) )
	println("Traffic assign running time: ", @sprintf("%.2f", run_time), " seconds" )

	println("\nTraffic flow of several links: ")
	from_nodes = start_nodes( net )
    to_nodes = end_nodes( net )
    for i in 1:length( from_nodes )
      println(from_nodes[i], "--", to_nodes[i], ": ", xk[i] )
    end

	println("finished")
end

test()
