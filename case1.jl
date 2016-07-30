include( "network.jl" )
include( "demand.jl" )
include( "frank_wolfe_multiuser.jl" )

using GraphPlot
import Graphs

net_path = "D:\\liye_360syn\\syn_develop\\julia_code\\traffic_network_mt\\data\\case1_net.csv"

net = Network( net_path )

# show result
#
edges = []
from_nodes = start_nodes( net )
to_nodes = end_nodes( net )
edge_labels = []
for i in 1:length( from_nodes )
  push!( edges, (from_nodes[i], to_nodes[i]) )
end

println("here")
# create graph and add edges
g = Graphs.simple_graph( length(get_vertices(net))  )
for (from_node, to_node) in edges
  Graphs.add_edge!( g, from_node, to_node )
end

nodelabel = [ 1:length( get_vertices(net) ) ]
locs_x = [0., 5, 5, 10]
locs_y = [5., 0, 5, 8]

gplot( g, locs_x, locs_y, nodelabel=nodelabel )

println(" good ")
