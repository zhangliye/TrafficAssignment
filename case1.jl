#############################################################################
#  Copyright 2016, Liye Zhang and contributors
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at http://mozilla.org/MPL/2.0/.
#############################################################################
# Traffic assignment algorithm test #
#############################################################################
include( "network.jl" )
include( "demand.jl" )
include( "frank_wolfe_multiuser.jl" )

using GraphPlot
import Graphs

function input(prompt::String = "")
  print(prompt * " ")
  chomp(readline())
end
a = input("your name: ")
println(a)

function show_traffic_network( net_path::String )
  net = Network( net_path )
  read(STDIN, Char)

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

end

data_dir = splitdir( abspath( @__FILE__ ) )[1] * "\\data\\"
net_path = data_dir * "demo_net.csv"
show_traffic_network( net_path )
