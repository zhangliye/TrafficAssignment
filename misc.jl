
"""
TA_dijkstra_shortest_paths(graph, travel_time, origin, start_nodes, end_nodes)

# Arguments
* graph: DiGraph
* travel_time: Array( link_amount )
* origin: int, start node number to find shortest path
* start_nodes: Array(link_number), start nodes of all links
* end_nodes: Array(link_number), end nodes of all links

# Return: DijkstraState object
    type DijkstraState{T}<: AbstractDijkstraState
    	parents::Vector{Int}
    	dists::Vector{T}
    	predecessors::Vector{Vector{Int}}
    	pathcounts::Vector{Int}
	end

"""
function TA_dijkstra_shortest_paths(graph, travel_time, origin, start_nodes, end_nodes)

    no_node = nv( graph )
    no_arc = ne( graph )

    # the index of row and col of distance matrix is the number of start and end node number
    #
    distmx = Inf*ones(no_node, no_node)   # distance matrix between all nodes
  	for i in 1:no_arc
  		distmx[start_nodes[i], end_nodes[i]] = travel_time[i]
  	end

    state = dijkstra_shortest_paths(graph, origin, distmx)
    return state
end

"""
create_graph(start_nodes, end_nodes)

Create a DiGraph object
"""
function create_graph(start_nodes, end_nodes)
    @assert Base.length(start_nodes)==Base.length(end_nodes)

    no_node = max( maximum(start_nodes), maximum(end_nodes) )
    no_arc = Base.length( start_nodes )

    graph = DiGraph( no_node )
    for i=1:no_arc
        add_edge!(graph, start_nodes[i], end_nodes[i])
    end
    return graph
end


"""
get_vector(state, origin, destination, link_dic)

Get the node IDs on the shortest route

# Arguments
* `state`: DistraryState object
* `origin`: int, from node ID
* `destination`: int, to node ID
* `link_dic`: # sparse matrix, (node_max, node_max), [from, to] = 1, 2, ...number_of_links

# Return: #vector, length -- number of links, [0, 0, 1, 0, 1,...], on the shortest path, link is marked with 1
"""
function get_vector(state, origin, destination, link_dic)
    current = destination
    parent = -1
    x = zeros(Int, maximum(link_dic))   #vector, length -- number of links

    # from end to origin, mark the link on the shortest path with 1
    while parent != origin && origin != destination
        parent = state.parents[current]

        link_idx = link_dic[parent, current]  # get the index of the link, from:to, 1, 2,...link_mumber
        if link_idx != 0
            x[link_idx] = 1
        end

        current = parent
    end

    return x
end
