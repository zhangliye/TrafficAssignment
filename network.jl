# Network extended from the following
# Sioux Falls network data
# http://www.bgu.ac.il/~bargera/tntp/
#
# user type- 1 for "pv", 2 for 'ev', permision property of the link is as '1-4-6'
# This network structure can

# Link travel time = free flow time * ( 1 + B * (flow/capacity)^Power ).
# Link generalized cost = Link travel time + toll_factor * toll + distance_factor * distance
# Data format:
# All traffic zone node start from 1:number_OD_zones
# Suppose all node number start from 1 to `max_node_index`
using Debug
import Base.print

# Traffic Assignment Data structure
type Network

    ############# data from file ############################
    # data from trip file head
    #
    number_of_zones::Int64    # number of the OD nodes, 1:number_OD_zones, n, for OD matrix
    number_of_nodes::Int64    # all other nodes from n+1: --> N
    first_thru_node::Int64    # In the some networks (like Sioux-Falls) it is equal to 1,
                              # indicating that traffic can move through all nodes, including zones.
                              # In other networks when traffic is not allow to go through zones, the zones are
                              # numbered 1 to n and the <FIRST THRU NODE> is set to n+1.
    #number_of_links::Int64   # number of road links, size of Array for all the following link data
                              # for start_nodes and end_nodes length

    ############################################################
    # data from trip file table
    #
    physical_links::Array{Tuple{Int64, Int64}}    # ( small_node, large_node ), every item for one physical lane
    directions::Array{Int64}                      # 1, from small to larger node; else 0
    lane_indicator::Array{Int64}                  # 1 for EV only, others 0

    ############################################################
    # parameter of physical links
    #
    capacity::Array{Float64}
    link_length::Array{Float64}       # only used for calculating toll and distance cost
    free_flow_time::Array{Float64}
    physical_free_flow_time::Array{Float64}
    B::Array{Float64}
    power::Array{Float64}
    speed_limit::Array{Float64}
    toll::Array{Float64}
    link_type::Array{Float64}                 # for road type
    link_permission::Array{AbstractString}    # 11, user type
    lane_num::Array{Int64}
    lane_capacity::Array{Float64}
    X::Array{Float64}

    ###########################################################
    # other data
    #
    toll_factor::Float64         # convert toll to additional general cost, 0 default not consider
    distance_factor::Float64     # convert distance to additional general cost, 0 default not consider
    best_objective::Float64      #

    network_name::AbstractString

    ###############################################
    #
    #link_pair_indexes = Set{Tuple{Int64, Int64}}      #

    # default class construction function
    #
    function Network()
		self = new()

        self.number_of_zones = 0    # number of the OD nodes, 1:number_OD_zones, n, for OD matrix
        self.number_of_nodes = 0
		self.first_thru_node = 0

		self.physical_links = Array{Tuple{Int64, Int64}}(0)    # ( small_node, large_node ), every item for one physical lane
        self.directions  = Array{Int64}(0)                     # 1, from small to larger node; else 0
        self.lane_indicator = Array{Int64}(0)

		# virtual and real lanes
		self.capacity = Array{Float64}(0)
		self.link_length = Array{Float64}(0)         # only used for calculating toll and distance cost
		self.free_flow_time = Array{Float64}(0)
		self.physical_free_flow_time = Array{Float64}(0)
		self.B = Array{Float64}(0)
		self.power =  Array{Float64}(0)
		self.speed_limit = Array{Float64}(0)
		self.toll = Array{Float64}(0)
		self.link_type = Array{Float64}(0)           # for road type
		self.link_permission = Array{AbstractString}(0)     # 11, user type
        self.lane_num = Array{Int64}(0)              # 12 for road type
        self.lane_capacity = Array{Float64}(0)       # 13 for road type
        self.X = Array{Float64}(0)

	    ###############################################
	    # other data
	    #
	    self.toll_factor = 0.         # convert toll to additional general cost, 0 default not consider
	    self.distance_factor = 0.     # convert distance to additional general cost, 0 default not consider
	    self.best_objective = 0.

	    self.network_name = "test_network"

	    return self
    end

end

###################################################################

"""
count sub strings in string array

# Arguments
* str: string to cound
* str_list: Array{String}, eg. ["ab", "ed"]

# Return: number of the string
"""
function count_str( str_cnt, str_list )
    n = 0
    for str in str_list
        if str_cnt == str
            n += 1
        end
    end

    return n
end

""" Change link capacity
"""
function changeCapacity( net::Network, link_index, capacity )

end

"""
  get the user

  Arguments
  user_type: "ae-ce-ae"
  Return: true for valid, else is false
"""

function is_user_type_valid( user_type )
type_names = split( user_type, "-")
    if length( type_names ) < 1
        return false
    end

    # check empty string
    if length( type_names ) == 1
        if type_names[1]==""
            return false
        end
    end

    # check repeated user type
    for name in type_names
        if count_str(name, type_names) != 1
            return false
        end
    end

    return true
end

"""
	Read traffic network files and trip table, create a data structure

	# Arguments
	* `network_file`: network file full path
	* `trip_file`: trip file full path

	# return: (TA_Data, od_demands), od_demands::Dict{ASCIIString, Demand}

	# Examples
    # network_file.txt
<NUMBER OF ZONES> 38
<NUMBER OF NODES> 416
<FIRST THRU NODE> 39		#
<NUMBER OF LINKS> 914		# a space before and after the number
<END OF METADATA>

       # it should be noted that, there is a two space character
#    1      2       3                   4             5                     6   7       8               9       10      11

~ 	Tail	Head	Capacity (veh/h)	Length (ft)	  Free Flow Time (min)	B	Power	Speed (ft/min) 	Toll 	Type	Permission;
    1	    117	    9000	            5280	      1.090458488	        0.15  4	    4842	         0	     1	1;
    2	    87	    9000	            5280	      1.090458488	        0.15  4	    4842	         0	     1	1;


# trip_file.txt

<NUMBER OF ZONES> 38
<TOTAL OD FLOW>  104694.40
<USER TYPE> 2                # Road user type, For example, 1- private car, 2 - emergency vehicle
<END OF METADATA>


Origin 1
    2 :    1365.90;    3 :     407.40;    4 :     861.40;    5 :     354.40;    6 :     545.10;
    7 :     431.50;    8 :       1.00;    9 :      56.80;   10 :      75.30;   11 :       1.00;
"""
function Network( network_data_file, network_name="traffic network" )

    network = Network()
    network.network_name = network_name

    t = isfile( network_data_file )
    n = open(network_data_file, "r")  # return a file stream, IOStream

    while (line=readline(n)) != ""
        if contains(line, "<NUMBER OF ZONES>")
            number_of_zones = parse(Int, line[ search(line, '>')+1 : end-1 ] )
            network.number_of_zones = number_of_zones
        elseif contains(line, "<NUMBER OF NODES>")
            number_of_nodes = parse(Int, line[ search(line, '>')+1 : end-1 ] )
            network.number_of_nodes = number_of_nodes
        elseif contains(line, "<FIRST THRU NODE>")
            first_thru_node = parse(Int, line[ search(line, '>')+1 : end-1 ] )
            network.first_thru_node = first_thru_node
        elseif contains(line, "<NUMBER OF LINKS>")
            number_of_links = parse(Int, line[ search(line, '>')+1 : end-1 ] )
        elseif contains(line, "<END OF METADATA>")
            break
        end
    end

    # Read all rows of link data
    #
    idx = 1
    while (line=readline(n)) != ""
        if contains(line, "~")  #skip the head line
            continue
        end

        if contains(line, ";")
            line = strip(line, '\n')
            line = strip(line, '\r')
            line = strip(line, ';')

            numbers = split(line, ",")

            # save physical lane
            lane_num = parse( Int, numbers[12] )
            start_node = parse(Int, numbers[1])
	        end_node = parse(Int, numbers[2])
	        capacity = parse(Float64, numbers[3])
            link_length = parse(Float64, numbers[4])
            free_flow_time = parse(Float64, numbers[5])
            B = parse(Float64, numbers[6])
            power = parse(Float64, numbers[7])
            speed_limit = parse(Float64, numbers[8])
            toll = parse(Float64, numbers[9])
            link_type = parse(Int, numbers[10])
            link_permission = strip( numbers[11] )
            @assert is_user_type_valid( link_permission ) == true
            lane_capacity = parse( Int, numbers[13] )

            if lane_num == 1
                addPhysicalLane( network, start_node, end_node, capacity, link_length, free_flow_time, B, power, speed_limit, toll, link_type, link_permission, lane_capacity )
            elseif lane_num > 1
                capacity = lane_capacity
                for i in 1:lane_num
	                network.number_of_nodes += 1
	                virtual_head = network.number_of_nodes
	                network.number_of_nodes += 1
	                virtual_tail = network.number_of_nodes
	                add_supper_line( network, start_node, virtual_head )
	                addPhysicalLane( network, virtual_head, virtual_tail, capacity, link_length, free_flow_time, B, power, speed_limit, toll, link_type, link_permission, lane_capacity )
	                add_supper_line( network, virtual_tail, end_node )
                end
            end

            idx = idx + 1
        end
    end

    return network

end # end of load_network function

function addPhysicalLinkMark(network, start_node, end_node)
    if start_node < end_node
      push!( network.physical_links, (start_node, end_node) )
      push!( network.directions, 1 )
    else
      push!( network.physical_links, (end_node, start_node) )
      push!( network.directions, 0 )
    end
end

""" add a new link

	return: link index
"""
function addPhysicalLane( net::Network, start_node, end_node, capacity, link_length, free_flow_time, B, power, speed_limit, toll, link_type, link_permission, lane_capacity)
  # add physical lane mark
  addPhysicalLinkMark(net, start_node, end_node)

  if contains(link_permission, "1")
    push!( net.lane_indicator, 1 )
  else
    push!( net.lane_indicator, 0 )
  end

  # save lane value
  push!( net.capacity, capacity )
  push!( net.link_length, link_length )
  push!( net.free_flow_time, free_flow_time )
  push!( net.physical_free_flow_time, free_flow_time )
  push!( net.B, B )
  push!( net.power, power )
  push!( net.speed_limit, speed_limit )
  push!( net.toll, toll )
  push!( net.link_type, link_type )
  push!( net.link_permission, link_permission )
  push!( net.lane_capacity, lane_capacity )
  push!( net.X, 0 )
end

function add_supper_line( net, from_node, to_node )
   add_one_side_super_line( net, from_node, to_node  )
   add_one_side_super_line( net, to_node, from_node  )
end

function add_one_side_super_line( net, from_node, to_node  )
  addPhysicalLinkMark(net, from_node, to_node)
  push!( net.lane_indicator, -1 )  # for super road mark

  link_index = ( from_node, to_node )

  # save lane value
  push!( net.capacity, Inf )
  push!( net.link_length, 5800 )
  push!( net.free_flow_time, 0 )
  push!( net.physical_free_flow_time, 0 )
  push!( net.B, 1 )
  push!( net.power, 1 )
  push!( net.speed_limit, Inf )
  push!( net.toll, 0 )
  push!( net.link_type, 1 )
  push!( net.link_permission, "0" )
  push!( net.lane_capacity, Inf )
  push!( net.X, 0 )
end

function print( net )
  println("link, link_capacity, lane_capacity, free_flow_time, link_permission")
  physical_links::Array{Tuple{Int64, Int64}}    # ( small_node, large_node ), every item for one physical lane
  directions::Array{Int64}
  for i in 1:length( net.physical_links )
    k = net.directions[i]==1 ? net.physical_links[i] : (net.physical_links[i][2], net.physical_links[i][1])
    println(k, ": ", net.capacity[i], ", ", net.lane_capacity[i], ", ", net.free_flow_time[i], ", ", net.link_permission[i])
  end
end

function start_nodes( net )
  nodes = Array{Int64}( length( net.physical_links ) )
  for i in 1:length( net.physical_links )
    nodes[i] =  ( net.directions[i]==1 ? net.physical_links[i][1] : net.physical_links[i][2] )
  end
  return nodes
end

function end_nodes( net )
  nodes = Array{Int64}( length( net.physical_links ) )
  for i in 1:length( net.physical_links )
    nodes[i] =  ( net.directions[i]==1 ? net.physical_links[i][2] : net.physical_links[i][1] )
  end
  return nodes
end

function get_link_number( net )
  return length( net.physical_links )
end

""" return the link index
"""
function get_link_index( net, from_node, to_node )
  if from_node < to_node
    node1 = from_node
    node2 = to_node
  else
    node1 = to_node
    node2 = from_node
  end

  for i in 1:length(net.physical_links)
    if net.physical_links[i] == (node1, node2)
      return i
    end
  end
  return -1
end

function set_link_ev( net, from_node, to_node )
  index = get_link_index( net, from_node, to_node )

  net.lane_indicator[index] = 1
  net.link_permission[index] = "1"

  net.free_flow_time[index] = Inf
  net.capacity[index] = 0.1^10
end

function set_link_pv( net, from_node, to_node )
  index = get_link_index( net, from_node, to_node )

  net.lane_indicator[index] = 0
  net.link_permission[index] = "0"

  net.free_flow_time[index] = net.physical_free_flow_time[index]
  net.capacity[index] = net.lane_capacity[index]
end

function reverse_link( net, from_node, to_node )
  index = get_link_index( net, from_node, to_node )

  if net.directions[index] == 1
    net.directions[index] = 0
  else
    net.directions[index] = 1
  end
end

function update_pv_network( net )
  network = deepcopy( net )
  for k in 1:length(net.capacity)
     if contains(net.link_permission[k], "1")
       net.capacity[k] = 0
       net.free_flow_time[k] = Inf
     end
  end
  return network
end

function get_vertices( net )
  nodes1 = start_nodes( net )
  nodes2 = end_nodes( net )
  vs = Set()
  for d in nodes1
    push!( vs, d )
  end
  for d in nodes2
    push!( vs, d )
  end

  return vs
end
