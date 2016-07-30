#############################################################################
#  Copyright 2016, Liye Zhang and contributors
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at http://mozilla.org/MPL/2.0/.
#############################################################################
# Class for traffic demand management
#
# Network extended from the following
# Sioux Falls network data
# http://www.bgu.ac.il/~bargera/tntp/
#
# user type- 1 for "pv", 2 for 'ev', permision property of the link is as '1-4-6'
#############################################################################

# Link travel time = free flow time * ( 1 + B * (flow/capacity)^Power ).
# Link generalized cost = Link travel time + toll_factor * toll + distance_factor * distance

# traffic demand structure
type Demand
    travel_demand::Array{Float64, 2}         # OD matrix
    od_pairs::Array{Tuple{Int64, Int64}}     # [(2,3), (4,5), ...], grow automatically
                                             # used for map index - (o, d)

    total_od_flow::Float64                   # sum of all od pairs ?, not used in this algorithm
    user_type::ASCIIString                   # user type name, such as "ev"
end

"""
read OD demand from trip file

# Arguments
* trip_file: full path of trip file

# Return:
"""
function Demand( trip_file )
    total_od_flow = 0
    number_of_zones_trip = 0
    user_type = ""

    f = open( trip_file, "r" )

    while (line=readline(f)) != ""
        if contains(line, "<NUMBER OF ZONES>")
            number_of_zones_trip = parse(Int, line[ search(line, '>')+1 : end-1 ] )
        elseif contains(line, "<TOTAL OD FLOW>")
            total_od_flow = parse(Float64, line[ search(line, '>')+1 : end-1 ] )
        elseif contains(line, "<USER TYPE>")
            user_type = strip(line[ search(line, '>')+1 : end-1 ] )
            user_type = strip( user_type )
        elseif contains(line, "<END OF METADATA>")
            break
        end
    end

    @assert number_of_zones_trip > 0
    @assert total_od_flow > 0
    @assert user_type != ""

    # create traffic demand structure
    #
    travel_demand = zeros(number_of_zones_trip, number_of_zones_trip)
    # od_pairs = Array{Tuple{Int64, Int64}}(0)  # [(2,3), (4,5), ...], grow automatically
    od_pairs = Array{Tuple{Int64, Int64}}(0)

    # fill the data
    while (line=readline(f)) != ""
        if contains(line, "Origin")
            origin = parse(Int, split(line)[2] )
        elseif contains(line, ";")
            pairs = split(line, ";")     # several pair in one row, eg. 2 :    1365.90;    3 :     407.40;    4 :     861.40;
            for i=1:size(pairs)[1]       # process one row
                if contains(pairs[i], ":")
                    pair = split(pairs[i], ":")
                    destination = parse(Int, strip(pair[1]) )   # destination node number
                    od_flow = parse(Float64, strip(pair[2]) )   # od pair value

                    travel_demand[origin, destination] = od_flow   # fill OD matrix
                    push!(od_pairs, (origin, destination))         # fill OD pairs
                    # println("origin=$origin, destination=$destination, flow=$od_flow")
                end
            end
        end
    end

    demand = Demand( travel_demand, od_pairs,total_od_flow,  user_type )
    return demand
end
