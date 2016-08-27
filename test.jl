# this is just to test the functions
#
function test()
  @show splitdir( abspath( @__FILE__ ) )[1] * "\\data\\"
end

test()
