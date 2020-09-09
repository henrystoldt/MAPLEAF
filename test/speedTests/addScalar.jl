function addFive(x::Array{Float64,1})
    for i = 1:length(x)
        x[i] += 5.0
    end
    return x
end