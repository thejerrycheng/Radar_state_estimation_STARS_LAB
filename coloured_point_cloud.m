function coloured_point_cloud(range_x, range_y, error)

num_points = size(error);

for i = 1:num_points

    if error < 0.05


        
    elseif error < 0.1



    elseif error < 0.5



    elseif error < 1



    elseif error < 2



    elseif error < 5



    else 


    end 

end 



figure






end

