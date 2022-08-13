%Code used to correct angle subratctions when they occur
function diff = angle_sub(diff_ang)
    diff = mod(diff_ang + pi, 2 * pi) - pi;  
end