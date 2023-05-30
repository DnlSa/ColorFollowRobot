function pckt = robot_pkt(dataOut)
    last_cmd = typecast(uint32(dataOut.last_cmd), 'uint32');% OKcomando 
    prefer_distance = typecast(uint32(dataOut.prefer_distance), 'uint32'); % distanza custom 0 per mantenerla di default impostata sul robot 
    pckt = [last_cmd, prefer_distance];
end
