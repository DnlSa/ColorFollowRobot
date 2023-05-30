function data = robot_unpack(pckt)

 cnt = 1;
 data.last_cmd = typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

 data.prefer_distance =typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

 data.latency = typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

 data.pos_x = typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

 data.pos_y = typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

 data.distance = typecast(uint8(pckt(cnt:cnt+3)), 'uint32');
 cnt = cnt + 4;

end
