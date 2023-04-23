function mapTransCallback(msg)
fix_map = rosReadOccupancyGrid(msg);
show(fix_map);
end