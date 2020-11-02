set term postscript eps enhanced color
set output "02.eps"
set size ratio -1
set xrange [-417:778]
set yrange [-78:1117]
set xlabel "x [m]"
set ylabel "z [m]"
plot "02.txt" using 1:2 lc rgb "#FF0000" title 'Ground Truth' w lines,"02.txt" using 3:4 lc rgb "#0000FF" title 'Visual Odometry' w lines,"< head -1 02.txt" using 1:2 lc rgb "#000000" pt 4 ps 1 lw 2 title 'Sequence Start' w points
