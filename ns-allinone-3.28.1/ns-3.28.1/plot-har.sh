set xrange [0:610]
set terminal pngcairo enhanced color lw 2 size 800, 600 font 'Arial-Bold'
set output "harsh.png"
set xlabel "Time (Seconds)" font ",18"
set ylabel "CWND (bytes)" font ",18"
set key outside;
set key right top;
plot "harsh-cwnd.plotme" title "CWND" with lines lw 1.5
