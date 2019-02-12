set xrange [0:610]
set terminal pngcairo enhanced color lw 2 size 800, 600 font 'Arial-Bold'
set output "harsh-sst.png"
set xlabel "Apna time aayega (Seconds)"
set ylabel "SSThresh (bytes)"
set key outside;
set key right top;
plot "harsh-sst.plotme" title "SSThresh" with lines lw 1.5
