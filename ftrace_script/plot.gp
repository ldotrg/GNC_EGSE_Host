# Usege :
# gnuplot -e 'in="plot_data_input.txt";out="output1.png";gtitle="EGSE RX CAN and TX RS422 driver IO time";count=10000;max=50.238;avg=1.881219;sigma=5.283921' plot.gp

# Scale factor, is millisecond
sc = 1

set terminal png truecolor size 1200,700
set output out

set title gtitle noenhanced
set xlabel "Timeframe # (20HZ)"
set ylabel "Driver IO-Time (ms)"

# Position & format of label has been carefully adjusted,
# do not change unless you are sure
set rmargin 11
set label 1 sprintf("count=\n%d\nmax=\n%\.3f\navg=\n%\.3f\nsigma=\n%\.6f",count,max * sc, avg * sc, sigma *sc)
set label 1 at screen 0.92, screen 0.6 left
set key off
set grid
plot in using 1:2 with line
