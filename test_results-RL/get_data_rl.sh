#!/bin/bash

echo "Results level "$1

filename="tmp$1.txt"

rm -f $filename


for x in {1..14}
do

    tst=$(tail -1 flatland_test$1-RL/result-$x | head -n1 | awk {'print $1'})

    if [ "$tst" != "Time:" ]; then
        echo "0 0 0" >> $filename
        continue
    fi

    rew=$(tail -3 flatland_test$1-RL/result-$x | head -n1 | awk {'print $2'})
    steps=$(tail -2 flatland_test$1-RL/result-$x | head -n1 | awk {'print $2'})
    time=$(tail -1 flatland_test$1-RL/result-$x | head -n1 | awk {'print $2'})

    echo $rew $steps $time >> $filename

done

counter=1
for x in 2 4 6 8 10 11 12 14 15 16 17 18 19 20
do
    

    agt_count=$(sed "${counter}q;d" $filename | awk {'print $1'})
    
    time=$(sed "${counter}q;d" $filename | awk {'print $3'})

    kehr=$(bc -l <<< $time'/'$x)
    res=$(bc -l <<< $kehr'*'$agt_count)
    
    sed -i -e $counter's/^/'$res' /' $filename

    sed -i -e $counter's/^/'$x' /' $filename

    counter=$((counter+1))
done

cat $filename


gnuplot -e "set terminal wxt size 1200,800;
            set title 'Evaluation Level $1';
            set grid lt 0 lw 0.5 lc rgb '#ff0000';
            $(counter=1; for x in 2 4 6 8 10 11 12 14 15 16 17 18 19 20; do agt_count=$(sed "${counter}q;d" $filename | awk {'print $1'}); echo "set label '$(sed "${counter}q;d" $filename | awk {'print $3'})' at $x+0.1,$(bc -l <<< $(sed "${counter}q;d" $filename | awk {'print $2'})"");"; counter=$((counter+1)); done)
            set arrow from 2,3600 to 20,3600 nohead linecolor rgb 'red';
            set style fill solid;
            set boxwidth 0.2;
            plot '$filename' using 1:2:xtic(1) w boxes t 'Reward',
                 '$filename' using 1:5 w lp t 'Time';
            pause mouse"