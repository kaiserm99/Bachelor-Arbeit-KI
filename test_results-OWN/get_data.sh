#!/bin/bash

echo "Results level "$1
re='^[0-9]+$'

filename="tmp$1.txt"

rm -f $filename

for x in {1..14}
do

    var=$(tail -1 flatland_test$1-OWN/result-$x)
    ll=$(echo $var | awk {'print $3'})
    ll=${ll%?}

    
    
    if ! [[ $ll =~ $re ]] ; then
        echo "0 0 0 0 0" >> $filename
        continue
    fi

    init=$(echo $var | awk {'print $6'})
    init=${init%?}

    high=$(echo $var | awk {'print $9'})
    high=${high%?}

    time=$(tail -n3 flatland_test$1-OWN/result-$x | head -n1 | awk {'print $5'})
    time=$(sed 's/.\{3\}$//' <<<"$time")

    echo $ll $init $high $time $time >> $filename
done



counter=1
for x in 2 4 6 8 10 11 12 13 14 15 16 17 18 19 20
do
    sed -i -e $counter's/^/'$x' /' $filename
    
    counter=$((counter+1))
done

cat $filename


gnuplot -e "set terminal wxt size 1200,800;
            set title 'Evaluation Level $1';
            set grid lt 0 lw 0.5 lc rgb '#ff0000';
            set logscale y;
            set arrow from 2,3600 to 20,3600 nohead linecolor rgb 'red';
            set style fill solid;
            set boxwidth 0.2;
            plot '$filename' using 1:6:xtic(1) w boxes t 'Reward',
                 '$filename' using 1:2 w lp t 'LowNodes', 
                 '$filename' using 1:3 w lp t 'InitNodes',
                 '$filename' using 1:4 w lp t 'HighNodes',
                 '$filename' using 1:5 w lp t 'Time';
            pause mouse"