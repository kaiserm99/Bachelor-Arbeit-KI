import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--level", help="Insert level number", type=int)

args = parser.parse_args()


print(("""#!/bin/bash
#SBATCH -a 1-14
#SBATCH -p gki_cpu-cascadelake
#SBATCH --mem=4G
#SBATCH --time=0:20:00

if [ 1 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 2 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 2 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 4 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 3 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 6 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi


if [ 4 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 8 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 5 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 10 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 6 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 11 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi


if [ 7 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 12 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 8 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 14 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 9 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 15 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 10 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 16 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 11 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 17 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 12 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 18 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 13 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 19 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi

if [ 14 -eq $SLURM_ARRAY_TASK_ID ]; then
   mkdir -p ./flatland_test{0}-RL
   python3 neurips2020-flatland-starter-kit/run.py -l {0} -a 20 &>> ./flatland_test{0}-RL/result-$SLURM_ARRAY_TASK_ID
   exit $?
fi""").format(args.level))
