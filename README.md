dist-selfish-fd
===============

MA-FD code, with the option of using selfish agents.

Instructions on running an example problem.
go to the src/ directory
./build_all
./translate/translate.py ../benchmarks/rovers/domain.pddl ../benchmarks/rovers/p12.pddl./preprocess/preprocess < output.sas
This specific problem has 4 agents called "rover[0-3]", so open agents file and insert the following:
4
rover0
rover1
rover2
rover3

The names are important, they are how the actions are distributed among the agents.
Then go to comm file and insert the following:
127.0.0.1:3010
127.0.0.1:3011
127.0.0.1:3012
127.0.0.1:3013

This gives an ip address to each of the 4 agents

Now run the 4_agents script, which contains the following text:
./search/downward --search "astar(lmcut())" --agents 0 < output > 0 &
./search/downward --search "astar(lmcut())" --agents 1 < output > 1 &
./search/downward --search "astar(lmcut())" --agents 2 < output > 2 &
./search/downward --search "astar(lmcut())" --agents 3 < output > 3
python parse_results.py 4

See other scripts on ways of running non-optimal algorithm and other heuristics.
