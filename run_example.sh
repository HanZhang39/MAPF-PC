
./bin/cbs -m sample_input/empty-16-16.map -a sample_input/agents_goal_sequences.txt -s 2 -k 10

./bin/cbs -m sample_input/empty-16-16.map -a sample_input/agents_goal_sequences.txt -s 2 -k 10

./bin/task_assignment -m sample_input/empty-16-16.map -a sample_input/agents_goals.txt -k 20 -t 300 -o output.txt --solver PBS
