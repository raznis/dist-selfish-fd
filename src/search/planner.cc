#include "globals.h"
#include "operator.h"
#include "option_parser.h"
#include "ext/tree_util.hh"
#include "timer.h"
#include "utilities.h"
#include "search_engine.h"

#include <iostream>
using namespace std;

int main(int argc, const char **argv) {
	register_event_handlers();

	if (argc < 2) {
		cout << OptionParser::usage(argv[0]) << endl;
		exit(1);
	}

	if (string(argv[1]).compare("--help") != 0)
		read_everything(cin);

	SearchEngine *engine = 0;

	//the input will be parsed twice:
	//once in dry-run mode, to check for simple input errors,
	//then in normal mode
	try {
		OptionParser::parse_cmd_line(argc, argv, true);
		engine = OptionParser::parse_cmd_line(argc, argv, false);
	} catch (ParseError &pe) {
		cout << pe << endl;
		exit(1);
	}

	if (g_agents_search) {
		//	partition_by_agent_names("agents");
		initialize_communication("comm");
	} else if (g_symmetry_pruning)
		perform_optimal_partition();

	Timer search_timer;
	engine->search();
	search_timer.stop();
	g_timer.stop();

	cout << "Not saving plan yet, since traceback is not yet implemented." << endl;
	//engine->save_plan_if_necessary();
	engine->statistics();
	engine->heuristic_statistics();
	cout << "Search time: " << search_timer << endl;
	cout << "Total time: " << g_timer << endl;
	cout << "Messages sent: " << g_num_of_messages_sent << endl;
	cout << "Messages received: " << g_num_of_messages_received << endl;

	return engine->found_solution() ? 0 : 1;
}
