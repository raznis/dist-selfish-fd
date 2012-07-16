#include <cassert>
#include <iostream>
#include <limits>
using namespace std;

#include "globals.h"
#include "search_engine.h"
#include "timer.h"
#include "option_parser.h"

SearchEngine::SearchEngine(const Options &opts) :
		search_space(OperatorCost(opts.get_enum("cost_type"))), cost_type(
				OperatorCost(opts.get_enum("cost_type"))) {
	solved = false;
	if (opts.get<int>("bound") < 0) {
		cerr << "error: negative cost bound " << opts.get<int>("bound") << endl;
		exit(2);
	}
	bound = opts.get<int>("bound");
}

SearchEngine::~SearchEngine() {
}

void SearchEngine::statistics() const {
}

bool SearchEngine::found_solution() const {
	return solved;
}

const SearchEngine::Plan &SearchEngine::get_plan() const {
	assert(solved);
	return plan;
}

void SearchEngine::set_plan(const Plan &p) {
	solved = true;
	plan = p;
}

void SearchEngine::search() {
	initialize();
	Timer timer;
	while (step() == IN_PROGRESS)
		;
	cout << "Actual search time: " << timer << " [t=" << g_timer << "]" << endl;
}

bool SearchEngine::check_goal_and_set_plan(const State &state) {
	if (test_goal(state)) {
		Plan plan;
		if (g_multiple_goal) {

			search_space.trace_path(state, plan);

			bool found_all_marginal_solutions = true;
			for (int i = 0; i <= g_num_of_agents; i++) { //last value is the optimal plan (does not have to exclude any agent)
				if (g_marginal_solution_for_agent[i] == -1
						&& !did_agent_participate(plan, i)) {
					g_marginal_solution_for_agent[i] = calculate_plan_cost(
							plan);
					cout << "found plan excluding agent " << i << " with cost "
							<< g_marginal_solution_for_agent[i] << endl;
					if(i == g_num_of_agents)
						set_plan(plan);
				}
				if (g_marginal_solution_for_agent[i] == -1)
					found_all_marginal_solutions = false;
			}
			if (found_all_marginal_solutions)
				return true;
			return false;
		}
		cout << "Solution found!" << endl;
		search_space.trace_path(state, plan);
		set_plan(plan);
		return true;
	}
	return false;
}

void SearchEngine::save_plan_if_necessary() const {
	if (found_solution())
		save_plan(get_plan(), 0);
}

int SearchEngine::get_adjusted_cost(const Operator &op) const {
	return get_adjusted_action_cost(op, cost_type);
}

void SearchEngine::add_options_to_parser(OptionParser &parser) {
	vector<string> cost_types;
	cost_types.push_back("NORMAL");
	cost_types.push_back("ONE");
	cost_types.push_back("PLUSONE");
	parser.add_enum_option("cost_type", cost_types, "NORMAL",
			"operator cost adjustment type");
	parser.add_option<int>("bound", numeric_limits<int>::max(),
			"bound on plan cost");
}
