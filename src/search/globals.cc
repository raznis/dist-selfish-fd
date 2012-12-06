#include "globals.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <sstream>
using namespace std;

#include <ext/hash_map>
using namespace __gnu_cxx;

#include "axioms.h"
#include "causal_graph.h"
#include "domain_transition_graph.h"
#include "heuristic.h"
#include "operator.h"
#include "rng.h"
#include "state.h"
#include "successor_generator.h"
#include "timer.h"

static const int PRE_FILE_VERSION = 3;

// TODO: This needs a proper type and should be moved to a separate
//       mutexes.cc file or similar, accessed via something called
//       g_mutexes. (Right now, the interface is via global function
//       are_mutex, which is at least better than exposing the data
//       structure globally.)

static vector<vector<set<pair<int, int> > > > g_inconsistent_facts;

bool test_goal(const State &state) {
	for (int i = 0; i < g_goal.size(); i++) {
		if (state[g_goal[i].first] != g_goal[i].second) {
			return false;
		}
	}
	return true;
}

int calculate_plan_cost(const vector<const Operator *> &plan) {
	// TODO: Refactor: this is only used by save_plan (see below)
	//       and the SearchEngine classes and hence should maybe
	//       be moved into the SearchEngine (along with save_plan).
	int plan_cost = 0;
	if (g_agents_search && !g_marginal_search && !g_multiple_goal) {
		cout << "Plan costs for all agents:" << endl;
		for (int agent = 0; agent < g_num_of_agents; agent++) {
			plan_cost = 0;
			for (int i = 0; i < plan.size(); i++) {
				if (plan[i]->agent != agent)
					plan_cost += plan[i]->get_cost();
			}
			cout << "Without Agent " << agent << ": " << plan_cost << endl;
		}
	}
	plan_cost = 0;
	for (int i = 0; i < plan.size(); i++) {
		plan_cost += plan[i]->get_cost();
	}
	return plan_cost;
}

void save_plan(const vector<const Operator *> &plan, int iter) {
	// TODO: Refactor: this is only used by the SearchEngine classes
	//       and hence should maybe be moved into the SearchEngine.
	ofstream outfile;
	if (iter == 0) {
		outfile.open(g_plan_filename.c_str(), ios::out);
	} else {
		ostringstream out;
		out << g_plan_filename << "." << iter;
		outfile.open(out.str().c_str(), ios::out);
	}
	for (int i = 0; i < plan.size(); i++) {
		cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")"
				<< endl;
		outfile << "(" << plan[i]->get_name() << ")" << endl;
	}
	outfile.close();
	int plan_cost = calculate_plan_cost(plan);
	ofstream statusfile;
	statusfile.open("plan_numbers_and_cost", ios::out | ios::app);
	statusfile << iter << " " << plan_cost << endl;
	statusfile.close();
	cout << "Plan length: " << plan.size() << " step(s)." << endl;
	cout << "Plan cost: " << plan_cost << endl;
}

bool did_agent_participate(const vector<const Operator *> &plan, int agent) {
	for (int i = 0; i < plan.size(); i++) {
		if (plan[i]->agent == agent)
			return true;
	}
	return false;
}

bool peek_magic(istream &in, string magic) {
	string word;
	in >> word;
	bool result = (word == magic);
	for (int i = word.size() - 1; i >= 0; i--)
		in.putback(word[i]);
	return result;
}

void check_magic(istream &in, string magic) {
	string word;
	in >> word;
	if (word != magic) {
		cout << "Failed to match magic word '" << magic << "'." << endl;
		cout << "Got '" << word << "'." << endl;
		if (magic == "begin_version") {
			cerr << "Possible cause: you are running the planner "
					<< "on a preprocessor file from " << endl
					<< "an older version." << endl;
		}
		exit(1);
	}
}

void read_and_verify_version(istream &in) {
	int version;
	check_magic(in, "begin_version");
	in >> version;
	check_magic(in, "end_version");
	if (version != PRE_FILE_VERSION) {
		cerr << "Expected preprocessor file version " << PRE_FILE_VERSION
				<< ", got " << version << "." << endl;
		cerr << "Exiting." << endl;
		exit(1);
	}
}

void read_metric(istream &in) {
	check_magic(in, "begin_metric");
	in >> g_use_metric;
	check_magic(in, "end_metric");
}

void read_variables(istream &in) {
	int count;
	in >> count;
	for (int i = 0; i < count; i++) {
		check_magic(in, "begin_variable");
		string name;
		in >> name;
		g_variable_name.push_back(name);
		int layer;
		in >> layer;
		g_axiom_layers.push_back(layer);
		int range;
		in >> range;
		g_variable_domain.push_back(range);
		if (range > numeric_limits<state_var_t>::max()) {
			cerr << "This should not have happened!" << endl;
			cerr << "Are you using the downward script, or are you using "
					<< "downward-1 directly?" << endl;
			exit(1);
		}

		in >> ws;
		vector<string> fact_names(range);
		for (size_t i = 0; i < fact_names.size(); i++)
			getline(in, fact_names[i]);
		g_fact_names.push_back(fact_names);
		check_magic(in, "end_variable");
		g_variable_agent.push_back(-2);
	}
}

void read_mutexes(istream &in) {
	g_inconsistent_facts.resize(g_variable_domain.size());
	for (size_t i = 0; i < g_variable_domain.size(); ++i)
		g_inconsistent_facts[i].resize(g_variable_domain[i]);

	int num_mutex_groups;
	in >> num_mutex_groups;

	/* NOTE: Mutex groups can overlap, in which case the same mutex
	 should not be represented multiple times. The current
	 representation takes care of that automatically by using sets.
	 If we ever change this representation, this is something to be
	 aware of. */

	for (size_t i = 0; i < num_mutex_groups; ++i) {
		check_magic(in, "begin_mutex_group");
		int num_facts;
		in >> num_facts;
		vector<pair<int, int> > invariant_group;
		invariant_group.reserve(num_facts);
		for (size_t j = 0; j < num_facts; ++j) {
			int var, val;
			in >> var >> val;
			invariant_group.push_back(make_pair(var, val));
		}
		check_magic(in, "end_mutex_group");
		for (size_t j = 0; j < invariant_group.size(); ++j) {
			const pair<int, int> &fact1 = invariant_group[j];
			int var1 = fact1.first, val1 = fact1.second;
			for (size_t k = 0; k < invariant_group.size(); ++k) {
				const pair<int, int> &fact2 = invariant_group[k];
				int var2 = fact2.first;
				if (var1 != var2) {
					/* The "different variable" test makes sure we
					 don't mark a fact as mutex with itself
					 (important for correctness) and don't include
					 redundant mutexes (important to conserve
					 memory). Note that the preprocessor removes
					 mutex groups that contain *only* redundant
					 mutexes, but it can of course generate mutex
					 groups which lead to *some* redundant mutexes,
					 where some but not all facts talk about the
					 same variable. */
					g_inconsistent_facts[var1][val1].insert(fact2);
				}
			}
		}
	}
}

void read_goal(istream &in) {
	check_magic(in, "begin_goal");
	int count;
	in >> count;
	for (int i = 0; i < count; i++) {
		int var, val;
		in >> var >> val;
		g_goal.push_back(make_pair(var, val));
	}
	check_magic(in, "end_goal");
}

void dump_goal() {
	cout << "Goal Conditions:" << endl;
	for (int i = 0; i < g_goal.size(); i++)
		cout << "  " << g_variable_name[g_goal[i].first] << ": "
				<< g_goal[i].second << endl;
}

void read_operators(istream &in) {
	int count;
	in >> count;
	for (int i = 0; i < count; i++)
		g_operators.push_back(Operator(in, false));
}

void read_axioms(istream &in) {
	int count;
	in >> count;
	for (int i = 0; i < count; i++)
		g_axioms.push_back(Operator(in, true));

	g_axiom_evaluator = new AxiomEvaluator;
	g_axiom_evaluator->evaluate(*g_initial_state);
}

void read_everything(istream &in) {
	read_and_verify_version(in);
	read_metric(in);
	read_variables(in);
	read_mutexes(in);
	g_initial_state = new State(in);
	read_goal(in);
	read_operators(in);
	read_axioms(in);
	check_magic(in, "begin_SG");
	g_successor_generator = read_successor_generator(in);
	check_magic(in, "end_SG");
	DomainTransitionGraph::read_all(in);
	g_causal_graph = new CausalGraph(in);
}

void dump_everything() {
	cout << "Use metric? " << g_use_metric << endl;
	cout << "Min Action Cost: " << g_min_action_cost << endl;
	cout << "Max Action Cost: " << g_max_action_cost << endl;
	// TODO: Dump the actual fact names.
	cout << "Variables (" << g_variable_name.size() << "):" << endl;
	for (int i = 0; i < g_variable_name.size(); i++)
		cout << "  " << g_variable_name[i] << " (range " << g_variable_domain[i]
				<< ")" << endl;
	cout << "Initial State:" << endl;
	g_initial_state->dump();
	dump_goal();
	/*
	 cout << "Successor Generator:" << endl;
	 g_successor_generator->dump();
	 for(int i = 0; i < g_variable_domain.size(); i++)
	 g_transition_graphs[i]->dump();
	 */
}

void verify_no_axioms_no_cond_effects() {
	if (!g_axioms.empty()) {
		cerr << "Heuristic does not support axioms!" << endl << "Terminating."
				<< endl;
		exit(1);
	}

	for (int i = 0; i < g_operators.size(); i++) {
		const vector<PrePost> &pre_post = g_operators[i].get_pre_post();
		for (int j = 0; j < pre_post.size(); j++) {
			const vector<Prevail> &cond = pre_post[j].cond;
			if (cond.empty())
				continue;
			// Accept conditions that are redundant, but nothing else.
			// In a better world, these would never be included in the
			// input in the first place.
			int var = pre_post[j].var;
			int pre = pre_post[j].pre;
			int post = pre_post[j].post;
			if (pre == -1 && cond.size() == 1 && cond[0].var == var
					&& cond[0].prev != post && g_variable_domain[var] == 2)
				continue;

			cerr << "Heuristic does not support conditional effects "
					<< "(operator " << g_operators[i].get_name() << ")" << endl
					<< "Terminating." << endl;
			exit(1);
		}
	}
}

bool are_mutex(const pair<int, int> &a, const pair<int, int> &b) {
	if (a.first == b.first) // same variable: mutex iff different value
		return a.second != b.second;
	return bool(g_inconsistent_facts[a.first][a.second].count(b));
}

bool g_use_metric;
int g_min_action_cost = numeric_limits<int>::max();
int g_max_action_cost = 0;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<vector<string> > g_fact_names;
vector<int> g_axiom_layers;
vector<int> g_default_axiom_values;
State *g_initial_state;
vector<pair<int, int> > g_goal;
vector<Operator> g_operators;
vector<Operator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
CausalGraph *g_causal_graph;

Timer g_timer;
string g_plan_filename = "sas_plan";
RandomNumberGenerator g_rng(2011); // Use an arbitrary default seed.

bool g_symmetry_pruning = false;
char **g_operator_switchability = NULL;
int g_num_of_public_actions = 0;

bool g_agents_search = false;
vector<int> g_variable_agent;
bool g_marginal_search = false;
int g_marginal_agent = -1;
int g_num_of_agents = -1;
bool g_multiple_goal = false;

bool g_parallel_search = false;

int g_agent_id = -1;
MAComm* g_ma_comm;
MAConfiguration g_comm_config;
vector<State*> g_current_solution;
vector<int> g_g_of_current_solution;
vector<int> g_num_of_agents_confirming_current_solution;

vector<bool> g_received_termination;
int g_num_of_messages_sent = 0;
int g_num_of_messages_received = 0;
bool g_message_delay = false;

void update_private_public_actions() {
	//	for (int var = 0; var < g_variable_domain.size(); var++) {
	//		cout << g_variable_name[var] << ", agent: " << g_variable_agent[var] << endl;
	//	}
	//Updating operators as private/public
	//TODO (Raz) As is implemented now, operators that affect a goal variable are public.
	cout << "Updating public and private actions... " << endl;
	int public_actions = 0;
	int private_actions = 0;
	int zero_cost_Actions = 0;
	for (int op_idx = 0; op_idx < g_operators.size(); op_idx++) {
		if (g_operators[op_idx].uses_public_variable()
				|| g_operators[op_idx].affect_goal_variable()) {
			g_operators[op_idx].is_public = true;
			public_actions++;
		} else {
			g_operators[op_idx].is_public = false;
			//assigning 0-cost to private actions not belonging to the agent
			if (!g_parallel_search && g_operators[op_idx].agent != g_agent_id){
				g_operators[op_idx].set_cost(0);
				cout << g_operators[op_idx].get_name() << endl;
				zero_cost_Actions++;
			}
			private_actions++;
		}
		//		cout << op_idx << ", "<< g_operators[op_idx].get_name() << ", "
		//				<< g_operators[op_idx].is_public << endl;
	}
	cout << "public: " << public_actions << ", private: " << private_actions << ", zero cost actions: " << zero_cost_Actions
			<< endl;
}

/*
 * Agent search helper functions
 */

void partition_by_agent_names(const char* configFileName) {

	string line;
	cout << "Opening agent names configuration file: " << configFileName
			<< endl;
	ifstream myfile(configFileName);
	int nAgents = 0;
	//int use_local_heuristic;
	if (myfile.is_open()) {
		//getline(myfile, line);
		//use_local_heuristic = atoi(line.c_str());
		//if(use_local_heuristic){
		//	cout<<"Using localized heuristics." <<endl;
		//	g_use_local_heuristic = true;
		//}
		//else{
		//	cout<<"Using global heuristic." <<endl;
		//	g_use_local_heuristic = false;
		//}
		getline(myfile, line);
		nAgents = atoi(line.c_str());
		g_num_of_agents = nAgents;
		cout << "number of agents is: " << g_num_of_agents << endl;
		string* agent_names = new string[nAgents];
		int* agent_ids = new int[nAgents];
		for (int i = 0; i < nAgents; i++) {
			getline(myfile, line);
			agent_names[i] = line;
			cout << "agent " << i << ": " << agent_names[i] << endl;
			agent_ids[i] = i;
			g_current_solution.push_back(0);//marginal solutions for the agents
			g_g_of_current_solution.push_back(-1); //-1 means that we haven't found a marginal solution for the agent.
			g_num_of_agents_confirming_current_solution.push_back(0);
			g_received_termination.push_back(false);

		}
		g_current_solution.push_back(0);	//another value for the optimal plan
		g_g_of_current_solution.push_back(-1); //another value for the optimal plan
		g_num_of_agents_confirming_current_solution.push_back(0);//another value for the optimal plan
		g_received_termination.push_back(false);//another value for the optimal plan

		myfile.close();
		cout << "Closed agent names configuration file" << endl;

		int num_of_associated_ops = 0;
		for (int op_idx = 0; op_idx < g_operators.size(); op_idx++) {
			string op_name = g_operators[op_idx].get_name();
			for (int agent_idx = 0; agent_idx < nAgents; agent_idx++) {
				if (op_name.find(agent_names[agent_idx]) != string::npos) {
					g_operators[op_idx].agent = agent_ids[agent_idx];
					num_of_associated_ops++;
					break;
				}
			}
		}
		if (g_operators.size() != num_of_associated_ops) {
			cout << "NOT ALL ACTIONS ARE ASSOCIATED!!!" << endl;
			exit(1);
		}

		//Assigning variables to agents
		for (int var = 0; var < g_variable_domain.size(); var++) {
			int used_by_agent = -1;
			for (int op_idx = 0; op_idx < g_operators.size(); op_idx++) {
				if (g_operators[op_idx].uses_variable(var)) {
					if (used_by_agent == -1) {
						used_by_agent = g_operators[op_idx].agent;
						g_variable_agent[var] = used_by_agent;
					} else if (used_by_agent != g_operators[op_idx].agent) {
						//this variable is public
						g_variable_agent[var] = -1;
						break;

					}
				}
			}
			if (used_by_agent == -1)
				cout << "Variable not associated to any agent!!!" << endl;
		}

		//Updating operators as private/public
		//TODO (Raz) As is implemented now, operators that affect a goal variable are public.
		update_private_public_actions();
	} else
		cout << "Unable to open file" << endl;

}

bool should_be_connected_in_action_graph(int op1_idx, int op2_idx) {
	int min = op1_idx > op2_idx ? op2_idx : op1_idx;
	int max = op1_idx < op2_idx ? op2_idx : op1_idx;

	if (g_operator_switchability && g_operator_switchability[min][max] != 0) {
		return g_operator_switchability[min][max] == 1;
	}

	const Operator op1 = g_operators[op1_idx];
	const Operator op2 = g_operators[op2_idx];
	const vector<PrePost> op1_prepost = op1.get_pre_post();
	const vector<Prevail> op1_prevail = op1.get_prevail();
	const vector<PrePost> op2_prepost = op2.get_pre_post();
	const vector<Prevail> op2_prevail = op2.get_prevail();

	for (int i = 0; i < op1_prepost.size(); i++) {
		int var = op1_prepost[i].var;
		int pre = op1_prepost[i].pre;
		int post = op1_prepost[i].post;
		for (int j = 0; j < op2_prepost.size(); j++) {
			if (op2_prepost[j].var == var) {
				int other_pre = op2_prepost[j].pre;
				int other_post = op2_prepost[j].post;
				if (pre == other_pre || post == other_pre || pre == other_post
						|| post == other_post) //TODO - check last condition (have the same effect)
								{
					if (g_operator_switchability)
						g_operator_switchability[min][max] = 1;
					return true;
				}
			}
		}
		for (int j = 0; j < op2_prevail.size(); j++) {
			if (op2_prevail[j].var == var) {
				int other_prevail = op2_prevail[j].prev;
				if (pre == other_prevail || post == other_prevail) {
					if (g_operator_switchability)
						g_operator_switchability[min][max] = 1;
					return true;
				}
			}
		}
	}
	for (int i = 0; i < op2_prepost.size(); i++) {
		int var = op2_prepost[i].var;
		int pre = op2_prepost[i].pre;
		int post = op2_prepost[i].post;
		for (int j = 0; j < op1_prevail.size(); j++) {
			if (op1_prevail[j].var == var) {
				int other_prevail = op1_prevail[j].prev;
				if (pre == other_prevail || post == other_prevail) {
					if (g_operator_switchability)
						g_operator_switchability[min][max] = 1;
					return true;
				}
			}
		}
	}
	if (g_operator_switchability)
		g_operator_switchability[min][max] = 2;
	return false;
}

void create_action_graph() {
	cout << "creating action graph, number of nodes: " << g_operators.size()
			<< endl;
	int num_of_edges = 0;
	ofstream action_graph_file;
	action_graph_file.open("temp_action_graph");
	//action_graph_file << g_operators.size() << " "
	//	<< "number of edges goes here...\n";
	for (int i = 0; i < g_operators.size(); i++) {
		for (int j = 0; j < g_operators.size(); j++) {
			if (i != j && should_be_connected_in_action_graph(i, j)) {
				action_graph_file << j + 1 << " ";
				if (i < j)
					num_of_edges++;
			}
		}
		action_graph_file << "\n";
	}

	action_graph_file.close();
	string inbuf;
	fstream input_file("temp_action_graph", ios::in);
	ofstream output_file("action_graph");

	output_file << g_operators.size() << " " << num_of_edges << "\n";
	while (!input_file.eof()) {
		getline(input_file, inbuf);
		output_file << inbuf << endl;
	}

	input_file.close();
	output_file.close();

	if (remove("temp_action_graph") != 0)
		perror("Error deleting file");
	else
		puts("Action graph file created, temp action graph file deleted");

}

double update_private_and_public(vector<int> num_of_private_actions,
		vector<int> num_of_actions) {
	//	cout << "done." << endl;
	for (int i = 0; i < g_operators.size(); i++) {
		if (!g_operators[i].is_public
				&& g_operators[i].affect_goal_variable()) {
			//this sets all goal achieving actions as public.
			g_operators[i].is_public = true;
			num_of_private_actions[g_operators[i].agent]--;
			g_num_of_public_actions++;
		}
		for (int j = i + 1; j < g_operators.size(); j++) {
			if (g_operators[i].agent != g_operators[j].agent
					&& should_be_connected_in_action_graph(i, j)) {
				if (!g_operators[i].is_public) {
					g_operators[i].is_public = true;
					num_of_private_actions[g_operators[i].agent]--;
					g_num_of_public_actions++;
				}
				if (!g_operators[j].is_public) {
					g_operators[j].is_public = true;
					num_of_private_actions[g_operators[j].agent]--;
					g_num_of_public_actions++;
				}
			}

		}

		//g_operators[i].dump();
	}

	double symmetry_factor = 0;
	for (int i = 0; i < num_of_actions.size(); i++) {
		//		cout << "Agent " << i << ": " << num_of_private_actions[i] << "/"
		//				<< num_of_actions[i] << " private actions" << endl;
		symmetry_factor += (num_of_private_actions[i]
				/ (double) g_operators.size())
				* ((g_operators.size() - num_of_actions[i])
						/ (double) g_operators.size());
	}
	cout << "Symmetry factor is: " << symmetry_factor << endl;
	return symmetry_factor;
}

double update_ops_with_agents(const char* num_of_partitions) {
	string inbuf;
	string filename = "action_graph.part.";
	filename.append(num_of_partitions);
	//cout << filename << endl;
	//cout << filename.length() << endl;
	fstream partition_file(filename.c_str(), ios::in);
	//	cout<<"opened partition file"<<endl;
	g_num_of_agents = atoi(num_of_partitions);
	vector<int> num_of_actions(g_num_of_agents, 0);
	vector<int> num_of_private_actions(g_num_of_agents, 0);

	for (int i = 0; i < g_operators.size(); i++) {
		getline(partition_file, inbuf);
		int agent = atoi(inbuf.c_str());
		g_operators[i].agent = agent;
		num_of_actions[agent]++;
		num_of_private_actions[agent]++;
	}
	//	cout << "removing partition file...";
	partition_file.close();
	remove(filename.c_str());
	double symmetry_factor = update_private_and_public(num_of_private_actions,
			num_of_actions);

	return symmetry_factor;
	//computing the results of our formula:

}
void reset_op_agents() {
	//cout << "resetting operators.." << endl;
	for (int i = 0; i < g_operators.size(); i++) {
		g_operators[i].agent = -1;
		g_operators[i].is_public = false;
	}

}

void perform_optimal_partition() {

	FILE* file = NULL;
//	if (correlation_index == -1)
	file = fopen("PartSymmetry.csv", "w");

	//---------------------------

	create_action_graph();
	//cout << "Verifying correctness of action graph file...";
	int res_graphchk = system("graphchk action_graph");
	cout << res_graphchk << endl;

	string ufactor_values[] = { "-ufactor=100", "-ufactor=300", "-ufactor=500",
			"-ufactor=700", "-ufactor=900", "-ufactor=1100", "-ufactor=1300",
			"-ufactor=1500", "-ufactor=1700", "-ufactor=1900" };
	string objtype_values[] = { "-objtype=cut", "-objtype=vol" };
	string num_of_agents_values[] = { "2", "3", "4", "5", "6" };
	string ptype_values[] = { "-ptype=rb", "-ptype=kway" };
	string ctype_values[] = { "-ctype=rm", "-ctype=shem" };
	string buffer;	// [10000];
	string best_config_found;
	double best_symmetry_score_found = 0.0;
	int res_gpmetis = 0;

//	int cnt = 0;
	for (int ufactor = 0; ufactor < 10; ufactor += 2) {
		for (int ptype = 0; ptype < 2; ptype++) {
			for (int ctype = 1; ctype < 2; ctype++) { //only shem for now..
				for (int objtype = 0; objtype < 2; objtype++) {
					for (int agents = 0; agents < 4; agents++) {
						//cout << ufactor_values[ufactor] << ", "<< objtype_values[objtype] << ", "<< num_of_agents_values[agents] <<endl;
//						if (correlation_index > -1 && cnt++
//								!= correlation_index)
//							continue;

						buffer.clear();
						buffer = "gpmetis ";
						buffer += ufactor_values[ufactor];
						buffer += " ";
						buffer += ptype_values[ptype];
						buffer += " ";
						buffer += ctype_values[ctype];
						buffer += " ";
						if (ptype == 1)
							buffer += objtype_values[objtype];
						buffer += " action_graph ";
						buffer += num_of_agents_values[agents];

						//sprintf (buffer, "gpmetis %s %s action_graph %s", (char*)ufactor_values[ufactor],(char*)objtype_values[objtype],(char*)num_of_agents_values[agents]);
						//cout << buffer << endl;
						res_gpmetis = system(buffer.c_str());
						cout << "ignore: " << res_gpmetis << endl;
						double res = update_ops_with_agents(
								num_of_agents_values[agents].c_str());

						//-------------------
						if (file)
							fprintf(file, "%s,%f\n", buffer.data(), res);
						//-------------------
						//cout<<"finished updating operator agents, symmetry score is: " << res << endl;
						if (res > best_symmetry_score_found) {
							best_symmetry_score_found = res;
							best_config_found.clear();
							best_config_found = buffer.c_str();
							//					cout << "best symmetry score until now is: "
							//							<< best_symmetry_score_found << endl;
						}

						reset_op_agents();
					}
				}
			}
		}
	}

	if (best_symmetry_score_found > 0) {
		cout << "best score: " << best_symmetry_score_found << ", with config "
				<< best_config_found << endl;
		res_gpmetis = system(best_config_found.c_str());
		//cout << best_config_found[best_config_found.length()- 1] << endl;
		const char* partitions = best_config_found.substr(
				best_config_found.length() - 1, 1).c_str();

		double best_score = update_ops_with_agents(partitions);
		cout << "Best score is: " << best_score << endl;
	} else {
		cout << "no partition with score > 0 found.." << endl;
	}
	//	int res_gpmetis =
	//			system("gpmetis -ufactor=800 -objtype=vol action_graph 4");
	//	cout << "Return values of graphchk and gpmetis: " << res_graphchk << ", "
	//			<< res_gpmetis << endl;

	//----------------------
	if (file)
		fclose(file);
}

/*
 * MA-A* stuff
 */
void initialize_communication(const char* configFileName) {
	// Load Multi-Agent Communication module
	// with the configuration file, specified by the first command line argument
	//MAComm ma(comm_argv[1]);
	g_ma_comm = new MAComm(configFileName, g_agent_id);
	// Connect to all agents
	g_ma_comm->connect();
	// Get a copy of the module's configuration
	g_comm_config = g_ma_comm->getConfigCopy();
	g_agent_id = g_ma_comm->getConfigCopy().thisId;

	//These are used for maintaining the current solution and who confirmed it. //TODO - removed when implementing multi_goal changes
//	g_g_of_current_solution = -1;
//	g_num_of_agents_confirming_current_solution = 0;
//	for (int i = 0; i < g_comm_config.nAgents(); i++) {
//		g_agents_confirming_current_solution.push_back(false);
//	}
//	for (int i = 0; i < g_comm_config.nAgents(); i++) {
//		cout << g_agents_confirming_current_solution[i] << ",";
//	}
//	cout << endl;
}
void close_connection() {
	g_ma_comm->disconnect();
}
