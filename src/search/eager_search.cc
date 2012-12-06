#include "eager_search.h"

#include "globals.h"
#include "heuristic.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "g_evaluator.h"
#include "sum_evaluator.h"
#include "plugin.h"
#include "message.h"

#include <cassert>
#include <cstdlib>
#include <set>
using namespace std;

EagerSearch::EagerSearch(const Options &opts) :
		SearchEngine(opts), reopen_closed_nodes(opts.get<bool>("reopen_closed")), do_pathmax(opts.get<bool>("pathmax")), use_multi_path_dependence(
				opts.get<bool>("mpd")), open_list(opts.get<OpenList<state_var_t *> *>("open")), f_evaluator(
				opts.get<ScalarEvaluator *>("f_eval")) {
	if (opts.contains("preferred")) {
		preferred_operator_heuristics = opts.get_list<Heuristic *>("preferred");
	}
}

void EagerSearch::initialize() {
	//TODO children classes should output which kind of search
	cout << "Agent " << g_agent_id << " conducting best first search" << (reopen_closed_nodes ? " with" : " without")
			<< " reopening closed nodes, (real) bound = " << bound << endl;
	if (do_pathmax)
		cout << "Using pathmax correction" << endl;
	if (use_multi_path_dependence)
		cout << "Using multi-path dependence (LM-A*)" << endl;
	assert(open_list != NULL);

	set<Heuristic *> hset;
	open_list->get_involved_heuristics(hset);

	for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
		estimate_heuristics.push_back(*it);
		search_progress.add_heuristic(*it);
	}

	// add heuristics that are used for preferred operators (in case they are
	// not also used in the open list)
	hset.insert(preferred_operator_heuristics.begin(), preferred_operator_heuristics.end());

	// add heuristics that are used in the f_evaluator. They are usually also
	// used in the open list and hence already be included, but we want to be
	// sure.
	if (f_evaluator) {
		f_evaluator->get_involved_heuristics(hset);
	}

	for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
		heuristics.push_back(*it);
	}

	assert(!heuristics.empty());

	for (size_t i = 0; i < heuristics.size(); i++)
		heuristics[i]->evaluate(*g_initial_state);
	open_list->evaluate(0, false);
	search_progress.inc_evaluated_states();
	search_progress.inc_evaluations(heuristics.size());

	if (open_list->is_dead_end()) {
		cout << "Initial state is a dead end." << endl;
	} else {
		search_progress.get_initial_h_values();
		if (f_evaluator) {
			f_evaluator->evaluate(0, false);
			search_progress.report_f_value(f_evaluator->get_value());
		}
		search_progress.check_h_progress(0);
		SearchNode node = search_space.get_node(*g_initial_state);
		node.open_initial(heuristics[0]->get_value());

		open_list->insert(node.get_state_buffer());
	}
}

void EagerSearch::statistics() const {
	search_progress.print_statistics();
	search_space.statistics();
}

int EagerSearch::step() {
	receive_messages();

	//TERMINATION DETECTION
	if (g_multiple_goal) {
		bool all_terminate = true;
		for (int i = 0; i <= g_num_of_agents && all_terminate; i++) {
			all_terminate = g_received_termination[i];
		}
		if (all_terminate) {
			cout << "Received Termination messages for all agents!" << endl;
			return SOLVED;
		}
		for (int marginal_agent = 0; marginal_agent <= g_num_of_agents; marginal_agent++) {
			if (((marginal_agent == g_num_of_agents && g_num_of_agents_confirming_current_solution[marginal_agent] == g_num_of_agents - 1)
					|| (marginal_agent < g_num_of_agents
							&& g_num_of_agents_confirming_current_solution[marginal_agent] == g_num_of_agents - 2))
					&& !g_received_termination[marginal_agent]) {		//Marginal agent doesn't need to confirm its marginal solution
				cout << "SOLVED MARGINAL PROBLEM " << marginal_agent << ", solution cost is " << g_g_of_current_solution[marginal_agent]
						<< endl;
				g_received_termination[marginal_agent] = true;
				//send out termination message
				for (int i = 0; i < g_num_of_agents; i++) {
					Message* m = new Message(g_current_solution[marginal_agent], 0, g_g_of_current_solution[marginal_agent], marginal_agent,
							i, TERMINATION, vector<bool>());
					g_ma_comm->sendMessage(m);
				}
			}
		}

	} else {
		if (g_received_termination[g_num_of_agents]) {
			//cout << "Received Termination message (for optimal solution)!" << endl;
			return SOLVED;
		}
		if (g_num_of_agents_confirming_current_solution[g_num_of_agents] == g_num_of_agents - 1
				|| (g_marginal_search && g_num_of_agents_confirming_current_solution[g_num_of_agents] == g_num_of_agents - 2)) {//current solution confirmed by all other agents
			cout << "SOLVED!!     Solution cost is: " << g_g_of_current_solution[g_num_of_agents] << endl;
			//TODO - send out termination message so that agents will get ready for the trace back
			for (int i = 0; i < g_num_of_agents; i++) {
				Message* m = new Message(g_current_solution[g_num_of_agents], 0, g_g_of_current_solution[g_num_of_agents], g_num_of_agents,
						i, TERMINATION, vector<bool>());
				g_ma_comm->sendMessage(m);
			}
			return SOLVED;
		}
	}
	pair<SearchNode, bool> n = fetch_next_node();
	if (!n.second) {
		//TODO - this is a hack to prevent agents from exiting before messages arrive.
		//return FAILED;
		//sleep(1);
		return IN_PROGRESS;
	}
	SearchNode node = n.first;

	State s = node.get_state();
	if (check_goal_and_set_plan(s)) {
		if (g_multiple_goal) {
			if (node.get_creating_op()->agent == g_agent_id) {		//I own this goal state
				vector<int> new_solutions_found;	//holds the marginal agents for which new solutions have been found
				for (int marginal_agent = 0; marginal_agent <= g_num_of_agents; marginal_agent++) {	//last iteration is the optimal solution
					if ((marginal_agent < g_num_of_agents && !node.get_participating_agents()[marginal_agent])
							|| marginal_agent == g_num_of_agents) { //marginal agent does not participate
						if (g_g_of_current_solution[marginal_agent] == -1 || g_g_of_current_solution[marginal_agent] > node.get_g()) { // this is the first solution I found or this is a better solution than the best one found so far.
							cout << "Found solution with g=" << node.get_g() << " for marginal agent " << marginal_agent
									<< ", Participating agents: ";
							for (int i = 0; i < g_num_of_agents; i++)
								cout << node.get_participating_agents()[i] << ", ";
							cout << endl;
							delete (g_current_solution[marginal_agent]);
							g_current_solution[marginal_agent] = new State(s);
							g_g_of_current_solution[marginal_agent] = node.get_g();
							g_num_of_agents_confirming_current_solution[marginal_agent] = 0;
							new_solutions_found.push_back(marginal_agent);
						}
					}
				}
				for (int i = 0; i < new_solutions_found.size(); i++) {
					int marginal_agent = new_solutions_found[i];
					for (int i = 0; i < g_num_of_agents; i++) {
						if (i != marginal_agent) { //Sending to all agents except the marginal agent
							Message* m = new Message(&s, node.get_creating_op(), node.get_g(), 0, i, SEARCH_NODE,
									node.get_participating_agents());
							g_ma_comm->sendMessage(m);
						}
					}
				}
			} else {	//This goal state was sent to me
				for (int marginal_agent = 0; marginal_agent <= g_num_of_agents; marginal_agent++) {	//last iteration is the optimal solution
					if (marginal_agent != g_agent_id
							&& ((marginal_agent < g_num_of_agents && !node.get_participating_agents()[marginal_agent])
									|| marginal_agent == g_num_of_agents)) { //marginal agent does not participate

						if (g_g_of_current_solution[marginal_agent] == -1 || g_g_of_current_solution[marginal_agent] >= node.get_g()) { // this is the first solution I saw or this is a better or equal solution than the best one found so far.
								//send confirmation message
							cout << "Sending confirmation for marginal problem " << marginal_agent << " to agent "
									<< node.get_creating_op()->agent << endl;
							g_g_of_current_solution[marginal_agent] = node.get_g();
							Message* m = new Message(&s, node.get_creating_op(), node.get_g(), marginal_agent,
									node.get_creating_op()->agent, SOLUTION_CONFIRMATION, node.get_participating_agents()); //TODO - HACK HACK  using h value as the marginal agent that's relevant for this solution confirmation
							g_ma_comm->sendMessage(m);
						}
					}
				}
			}
		} else {	//NOT PERFORMING MULTI_GOAL SEARCH
			int marginal_agent = g_num_of_agents;	//HACK HACK this represents the index of the optimal solution
			if (node.get_creating_op()->agent == g_agent_id) { //I own this goal state

				if (g_g_of_current_solution[marginal_agent] == -1 || g_g_of_current_solution[marginal_agent] > node.get_g()) { // this is the first solution I found or this is a better solution than the best one found so far.
					cout << "Found solution with g=" << node.get_g() << ", Participating agents: ";
					for (int i = 0; i < g_num_of_agents; i++)
						cout << node.get_participating_agents()[i] << ", ";
					cout << endl;
					delete (g_current_solution[marginal_agent]);
					g_current_solution[marginal_agent] = new State(s);
					g_g_of_current_solution[marginal_agent] = node.get_g();
					g_num_of_agents_confirming_current_solution[marginal_agent] = 0;
					for (int i = 0; i < g_num_of_agents; i++) {
						Message* m = new Message(&s, node.get_creating_op(), node.get_g(), 0, i, SEARCH_NODE,
								node.get_participating_agents());
						g_ma_comm->sendMessage(m);
					}
				}
			} else {	//This goal state was sent to me
				if (g_g_of_current_solution[marginal_agent] == -1 || g_g_of_current_solution[marginal_agent] >= node.get_g()) { // this is the first solution I saw or this is a better or equal solution than the best one found so far.
						//send confirmation message
					cout << "Sending confirmation of solution to agent " << node.get_creating_op()->agent << endl;
					g_g_of_current_solution[marginal_agent] = node.get_g();
					Message* m = new Message(&s, node.get_creating_op(), node.get_g(), marginal_agent, node.get_creating_op()->agent,
							SOLUTION_CONFIRMATION, node.get_participating_agents()); //TODO - HACK HACK  using h value as the marginal agent that's relevant for this solution confirmation
					g_ma_comm->sendMessage(m);
				}
			}
		}
		//return SOLVED;
		return IN_PROGRESS;
	}
	//pruning states that are no longer relevant in the search for marginal problems
	if (g_multiple_goal && !node.is_relevant_for_marginal_search()) {
		return IN_PROGRESS;
	}

//	const Operator * creating_op = node.get_creating_op();
//	bool apply_ma_pruning = (g_agents_search || g_symmetry_pruning)
//			&& creating_op && !creating_op->is_public;
//	int creating_op_agent = -1;
//	if (creating_op) {
//		creating_op_agent = creating_op->agent;
//	}

	//Sending states created by public actions to relevant agents.
	if (node.get_creating_op() && node.get_creating_op()->is_public)
		send_state_to_relevant_agents(node.get_creating_op(), s, node.get_g(), node.get_h(), node.get_participating_agents());

	vector<const Operator *> applicable_ops;
	set<const Operator *> preferred_ops;

	g_successor_generator->generate_applicable_ops(s, applicable_ops);
	// This evaluates the expanded state (again) to get preferred ops
	for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
		Heuristic *h = preferred_operator_heuristics[i];
		h->evaluate(s);
		if (!h->is_dead_end()) {
			// In an alternation search with unreliable heuristics, it is
			// possible that this heuristic considers the state a dead end.
			vector<const Operator *> preferred;
			h->get_preferred_operators(preferred);
			preferred_ops.insert(preferred.begin(), preferred.end());
		}
	}
	search_progress.inc_evaluations(preferred_operator_heuristics.size());

	for (int i = 0; i < applicable_ops.size(); i++) {
		const Operator *op = applicable_ops[i];

		if ((node.get_real_g() + op->get_cost()) >= bound)
			continue;

		//pruning actions not belonging to this agent
		if (op->agent != g_agent_id) {
			continue;
		}

		//pruning action of marginal agents
		//TODO - a smarter way to do this is to alter the problem so that it doesn't contain the marginal agent to begin with.
		//This will make the heuristic estimate much better.
		if (g_marginal_search && g_marginal_agent == op->agent)
			continue;

		//if performing this action adds the last agent then prune it, saving heuristic calculation
		if (g_multiple_goal && !node.is_state_with_agent_action_relevant_for_marginal_search(op->agent)) {
			continue;
		}

		State succ_state(s, *op);
		handle_state(succ_state, op, preferred_ops, node, s);
	}

	return IN_PROGRESS;
}

void EagerSearch::send_state_to_relevant_agents(const Operator* op, State new_state, int g, int h, vector<bool> participating) {
	bool* sent_message_to_agent = new bool[g_comm_config.nAgents()];
	for (int i = 0; i < g_comm_config.nAgents(); i++)
		sent_message_to_agent[i] = false;
	for (int op_idx = 0; op_idx < g_operators.size(); op_idx++) {
		Operator curr_op = g_operators[op_idx];
		if (sent_message_to_agent[curr_op.agent] || !curr_op.is_public || curr_op.agent == g_agent_id
				|| (g_marginal_search && g_marginal_agent == curr_op.agent))
			continue;
		//curr_op is public, owned by a different agent, which we did not mark as one to send a message to.
		if (curr_op.is_applicable_public(new_state)) {
			Message* m = new Message(&new_state, op, g, h, (int) curr_op.agent, SEARCH_NODE, participating);
			g_ma_comm->sendMessage(m);
			sent_message_to_agent[curr_op.agent] = true;
		}
	}

	delete[] sent_message_to_agent;
}

void EagerSearch::handle_state(State succ_state, const Operator *op, set<const Operator *> preferred_ops, SearchNode node, State s) {
	search_progress.inc_generated();
	bool is_preferred = (preferred_ops.find(op) != preferred_ops.end());

	SearchNode succ_node = search_space.get_node(succ_state);

	// Previously encountered dead end. Don't re-evaluate.
	if (succ_node.is_dead_end())
		return;

	// update new path
	if (use_multi_path_dependence || succ_node.is_new()) {
		bool h_is_dirty = false;
		for (size_t i = 0; i < heuristics.size(); i++)
			h_is_dirty = h_is_dirty || heuristics[i]->reach_state(s, *op, succ_node.get_state());
		if (h_is_dirty && use_multi_path_dependence)
			succ_node.set_h_dirty();
	}

	if (succ_node.is_new()) {
		// We have not seen this state before.
		// Evaluate and create a new node.
		for (size_t i = 0; i < heuristics.size(); i++)
			heuristics[i]->evaluate(succ_state);
		succ_node.clear_h_dirty();
		search_progress.inc_evaluated_states();
		search_progress.inc_evaluations(heuristics.size());

		// Note that we cannot use succ_node.get_g() here as the
		// node is not yet open. Furthermore, we cannot open it
		// before having checked that we're not in a dead end. The
		// division of responsibilities is a bit tricky here -- we
		// may want to refactor this later.
		open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
		bool dead_end = open_list->is_dead_end();
		if (dead_end) {
			succ_node.mark_as_dead_end();
			search_progress.inc_dead_ends();
			return;
		}

		//TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
		int succ_h = heuristics[0]->get_heuristic();
		if (do_pathmax) {
			if ((node.get_h() - get_adjusted_cost(*op)) > succ_h) {
				//cout << "Pathmax correction: " << succ_h << " -> " << node.get_h() - get_adjusted_cost(*op) << endl;
				succ_h = node.get_h() - get_adjusted_cost(*op);
				heuristics[0]->set_evaluator_value(succ_h);
				open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
				search_progress.inc_pathmax_corrections();
			}
		}
		succ_node.open(succ_h, node, op);

		open_list->insert(succ_node.get_state_buffer());
		if (search_progress.check_h_progress(succ_node.get_g())) {
			reward_progress();
		}
	} else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
		// We found a new cheapest path to an open or closed state.
		if (reopen_closed_nodes) {
			//TODO:CR - test if we should add a reevaluate flag and if it helps
			// if we reopen closed nodes, do that
			if (succ_node.is_closed()) {
				/* TODO: Verify that the heuristic is inconsistent.
				 * Otherwise, this is a bug. This is a serious
				 * assertion because it can show that a heuristic that
				 * was thought to be consistent isn't. Therefore, it
				 * should be present also in release builds, so don't
				 * use a plain assert. */
				//TODO:CR - add a consistent flag to heuristics, and add an assert here based on it
				search_progress.inc_reopened();
			}
			succ_node.reopen(node, op);
			heuristics[0]->set_evaluator_value(succ_node.get_h());
			// TODO: this appears fishy to me. Why is here only heuristic[0]
			// involved? Is this still feasible in the current version?
			open_list->evaluate(succ_node.get_g(), is_preferred);

			open_list->insert(succ_node.get_state_buffer());
		} else {
			// if we do not reopen closed nodes, we just update the parent pointers
			// Note that this could cause an incompatibility between
			// the g-value and the actual path that is traced back
			succ_node.update_parent(node, op);
			cout << "SHOULDN'T BE HERE IN MA-FD!!!!!!!!!!!!!!!!!!!!!!!!!!!   Must always reopen nodes!!!!!" << endl;
		}
	}
}

void EagerSearch::handle_state_from_message(State succ_state, const Operator *op, int g_from_message, int h_from_message,
		bool* participating) {
	search_progress.inc_generated();
	bool is_preferred = true;

	vector<bool> participating_agents;
	for (int i = 0; i < g_num_of_agents; i++) {
		participating_agents.push_back((bool) participating[i]);
	}
	SearchNode succ_node = search_space.get_node(succ_state);

	// Previously encountered dead end. Don't re-evaluate.
	if (succ_node.is_dead_end())
		return;

	// update new path
//	if (use_multi_path_dependence || succ_node.is_new()) {
//		bool h_is_dirty = false;
//		for (size_t i = 0; i < heuristics.size(); i++)
//			h_is_dirty = h_is_dirty
//					|| heuristics[i]->reach_state(s, *op,
//							succ_node.get_state());
//		if (h_is_dirty && use_multi_path_dependence)
//			succ_node.set_h_dirty();
//	}

	if (succ_node.is_new()) {
		// We have not seen this state before.
		// Evaluate and create a new node.
		for (size_t i = 0; i < heuristics.size(); i++)
			heuristics[i]->evaluate(succ_state);
		succ_node.clear_h_dirty();
		search_progress.inc_evaluated_states();
		search_progress.inc_evaluations(heuristics.size());

		// Note that we cannot use succ_node.get_g() here as the
		// node is not yet open. Furthermore, we cannot open it
		// before having checked that we're not in a dead end. The
		// division of responsibilities is a bit tricky here -- we
		// may want to refactor this later.
		open_list->evaluate(g_from_message, is_preferred);
		bool dead_end = open_list->is_dead_end();
		if (dead_end) {
			succ_node.mark_as_dead_end();
			search_progress.inc_dead_ends();
			return;
		}

		//TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
		int succ_h = max(heuristics[0]->get_heuristic(), h_from_message); //Updating the h value to be the maximal of this agent's h value and the sending agent's h value.
		if (do_pathmax) {
			cout << "SHOULD NOT BE HERE IN MA_FD - do_pathmax not implemented yet..." << endl;
//			if ((node.get_h() - get_adjusted_cost(*op)) > succ_h) {
//				//cout << "Pathmax correction: " << succ_h << " -> " << node.get_h() - get_adjusted_cost(*op) << endl;
//				succ_h = node.get_h() - get_adjusted_cost(*op);
//				heuristics[0]->set_evaluator_value(succ_h);
//				open_list->evaluate(node.get_g() + get_adjusted_cost(*op),
//						is_preferred);
//				search_progress.inc_pathmax_corrections();
//			}
		}
		succ_node.open(succ_h, g_from_message, op, succ_state, participating_agents);

		open_list->insert(succ_node.get_state_buffer());
		if (search_progress.check_h_progress(succ_node.get_g())) {
			reward_progress();
		}
	} else if (succ_node.get_g() > g_from_message) {
		// We found a new cheapest path to an open or closed state.
		if (reopen_closed_nodes) {
			//TODO:CR - test if we should add a reevaluate flag and if it helps
			// if we reopen closed nodes, do that
			if (succ_node.is_closed()) {
				/* TODO: Verify that the heuristic is inconsistent.
				 * Otherwise, this is a bug. This is a serious
				 * assertion because it can show that a heuristic that
				 * was thought to be consistent isn't. Therefore, it
				 * should be present also in release builds, so don't
				 * use a plain assert. */
				//TODO:CR - add a consistent flag to heuristics, and add an assert here based on it
				search_progress.inc_reopened();
			}
			succ_node.reopen(g_from_message, op, succ_state, participating_agents);
			heuristics[0]->set_evaluator_value(succ_node.get_h());
			// TODO: this appears fishy to me. Why is here only heuristic[0]
			// involved? Is this still feasible in the current version?
			open_list->evaluate(succ_node.get_g(), is_preferred);

			open_list->insert(succ_node.get_state_buffer());
		} else {
			// if we do not reopen closed nodes, we just update the parent pointers
			// Note that this could cause an incompatibility between
			// the g-value and the actual path that is traced back
			cout << "SHOULD ALWAYS REOPEN CLOSED NODES IN MA-A*!" << endl;
			//succ_node.update_parent(node, op);
			//TODO - For non-optimal implementation, should call update_parent in order to have trace_back work correctly.
		}
	}
}

void EagerSearch::receive_messages() {
	//cout << "receiving messages...";
	while (!g_ma_comm->noIncomingMessage()) {
		Message* m = g_ma_comm->receiveMessage();
		State new_state = State(m);
		if (g_marginal_search && m->sender_id == g_marginal_agent)
			continue;
		if (m->msgType == SEARCH_NODE) {
			handle_state_from_message(new_state, ((const Operator*) &g_operators[m->creating_op_index]), (int) m->g, (int) m->h,
					((bool*) m->participating_agents));
		} else if (m->msgType == SOLUTION_CONFIRMATION) {
			int marginal_agent = (int) m->h;//TODO - HACK HACK  using h value as the marginal agent that's relevant for this solution confirmation
			if (!g_current_solution[marginal_agent]) {	//TODO - need to handle this case. Maybe push back into message queue ???
				cout << "PROBLEM!!!!!!    received solution confirmation from agent " << (int) m->sender_id << " for marginal problem "
						<< marginal_agent << " without having a solution!!!        Participating agents: ";
				for (int i = 0; i < g_num_of_agents; i++)
					cout << ((bool*) m->participating_agents)[i] << ", ";
				cout << endl;
			} else {
				if (g_current_solution[marginal_agent]->operator ==(new_state)) {
					g_num_of_agents_confirming_current_solution[marginal_agent]++;
					if (g_multiple_goal)
						cout << "received confirmation for marginal problem " << marginal_agent << " from agent " << (int) m->sender_id
								<< ", number of confirming agents is " << g_num_of_agents_confirming_current_solution[marginal_agent]
								<< endl;
					else
						cout << "received confirmation from agent " << (int) m->sender_id << ", number of confirming agents is "
								<< g_num_of_agents_confirming_current_solution[marginal_agent] << endl;
				}
			}
		} else if (m->msgType == TERMINATION) {
			g_received_termination[(int) m->h] = true;
			if (g_multiple_goal){
				cout << "received termination for marginal problem " << (int) m->h << endl;
				continue;
			}
			cout << "received termination message" << endl;
			return;
		}

	}
	//cout << "done" << endl;
}

pair<SearchNode, bool> EagerSearch::fetch_next_node() {
	/* TODO: The bulk of this code deals with multi-path dependence,
	 which is a bit unfortunate since that is a special case that
	 makes the common case look more complicated than it would need
	 to be. We could refactor this by implementing multi-path
	 dependence as a separate search algorithm that wraps the "usual"
	 search algorithm and adds the extra processing in the desired
	 places. I think this would lead to much cleaner code. */

	while (true) {
		if (open_list->empty()) {
			//cout << "Completely explored state space -- no solution!" << endl;
			return make_pair(search_space.get_node(*g_initial_state), false);
		}
		vector<int> last_key_removed;
		State state(open_list->remove_min(use_multi_path_dependence ? &last_key_removed : 0));
		SearchNode node = search_space.get_node(state);

		if (node.is_closed())
			continue;

		if (use_multi_path_dependence) {
			assert(last_key_removed.size() == 2);
			int pushed_h = last_key_removed[1];
			assert(node.get_h() >= pushed_h);
			if (node.get_h() > pushed_h) {
				// cout << "LM-A* skip h" << endl;
				continue;
			}
			assert(node.get_h() == pushed_h);
			if (!node.is_closed() && node.is_h_dirty()) {
				for (size_t i = 0; i < heuristics.size(); i++)
					heuristics[i]->evaluate(node.get_state());
				node.clear_h_dirty();
				search_progress.inc_evaluations(heuristics.size());

				open_list->evaluate(node.get_g(), false);
				bool dead_end = open_list->is_dead_end();
				if (dead_end) {
					node.mark_as_dead_end();
					search_progress.inc_dead_ends();
					continue;
				}
				int new_h = heuristics[0]->get_heuristic();
				if (new_h > node.get_h()) {
					assert(node.is_open());
					node.increase_h(new_h);
					open_list->insert(node.get_state_buffer());
					continue;
				}
			}
		}

		node.close();
		assert(!node.is_dead_end());
		update_jump_statistic(node);
		search_progress.inc_expanded();
		return make_pair(node, true);
	}
}

void EagerSearch::reward_progress() {
	// Boost the "preferred operator" open lists somewhat whenever
	// one of the heuristics finds a state with a new best h value.
	open_list->boost_preferred();
}

void EagerSearch::dump_search_space() {
	search_space.dump();
}

void EagerSearch::update_jump_statistic(const SearchNode &node) {
	if (f_evaluator) {
		heuristics[0]->set_evaluator_value(node.get_h());
		f_evaluator->evaluate(node.get_g(), false);
		int new_f_value = f_evaluator->get_value();
		search_progress.report_f_value(new_f_value);
	}
}

void EagerSearch::print_heuristic_values(const vector<int> &values) const {
	for (int i = 0; i < values.size(); i++) {
		cout << values[i];
		if (i != values.size() - 1)
			cout << "/";
	}
}

static SearchEngine *_parse(OptionParser &parser) {
	//open lists are currently registered with the parser on demand,
	//because for templated classes the usual method of registering
	//does not work:
	Plugin<OpenList<state_var_t *> >::register_open_lists();

	parser.add_option<OpenList<state_var_t *> *>("open");
	parser.add_option<bool>("reopen_closed", false, "reopen closed nodes");
	parser.add_option<bool>("pathmax", false, "use pathmax correction");
	parser.add_option<ScalarEvaluator *>("f_eval", 0, "set evaluator for jump statistics");
	parser.add_list_option<Heuristic *>("preferred", vector<Heuristic *>(), "use preferred operators of these heuristics");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	EagerSearch *engine = 0;
	if (!parser.dry_run()) {
		opts.set<bool>("mpd", false);
		engine = new EagerSearch(opts);
	}

	return engine;
}

static SearchEngine *_parse_astar(OptionParser &parser) {
	parser.add_option<ScalarEvaluator *>("eval");
	parser.add_option<bool>("pathmax", false, "use pathmax correction");
	parser.add_option<bool>("mpd", false, "use multi-path dependence (LM-A*)");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	EagerSearch *engine = 0;
	if (!parser.dry_run()) {
		GEvaluator *g = new GEvaluator();
		vector<ScalarEvaluator *> sum_evals;
		sum_evals.push_back(g);
		ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
		sum_evals.push_back(eval);
		ScalarEvaluator *f_eval = new SumEvaluator(sum_evals);

		// use eval for tiebreaking
		std::vector<ScalarEvaluator *> evals;
		evals.push_back(f_eval);
		evals.push_back(eval);
		OpenList<state_var_t *> *open = new TieBreakingOpenList<state_var_t *>(evals, false, false);

		opts.set("open", open);
		opts.set("f_eval", f_eval);
		opts.set("reopen_closed", true);
		engine = new EagerSearch(opts);
	}

	return engine;
}

static SearchEngine *_parse_greedy(OptionParser &parser) {
	parser.add_list_option<ScalarEvaluator *>("evals");
	parser.add_list_option<Heuristic *>("preferred", vector<Heuristic *>(), "use preferred operators of these heuristics");
	parser.add_option<int>("boost", 0, "boost value for preferred operator open lists");
	SearchEngine::add_options_to_parser(parser);

	Options opts = parser.parse();
	opts.verify_list_non_empty<ScalarEvaluator *>("evals");

	EagerSearch *engine = 0;
	if (!parser.dry_run()) {
		vector<ScalarEvaluator *> evals = opts.get_list<ScalarEvaluator *>("evals");
		vector<Heuristic *> preferred_list = opts.get_list<Heuristic *>("preferred");
		OpenList<state_var_t *> *open;
		if ((evals.size() == 1) && preferred_list.empty()) {
			open = new StandardScalarOpenList<state_var_t *>(evals[0], false);
		} else {
			vector<OpenList<state_var_t *> *> inner_lists;
			for (int i = 0; i < evals.size(); i++) {
				inner_lists.push_back(new StandardScalarOpenList<state_var_t *>(evals[i], false));
				if (!preferred_list.empty()) {
					inner_lists.push_back(new StandardScalarOpenList<state_var_t *>(evals[i], true));
				}
			}
			open = new AlternationOpenList<state_var_t *>(inner_lists, opts.get<int>("boost"));
		}

		opts.set("open", open);
		opts.set("reopen_closed", false);
		opts.set("pathmax", false);
		opts.set("mpd", false);
		ScalarEvaluator *sep = 0;
		opts.set("f_eval", sep);
		opts.set("bound", numeric_limits<int>::max());
		opts.set("preferred", preferred_list);
		engine = new EagerSearch(opts);
	}
	return engine;
}

static Plugin<SearchEngine> _plugin("eager", _parse);
static Plugin<SearchEngine> _plugin_astar("astar", _parse_astar);
static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
