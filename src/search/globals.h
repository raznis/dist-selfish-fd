#ifndef GLOBALS_H
#define GLOBALS_H

#include "operator_cost.h"

#include <iosfwd>
#include <string>
#include <vector>

class AxiomEvaluator;
class CausalGraph;
class DomainTransitionGraph;
class Operator;
class Axiom;
class State;
class SuccessorGenerator;
class Timer;
class RandomNumberGenerator;

bool test_goal(const State &state);
void save_plan(const std::vector<const Operator *> &plan, int iter);
int calculate_plan_cost(const std::vector<const Operator *> &plan);

void read_everything(std::istream &in);
void dump_everything();

void verify_no_axioms_no_cond_effects();

void check_magic(std::istream &in, std::string magic);

bool are_mutex(const std::pair<int, int> &a, const std::pair<int, int> &b);


extern bool g_use_metric;
extern int g_min_action_cost;
extern int g_max_action_cost;

// TODO: The following five belong into a new Variable class.
extern std::vector<std::string> g_variable_name;
extern std::vector<int> g_variable_domain;
extern std::vector<std::vector<std::string> > g_fact_names;
extern std::vector<int> g_axiom_layers;
extern std::vector<int> g_default_axiom_values;

extern State *g_initial_state;
extern std::vector<std::pair<int, int> > g_goal;

extern std::vector<Operator> g_operators;
extern std::vector<Operator> g_axioms;
extern AxiomEvaluator *g_axiom_evaluator;
extern SuccessorGenerator *g_successor_generator;
extern std::vector<DomainTransitionGraph *> g_transition_graphs;
extern CausalGraph *g_causal_graph;
extern Timer g_timer;
extern std::string g_plan_filename;
extern RandomNumberGenerator g_rng;

extern bool g_symmetry_pruning;
extern char **g_operator_switchability;
extern int g_num_of_public_actions;
extern void perform_optimal_partition();

extern bool g_agents_search;
extern std::vector<int> g_variable_agent;	//(Raz) to which agent the variable belongs to if it's private, or -1 if public.
void partition_by_agent_names(const char* configFileName);
extern bool g_marginal_search;
extern int g_marginal_agent;
extern int g_num_of_agents;
extern bool g_multiple_goal;
extern std::vector<int> g_marginal_solution_for_agent;
extern bool did_agent_participate(const std::vector<const Operator *> &plan, int agent);
#endif
