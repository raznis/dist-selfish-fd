#include "message.h"
#include "search_space.h"
#include "operator.h"
#include "state_proxy.h"

#include <string.h>

inline int get_op_index(const Operator *op) {
	/* TODO: The op_index computation is duplicated from
	 OperatorRegistry::get_op_index() and actually belongs neither
	 here nor there. There should be some canonical way of getting
	 from an Operator pointer to an index, but it's not clear how to
	 do this in a way that best fits the overall planner
	 architecture (taking into account axioms etc.) */
	int op_index = op - &*g_operators.begin();
	assert(op_index >= 0 && op_index < g_operators.size());
	return op_index;
}

Message::Message() {
	memset(this, 0, sizeof(Message));
}

Message::Message(State* new_state, const Operator *creating_op, int g_value,
		int h_value, unsigned char destination_id, unsigned char mes_type,
		vector<bool> participating) {
	dest_id = destination_id;
	creating_op_index = get_op_index(creating_op);
	g = g_value;
	h = h_value;
	//message_id = mess_id;
	//sender_id = send_id;
	msgType = mes_type;
	//real_g = -1; //not used for now...
	num_of_vars = g_variable_domain.size();
	const state_var_t *state_vars = new_state->get_buffer();
	vars = new state_var_t[g_variable_domain.size()];
	for (int i = 0; i < g_variable_domain.size(); i++)
		vars[i] = state_vars[i];

	// ToDo Raz: initialize num_of_actions_in_plan and operator_indices_of_plan
	num_of_actions_in_plan = 0;
	operator_indices_of_plan = 0;

	participating_agents = new bool[g_num_of_agents];
	if (g_multiple_goal) {
		for (int i = 0; i < g_num_of_agents; i++)
			participating_agents[i] = participating[i];
	}
}
//TRACE BACK MESSAGE
Message::Message(StateProxy &current_state, vector<const Operator *> &path,
		int target_agent) {
	//cout<<"starting to build the trace back message..." <<endl;
	dest_id = target_agent;
	//cout<<"sending to agent " << (int)dest_id << endl;
	creating_op_index = 0;
	g = -1;
	h = -1;
	msgType = TRACE_BACK;
	num_of_vars = g_variable_domain.size();
	//cout<<"num_of_vars is " << num_of_vars<< endl;
	vars = new state_var_t[num_of_vars];
	for (int i = 0; i < num_of_vars; i++) {
		//cout<<(int)current_state.state_data[i]<<",";
		vars[i] = current_state.state_data[i];
	}
	cout << endl;
	//cout<<"done updating vars..." <<endl;
	num_of_actions_in_plan = path.size();
	operator_indices_of_plan = new int[num_of_actions_in_plan];
	//cout<<"num of actions: "<<num_of_actions_in_plan<<endl;
	for (int i = 0; i < num_of_actions_in_plan; i++)
		operator_indices_of_plan[i] = get_op_index(path[i]);
	//cout<<"done building trace message."<<endl;
}

Message::~Message() {
	if (vars)
		delete[] vars;
	if (operator_indices_of_plan)
		delete[] operator_indices_of_plan;
	if (participating_agents)
		delete[] participating_agents;
}
//SHOULD BE USED ONLY WHEN RECEIVING MESSAGES
void Message::dump() const {
	cout << "Message " << message_id << " from agent " << (int) sender_id
			<< " to agent " << (int) dest_id << ", op index = "
			<< (int) creating_op_index << endl;
	cout << "g = " << g << ", h = " << h << ", message_type = " << (int) msgType
			<< ", num of vars = " << num_of_vars << endl;
	for (int i = 0; i < g_variable_domain.size(); i++) {
		cout << (int) vars[i] << ",";
	}
	cout << endl;
}

int Message::serialize(char* buffer) const {
	char* p = buffer;
	int fieldSize = varsOffset();
	memcpy(p, this, fieldSize);
	p += fieldSize;

	fieldSize = num_of_vars * sizeof(state_var_t);
	memcpy(p, vars, fieldSize);
	p += fieldSize;

	fieldSize = num_of_actions_in_plan * sizeof(int);
	memcpy(p, operator_indices_of_plan, fieldSize);
	p += fieldSize;

	if (g_multiple_goal && msgType == SEARCH_NODE) {
		fieldSize = g_num_of_agents * sizeof(bool);
		memcpy(p, participating_agents, fieldSize);
		p += fieldSize;
	}
	return p - buffer;
}

Message* Message::deserialize(char* buffer) {
	char* p = buffer;
	Message* m = new Message();
	int fieldSize = m->varsOffset();
	memcpy(m, p, fieldSize);
	p += fieldSize;

	if (m->num_of_vars) {
		fieldSize = m->num_of_vars * sizeof(state_var_t);
		m->vars = new state_var_t[m->num_of_vars];
		memcpy(m->vars, p, fieldSize);
		p += fieldSize;
	}

	if (m->num_of_actions_in_plan) {
		fieldSize = m->num_of_actions_in_plan * sizeof(int);
		m->operator_indices_of_plan = new int[m->num_of_actions_in_plan];
		memcpy(m->operator_indices_of_plan, p, fieldSize);
		p += fieldSize;
	}

	if (g_multiple_goal && m->msgType == SEARCH_NODE) {
		fieldSize = g_num_of_agents * sizeof(bool);
		m->participating_agents = new bool[g_num_of_agents];
		memcpy(m->participating_agents, p, fieldSize);
		p += fieldSize;
	}

	return m;
}
