#ifndef _Message_
#define _Message_

#include "state_var_t.h"
#include "globals.h"
class SearchNode;
class Operator;
class State;
class StateProxy;

enum MessageType {
	SEARCH_NODE,
	SOLUTION_NODE,
	SOLUTION_CONFIRMATION,
	TERMINATION,
	ACKNOWLEDGEMENT,
	TRACE_BACK,
	MSG_INIT,
	MSG_FIN
};

#pragma pack(1)
struct Message {
	unsigned char sender_id;
	int message_id;
	unsigned char dest_id;
	unsigned char msgType;
	int g;
	//int real_g;
	int h;
	unsigned short creating_op_index; //This will work only if the order of operators is constant between agents.
	//const state_var_t *parent_state;	//I think this isn't necessary
	int num_of_vars; //This is the size of vars.
	int num_of_actions_in_plan;

	state_var_t* vars; // values for vars
	int* operator_indices_of_plan;	//this holds the plan (indices of operators in the plan)

private:
	int varsOffset() const {
		return (char*) (&this->vars) - (char*) this;
	}

public:
	Message();
	Message(State* new_state, const Operator *creating_op, int g_value, int h_value,
			unsigned char destination_id, unsigned char mes_type);
	Message(StateProxy &current_state, vector<const Operator *> &path, int target_agent);
	~Message();
	void dump() const;
	SearchNode* translate_to_search_node() const;

	int serialize(char* buffer) const;
	static Message* deserialize(char* buffer);
};
#pragma pack()

#endif
