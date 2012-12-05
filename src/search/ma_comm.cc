/*
 * ma_comm.cc
 *
 *  Created on: Jul 24, 2012
 *      Author: raznis
 */
#include "practical_socket.h"  // For Socket, ServerSocket, and SocketException
#include <iostream>           // For cout, cerr
#include <cstdlib>            // For atoi()
#include <iostream>
#include <fstream>
#include <string.h>
#include "ma_comm.h"
#include "message.h"

#define RCVBUFSIZE (8192)

//----------------------------------------------------------
struct ListenerData {
	int agent;
	MAComm* comm;

	ListenerData(int agent, MAComm* comm) {
		this->agent = agent;
		this->comm = comm;
	}
};
//----------------------------------------------------------

void *listenerThreadEntry(void *data) {
	// Guarantees that thread resources are deallocated upon return
	ListenerData* ldata = (ListenerData*) data;
	MAComm* comm = ldata->comm;
	int agent = ldata->agent;
	pthread_detach(pthread_self());
	comm->increaseThreadCounter();

	delete ldata;

	// Listen
	TCPSocket *sock = comm->m_Sockets[agent];
	cout << "Listening to agent ";
	try {
		cout << sock->getForeignAddress() << ":";
	} catch (SocketException &e) {
		cerr << "Unable to get foreign address" << endl;
	}
	try {
		cout << sock->getForeignPort();
	} catch (SocketException &e) {
		cerr << "Unable to get foreign port" << endl;
	}
	cout << endl; //<< " with thread " << pthread_self() << endl;

	Message* m = NULL;
	try {
		while ((m = MAProtocol::receiveMsg(sock)) != NULL && m->msgType != MSG_FIN)
			// NULL - TCP connection terminated
			// MSG_FIN - remote agent requests a connection termination
			comm->m_InQueue.push(m);
	} catch (SocketException &e) {
		cerr << e.what() << endl;
	}

	try {
		// Destructor closes socket
		comm->m_Sockets[agent] = NULL;
		delete sock;
	} catch (...) {
	}

	if (m == NULL)
		cout << "Connection to agent " << agent << " terminated" << endl;
	else
		cout << "Connection to agent " << agent << " terminated on FIN" << endl;

	comm->decreaseThreadCounter();
	return NULL;
}

void *senderThreadEntry(void *data) {
	MAComm* comm = (MAComm*) data;
	pthread_detach(pthread_self());
	comm->increaseThreadCounter();
	bool fin = false;
	do {
		Message* m = comm->m_OutQueue.pop();
		if (m->msgType == MSG_FIN)
			fin = true;
		else {
			if (comm->m_Sockets[m->dest_id]) {
				if (g_message_delay && !m->msgType == SOLUTION_CONFIRMATION) {
					usleep((rand() % 10000) * 10);
				}
				MAProtocol::sendMsg(comm->m_Sockets[m->dest_id], m);
			}
		}

		if (m)
			delete m;
	} while (!fin);
	comm->decreaseThreadCounter();
	return NULL;
}

void MAProtocol::sendMsg(TCPSocket* sock, Message* m) {
	char buff[RCVBUFSIZE + sizeof(int)];
	// put the size in the front of the message
	int size = m->serialize(buff + sizeof(int));
	memcpy(buff, &size, sizeof(int));

	//int size = *((int*)buff) = m->serialize(buff + sizeof(int));
	sock->send(buff, size + sizeof(int));
	g_num_of_messages_sent++;
}

Message* MAProtocol::receiveMsg(TCPSocket *sock) {
	int size;
	char* p = (char*) &size;
	int leftToRead = sizeof(int);
	// First field: size of message
	while (leftToRead) {
		int rcv = sock->recv(p, leftToRead);
		if (rcv == 0)
			return NULL;
		leftToRead -= rcv;
		p += rcv;
	}

	// Following: rest of the message
	// the size is filtered from the returned data
	char buff[RCVBUFSIZE];
	p = buff;
	leftToRead = size;
	while (leftToRead) {
		int rcv = sock->recv(p, leftToRead);
		if (rcv == 0)
			return NULL;
		leftToRead -= rcv;
		p += rcv;
	}
	g_num_of_messages_received++;
	return Message::deserialize(buff);
}

MAComm::MAComm(const char *configFileName, int agent_id) :
		m_InQueue("InQueue"), m_OutQueue("OutQueue") {
	m_ThreadCounter = 0;
	pthread_mutex_init(&m_Mutex, NULL);
	pthread_cond_init(&m_NoThreadsCond, NULL);

	m_outMsgId = 0;
	// read configuration from file
	setConfiguration(configFileName, agent_id);
	// Init sockets array
	m_Sockets = new TCPSocket*[m_Config.nAgents()];
	for (int i = 0; i < m_Config.nAgents(); ++i)
		m_Sockets[i] = NULL;
}

MAComm::~MAComm() {
	delete[] m_Sockets;
}

void MAComm::increaseThreadCounter() {
	pthread_mutex_lock(&m_Mutex);
	m_ThreadCounter++;
	pthread_mutex_unlock(&m_Mutex);
}

void MAComm::decreaseThreadCounter() {
	pthread_mutex_lock(&m_Mutex);
	if (--m_ThreadCounter == 0)
		pthread_cond_broadcast(&m_NoThreadsCond);
	pthread_mutex_unlock(&m_Mutex);
}

void MAComm::waitForThreadsTermination() {
	pthread_mutex_lock(&m_Mutex);
	while (m_ThreadCounter > 0)
		pthread_cond_wait(&m_NoThreadsCond, &m_Mutex);
	pthread_mutex_unlock(&m_Mutex);
}

void MAComm::connect() {
	// Wait for connections from all agents with IDs lower than this agent
	waitForConnections();
	// Initiate connections to all agents with IDs higher than this agent
	initiateConnections();

	// Create sender thread
	pthread_t threadID; // Thread ID from pthread_create()
	if (pthread_create(&threadID, NULL, senderThreadEntry, (void*) this) != 0) {
		cerr << "Unable to create thread" << endl;
		exit(1);
	}

	cout << "MAComm successfuly connected" << endl;
}

bool MAComm::waitForConnectionsState() {
	for (int agent = 0; agent < m_Config.thisId; ++agent)
		if (m_Sockets[agent] == NULL)
			return true;
	return false;
}

void MAComm::waitForConnections() {
	try {
		TCPServerSocket servSock(m_Config.thisPort()); // Socket descriptor for server

		while (waitForConnectionsState()) {
			cout << "Waiting for connection request... ";
			cout.flush();
			TCPSocket* client = servSock.accept();
			Message* m = MAProtocol::receiveMsg(client);
			int agent = m->sender_id;
			cout << "Accepted agent " << agent << endl;
			delete m;
			m_Sockets[agent] = client;

			// Create listener thread
			pthread_t threadID; // Thread ID from pthread_create()
			if (pthread_create(&threadID, NULL, listenerThreadEntry, (void *) new ListenerData(agent, this)) != 0) {
				cerr << "Unable to create thread" << endl;
				exit(1);
			}
		}
	} catch (SocketException &e) {
		cerr << e.what() << endl;
		exit(1);
	}
}

void MAComm::initiateConnections() {
	for (int agent = m_Config.thisId + 1; agent < m_Config.nAgents();) {
		try {
			string host = m_Config.servers[agent].host;
			unsigned short port = m_Config.servers[agent].port;
			//cout << "Connecting to agent " << agent << " " << host << ":"	<< port << "... " << endl;
			//cout.flush();
			// Establish connection with the remote agent server
			m_Sockets[agent] = new TCPSocket(host, port);

			// Send init message
			Message m;
			m.sender_id = m_Config.thisId;
			m.message_id = 0xFFFFFFFF;
			m.dest_id = agent;
			m.msgType = MSG_INIT;
			MAProtocol::sendMsg(m_Sockets[agent], &m);

			// Create listener thread
			pthread_t threadID; // Thread ID from pthread_create()
			if (pthread_create(&threadID, NULL, listenerThreadEntry, (void *) new ListenerData(agent, this)) != 0) {
				cerr << "Unable to create thread" << endl;
				exit(1);
			}
		} catch (SocketException &e) {
			if (m_Sockets[agent]) {
				delete m_Sockets[agent];
				m_Sockets[agent] = NULL;
			}
			//cerr << e.what() << endl;
			//exit(1);
			continue;
		}
		++agent;
	}
}

void MAComm::setConfiguration(const char *configFileName, int agent_id) {
	string line;
	cout << "Opening communication file: " << configFileName << endl;
	ifstream myfile(configFileName);
	if (myfile.is_open()) {
		m_Config.thisId = agent_id;
		cout << "configuring agent " << m_Config.thisId << endl;
		while (myfile.good()) {
			getline(myfile, line);

			int pos = line.find(":");
			if (pos == -1) {
//				pos = line.find("="); //trying to find delta
//				if (pos == -1) {
//					m_Config.thisId = atoi(line.c_str());
//					cout << "This agent's id: " << m_Config.thisId << endl;
//					break; //Added this
//				} else {
//					g_public_actions_cost_delta = atoi(line.substr(pos + 1,
//							line.length() - pos - 1).c_str());
//					cout << "cost to add to public actions of other agent: " << g_public_actions_cost_delta << endl;
//				}
			} else {
				IPAddress addr;
				addr.host = line.substr(0, pos);
				addr.port = atoi(line.substr(pos + 1, line.length() - pos - 1).c_str());
				m_Config.servers.push_back(addr);
				cout << "Adding agent " << m_Config.servers.size() - 1 << " with address " << addr.host << ":" << addr.port << endl;
			}
		}

		myfile.close();
	} else
		cout << "Unable to open file" << endl;
}

Message* MAComm::receiveMessage() {
	return m_InQueue.pop();
}

void MAComm::sendMessage(Message* m) {
	m->sender_id = m_Config.thisId;
	m->message_id = m_outMsgId++;
	m_OutQueue.push(m);
}

void MAComm::disconnect() {
	sleep(10); //TODO - (Raz) added this so we would not lose the FIN message. Don't know why it happened.
	Message* m = new Message();
	m->msgType = MSG_FIN;
	m->message_id = 0xFFFFFFFF;

	// Notify remote agents of FIN
	for (int agent = 0; agent < m_Config.nAgents(); ++agent) {
		if (agent == m_Config.thisId)
			continue;

		try {
			string host = m_Config.servers[agent].host;
			unsigned short port = m_Config.servers[agent].port;
			cout << "Sending FIN to agent " << agent << " " << host << ":" << port << "... " << endl;
			m->dest_id = agent;
			if (m_Sockets[agent])
				MAProtocol::sendMsg(m_Sockets[agent], m);
		} catch (SocketException &e) {
			cerr << e.what() << endl;
			continue;
		}
	}

	// Notify sender thread of termination
	sendMessage(m);

	// Wait for all threads to terminate
	waitForThreadsTermination();

	sleep(2);

	cout << "MAComm successfuly disconnected" << endl;
}

