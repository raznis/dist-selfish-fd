/*
 * ma_comm.h
 *
 *  Created on: Jul 24, 2012
 *      Author: raznis
 */

#ifndef MA_COMM_H_
#define MA_COMM_H_
#include "blocking_message_queue.h"
#include <string>
#include <vector>
#include <pthread.h>
using namespace std;

class TCPSocket;
struct Message;

struct IPAddress
{
	string host;
	unsigned short port;
};

struct MAConfiguration
{
	vector<IPAddress> servers;
	int thisId;
	int nAgents() {return servers.size();}
	unsigned short thisPort() {	return servers[thisId].port; }
};

class MAProtocol
{
public:
	static void sendMsg(TCPSocket* sock, Message* m);
	static Message* receiveMsg(TCPSocket *sock);
};

class MAComm
{
	friend void* senderThreadEntry (void*);
	friend void* listenerThreadEntry (void*);
private:
	void setConfiguration(const char* configFileName, int agent_id);
	void waitForConnections();
	void initiateConnections();
	bool waitForConnectionsState();
	void increaseThreadCounter();
	void decreaseThreadCounter();
	void waitForThreadsTermination();
	MAConfiguration m_Config;
	TCPSocket** m_Sockets;
	BlockingMessageQueue<Message*> m_InQueue;
	BlockingMessageQueue<Message*> m_OutQueue;

	int m_outMsgId;
	int m_ThreadCounter;
	pthread_mutex_t m_Mutex;
	pthread_cond_t m_NoThreadsCond;

public:
	MAComm(const char* configFileName, int agent_id);
	~MAComm();
	MAConfiguration getConfigCopy() {return m_Config;}

	void connect();
	void disconnect();

	void sendMessage(Message* m);  // Allocate message using "new" before sending
	Message* receiveMessage();   // delete Message using "delete" after use
	bool noIncomingMessage() {return m_InQueue.empty();}
	int get_current_message_id() {return m_outMsgId;}
};

#endif /* MA_COMM_H_ */
