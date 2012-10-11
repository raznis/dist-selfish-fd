/*
 * blocking_message_queue.h
 *
 *  Created on: Jul 24, 2012
 *      Author: raznis
 */

#ifndef BLOCKING_MESSAGE_QUEUE_H_
#define BLOCKING_MESSAGE_QUEUE_H_
#include <queue>
#include <pthread.h>
#include <iostream>

using namespace std;

template <class MESSAGE>
class BlockingMessageQueue
{
private:
	int m_Transaction;
	queue<MESSAGE> m_Queue;
	pthread_mutex_t m_Mutex;
	pthread_cond_t m_NotFull, m_NotEmpty;
	int m_MaxMessages;
	const char* m_Name;
public:
	BlockingMessageQueue(const char* queueName, int maxMessages=10000);//TODO- changed from 1000
	void push(MESSAGE m); 	// blocking
	MESSAGE pop();			// blocking
	bool empty();
};

template <class MESSAGE>
BlockingMessageQueue<MESSAGE>::BlockingMessageQueue(const char* queueName, int maxMessages)
{
	m_MaxMessages = maxMessages;
	pthread_mutex_init(&m_Mutex,NULL);
	pthread_cond_init(&m_NotFull,NULL);
	pthread_cond_init(&m_NotEmpty,NULL);
	//m_NotFull = PTHREAD_COND_INITIALIZER;
	//m_NotEmpty = PTHREAD_COND_INITIALIZER;
	m_Name = queueName;
	m_Transaction = 0;
}

template <class MESSAGE>
void BlockingMessageQueue<MESSAGE>::push(MESSAGE m)
{
	pthread_mutex_lock(&m_Mutex);
	//int trans = m_Transaction++;		//TODO (ask udi if trans is needed somewhere)
	m_Transaction++;
	//cout << m_Name << " " << trans <<": begin push message" << endl;
	while (m_Queue.size() == m_MaxMessages)
		pthread_cond_wait(&m_NotFull, &m_Mutex);
	bool wasEmpty = m_Queue.empty();
	m_Queue.push(m);
	if (wasEmpty)
	{
		//cout << "broadcast" << endl;
		pthread_cond_broadcast(&m_NotEmpty);
	}
	pthread_mutex_unlock(&m_Mutex);
	//cout << m_Name << " " << trans <<": end push message" << endl;
}

template <class MESSAGE>
MESSAGE BlockingMessageQueue<MESSAGE>::pop()
{
	pthread_mutex_lock(&m_Mutex);
	//int trans = m_Transaction++;	//TODO (ask udi if trans is needed somewhere)
	m_Transaction++;
	//cout << m_Name << " " << trans <<": begin pop message" << endl;
	while (m_Queue.empty())
		pthread_cond_wait(&m_NotEmpty, &m_Mutex);
	bool wasFull = (m_Queue.size() == m_MaxMessages);
	MESSAGE m = m_Queue.front();
	m_Queue.pop();
	if (wasFull)
		pthread_cond_broadcast(&m_NotFull);
	pthread_mutex_unlock(&m_Mutex);
	//cout << m_Name << " " << trans <<": end pop message" << endl;
	return m;
}

template <class MESSAGE>
bool BlockingMessageQueue<MESSAGE>::empty()
{
	return m_Queue.empty();
}


#endif /* BLOCKING_MESSAGE_QUEUE_H_ */
