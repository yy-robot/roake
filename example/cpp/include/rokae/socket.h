#pragma once

class MySocket {

private:
	sockaddr_in addr_in;
	int clientfd;
public:
	MySocket(const char* addr, int post);
	bool Socket();
	bool Connect();
	int Send(const char* str);
	int Receive(char* str);
};