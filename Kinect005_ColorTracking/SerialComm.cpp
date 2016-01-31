#include "serialcomm.h"
#include <iostream>

CSerialComm::CSerialComm() {}
CSerialComm::~CSerialComm() {}

int CSerialComm::connect(char* portNum, int baudRate)
{
	if (!serial.OpenPort(portNum)) return 0;
	serial.ConfigurePort(baudRate, 8, FALSE, NOPARITY, ONESTOPBIT);
	serial.SetCommunicationTimeouts(0, 0, 0, 0, 0);

	serial.SetCommunicationTimeouts(10,10,10,10,10);

	return 1;
}

string CSerialComm::readUnit()
{
	int okToGo = 0;
	string str = "";
	char c=' ';
	while (c != ']')
	{
		c = readChar(&okToGo);
		//cout << "ok? " << okToGo << endl;
		if (!okToGo) return str;
		str = str + c;
		
	}
	return str;
}

void CSerialComm::flush()
{
	BYTE c = '\0';
	int result = 1;
	while (result)
	{
		result = serial.ReadByte(c);
	}
}

char CSerialComm::readChar(int *a)
{
	BYTE c = '\0';
	*a = serial.ReadByte(c);
	return c;
}


int CSerialComm::sendString(string str)
{
	char *a = new char[str.size() + 1];
	a[str.size()] = 0;
	memcpy(a, str.c_str(), str.size());
	int result = 0;
	for (int i = 0; i < str.size(); i++)
	{
		result = sendCommand(a[i]);
		if (result == 0) return 0;
		//std::cout << a[i] << endl;
	}

	return 1;
}


int CSerialComm::sendCommand(unsigned char pos)
{
	if (serial.WriteByte(pos))
		return 1;
	else
		return 0;
}
void CSerialComm::disconnect()
{
	serial.ClosePort();
}
