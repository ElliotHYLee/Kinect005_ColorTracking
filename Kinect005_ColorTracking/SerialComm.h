#include "serialport.h"
#include <string>
using namespace std;

class CSerialComm
{
public:
	CSerialComm();
	~CSerialComm();
	CSerialPort  serial;
	int    connect(char* _portNum, int baudRate);
	int    sendCommand(unsigned char pos);
	int    sendString(string str);
	string readUnit();
	char   readChar(int *a);

	void   flush();


	void   disconnect();
};
