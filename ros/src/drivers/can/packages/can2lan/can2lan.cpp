#include <cstring>
#include <iostream>
#include <string>
#include <list>
#include <sstream>
#include <vector>
#include <thread>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

struct CommandData {
  double linear_x;//线性
  double angular_z;//角度
  int modeValue;//模式
  int gearValue;//档位
  int lampValue;//灯
  int accellValue;//轴加速
  int brakeValue;//刹车
  int steerValue;//方向盘
  double linear_velocity;//线速度
  double steering_angle;//转向角
};

struct ReceiveData {
  char   tm[24];
  double speed;
  double angle;
  int torque;
  int drivepedal;
  int brakepedal;
  int driveshift;
};

typedef struct {
  int8_t 	len;
  uint32_t 	canid;
  int32_t 	datalr;
  int32_t 	datahr;
}CAN_TypeDef;

#define TAKE_OVER_PATTERN 0x01

#define CAN_KEY_MODE (0)
#define CAN_KEY_TIME (1)
#define CAN_KEY_VELOC (2)
#define CAN_KEY_ANGLE (3)
#define CAN_KEY_TORQUE (4)
#define CAN_KEY_ACCEL (5)
#define CAN_KEY_BRAKE (6)
#define CAN_KEY_SHIFT (7)
#define CAN_KEY_X (8)
#define CAN_KEY_Y (9)

static CommandData command_data;
static ReceiveData receive_data;

static bool parseCanValue(const std::string &can_data)
{
	std::istringstream ss(can_data);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ','))
	{
		columns.push_back(column);
	}
	
	for (std::size_t i = 0; i < columns.size(); i += 1)
	{
		switch (i)
		{
			case CAN_KEY_MODE:
				command_data.linear_x = std::stod(columns[i]);
				 break;
			case CAN_KEY_TIME:
				command_data.angular_z = std::stod(columns[i]);
				 break;
			case CAN_KEY_VELOC:
				command_data.modeValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_ANGLE:
				command_data.gearValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_TORQUE:
				command_data.lampValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_ACCEL:
				command_data.accellValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_BRAKE:
				command_data.brakeValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_SHIFT:
				command_data.steerValue = std::stoi(columns[i]);
				 break;
			case CAN_KEY_X:
				command_data.linear_velocity = std::stod(columns[i]);
				 break;
			case CAN_KEY_Y:
				command_data.steering_angle = std::stod(columns[i]);
				 break;
			default :
				 break;
		}
	}
	
	return true;
}

static bool sendCanValue(const int sockfd)
{
	CAN_TypeDef can_data;
	//方向盘
	can_data.len = 5;
	can_data.canid = htonl (0x130);
	can_data.datahr = (TAKE_OVER_PATTERN);
	can_data.datalr = htons(command_data.angular_z) + htonl(command_data.steering_angle);
	send(sockfd, &can_data, sizeof(can_data),0); 
	
	//刹车
	can_data.len = 5;
	can_data.canid = htonl (0x132);
	can_data.datalr = (htonl(0x0101) + htons(command_data.brakeValue));
	can_data.datahr = TAKE_OVER_PATTERN;
	send(sockfd, &can_data, sizeof(can_data),0);
	
	//油门
	can_data.len = 4;
	can_data.canid = htonl (0x134);
	can_data.datalr = htonl(0x0100+TAKE_OVER_PATTERN) + htons(command_data.linear_x);
	send(sockfd, &can_data, sizeof(can_data),0);
	
	//档位
	can_data.len = 2;
	can_data.canid = htonl (0x136);
	can_data.datalr = htons(0x01) + command_data.modeValue;
	send(sockfd, &can_data, sizeof(can_data),0);
	
	//灯
	can_data.len = 2;
	can_data.canid = htonl (0x138);
	can_data.datalr = htons(command_data.lampValue);
	send(sockfd, &can_data, sizeof(can_data),0);
	
	return true;
}

int canSend(int sockfd)
{
	int req_sock=0;
	
	struct sockaddr_in sock_in;
	sock_in.sin_family=AF_INET;
	sock_in.sin_port=  htons(10001);
	sock_in.sin_addr.s_addr=inet_addr("127.0.0.1");
	socklen_t len=sizeof(sock_in);
		
	while(1)
	{
		req_sock=socket(AF_INET,SOCK_STREAM,0);
		connect(req_sock,(struct sockaddr*)&sock_in,len);
		char buf[1024];
		memset(buf,0,1024);
		ssize_t s=read(req_sock,buf,sizeof(buf));

		std::string reply = buf;
		parseCanValue(reply);
		sendCanValue(sockfd);

		close(req_sock);
	}

}

int canTranx(int *sockfd)
{
    int req_sock=0;
	
	struct sockaddr_in sock_in;
	sock_in.sin_family=AF_INET;
	sock_in.sin_port=  htons(5000);
	sock_in.sin_addr.s_addr=inet_addr("192.168.1.5");
	socklen_t len=sizeof(sock_in);
	
	req_sock=socket(AF_INET,SOCK_STREAM,0);
	connect(req_sock,(struct sockaddr*)&sock_in,len);
	
	*sockfd = req_sock;
}

// struct ReceiveData {
  // char * tm;
  // double speed;
  // double angle;
  // int torque;
  // int drivepedal;
  // int brakepedal;
  // int driveshift;
// };
void parsing_vehicle_data(CAN_TypeDef can_data)
{
	uint8_t   	len = can_data.len; 
	uint32_t 	canid = ntohl(can_data.canid);
	int32_t 	datalr = ntohl(can_data.datalr);
	int32_t 	datahr = ntohl(can_data.datahr);
	
	switch(canid){
		case 0x131:receive_data.angle = datalr >> 16;
		break;
		case 0x133:receive_data.brakepedal = datalr >> 16;
		break;
		case 0x135:receive_data.drivepedal = datalr >> 16;
		break;
		case 0x139:receive_data.speed = (short)ntohs(((datalr)));
		break;
		default:
		break;
	}
}

static void ReceiceCommand(int sockfd)
{
	std::ostringstream oss;
	oss << 0x6 << "," << receive_data.brakepedal << ",";
	oss << 0x0 << "," << receive_data.tm << ",";
	oss << 0x1 << "," << receive_data.speed << ",";
	oss << 0x2 << "," << receive_data.angle << ",";
	oss << 0x3 << "," << receive_data.torque << ",";
	oss << 0x4 << "," << receive_data.drivepedal << ",";
	oss << 0x5 << "," << receive_data.driveshift << ",";
	oss << 0x7 << "," << receive_data.brakepedal << ",";
	std::string cmd(oss.str());
	
	send(sockfd,cmd.c_str(), cmd.size(),0);
	
}

int canReceive(int sockfd)
{
	CAN_TypeDef can_data;
	
	int req_sock=0;
	struct sockaddr_in sock_in;
	sock_in.sin_family=AF_INET;
	sock_in.sin_port=  htons(10000);
	sock_in.sin_addr.s_addr=inet_addr("127.0.0.1");
	socklen_t len=sizeof(sock_in);
	
	while(1)
	{
		req_sock=socket(AF_INET,SOCK_STREAM,0);
		connect(req_sock,(struct sockaddr*)&sock_in,len);
	
		memset(&can_data,0,sizeof(can_data));
		recv(sockfd, &can_data, sizeof(can_data),0);
		parsing_vehicle_data(can_data);
		
		ReceiceCommand(req_sock);

		close(req_sock);
	}
}

int main(int argc, char *argv[])
{
	int socket = 0;
	
	std::thread cantranx(canTranx, &socket);
	std::thread cansend(canSend, socket);
	std::thread canreceive(canReceive, socket);
	
	cansend.join();
	cantranx.join();
	canreceive.join();
	
	return 0;
}