#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <common/mavlink.h>

#define MAXBUF		    (1024)

int _serial_fd = 0;
int _socket_fd = 0;

int setupUDP(void)
{
    int sock;
    int status, buflen, sinlen;
    char buffer[MAXBUF];
    int yes = 1;
    struct sockaddr_in sock_in;

    sinlen = sizeof(struct sockaddr_in);
    memset(&sock_in, 0, sinlen);
    buflen = MAXBUF;

    sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if ( sock < 0 ){
        printf("ERROR: UDP connection fail\n");
        return -1;
    }

    sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    sock_in.sin_port = htons(14551);
    sock_in.sin_family = PF_INET;

    status = bind(sock, (struct sockaddr *)&sock_in, sinlen);
    printf("Bind Status = %d\n", status);


    if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
        printf("ERROR: nonblock error\n");
        return -2;
    }

    return sock;
}

int setupSerial()
{
    char *uart_name = (char*)"/dev/ttyUSB0";
    int speed = B57600; //B1000000; //B57600; // B230400; //B57600; 
    int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY  /*O_NONBLOCK*/);
    if (uart < 0) {
	printf("FAIL: Error opening port");
	return -1;
    }

    struct termios uart_config;
    tcgetattr(uart, &uart_config);

    uart_config.c_cflag |= CLOCAL | CREAD | CS8 ;
    uart_config.c_iflag  = IGNPAR | ICRNL;
    uart_config.c_oflag = 0;
    uart_config.c_lflag = 0;

    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0 ) {
	printf("FAIL: Error setting baudrate / termios config for cfsetispeed ");
	return -1;
    }

    tcflush (uart, TCIFLUSH);

    if (tcsetattr(uart, TCSANOW, &uart_config) < 0) {
	printf("FAIL: Error setting baudrate / termios config for tcsetattr");
	return -1;
    }

    return uart;

}

int sendHeartbeat() 
{
    uint8_t buf[MAXBUF];
    uint16_t len;
    mavlink_message_t msg;

    struct sockaddr_in gcAddr;
    memset(&gcAddr, 0, sizeof(gcAddr));
    memset(buf, 0, MAXBUF);

    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    gcAddr.sin_port = htons(14550);

    // send heartbeat
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    int sentLen = sendto(_socket_fd, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

int main(void){
    uint8_t buf[MAXBUF];
   
    /* udp socket setup */
    _socket_fd = setupUDP();

    /* serial setup */
    _serial_fd = setupSerial();
    
    sendHeartbeat();

    /* poll descriptor */
    struct pollfd fds[2];
    fds[0].fd = _serial_fd;
    fds[0].events = POLLIN;
    fds[1].fd = _socket_fd;
    fds[1].events = POLLIN;

    /* Set Sock Structure */
    struct sockaddr_in gcAddr;
    unsigned int fromLen = sizeof(gcAddr);

    /* main thread loop */
    while (1) {
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);
        if ( status > 0 ) {

            if (fds[0].revents & POLLIN) {      // by Serial
                printf(">>>> recv from serial \n");

                // receive data from serial
                int recvLen = read(_serial_fd, buf, MAXBUF);
                if ( recvLen < 0 ) {
                    printf("ERROR : cannot read data\n");
                    continue;
                }
                
                // parse data for mavlink binary
                char strbuf[MAXBUF] = {};
                int  pos_strbuf = 0;
                char str[3] = {};
                int  ps = 0;        // position of string
                for(int i = 0 ; i < recvLen; i++ ) {

                    if ( buf[i] == 0x20 ) {

                        int number = (int)strtol(str, NULL, 16);
                        strbuf[pos_strbuf++] = (char)number;

                        // init string data
                        memset(str, 0, sizeof(str));
                        ps = 0;
                    }
                    else if ( buf[i] == 0x0D || buf[i] == 0x0A ) {
                        // ignore 
                    }
                    else {
                        // append data
                        str[ps++] = buf[i];
                    }
                }

                // Send MAVdata to QGroundControl UDP Socket
                gcAddr.sin_family = AF_INET;
                gcAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
                gcAddr.sin_port = htons(14550);
                int sendUdpLen = sendto(_socket_fd, strbuf, pos_strbuf, 0, (struct sockaddr *)&gcAddr, sizeof(struct sockaddr_in));					

            }
            else if (fds[1].revents & POLLIN) {      // by UDP Socket
                memset(buf, 0, MAXBUF);
                printf(">>> Send Mav Data to Serial\n");
                int recvLen = recvfrom(_socket_fd, (void *)buf, MAXBUF, 0, (struct sockaddr *)&gcAddr, &fromLen);

                // pass to serial
                int sentLen = write(_serial_fd, buf, recvLen);
                write(_serial_fd, "\r\n", 2);

            }

        }
        else {
            printf(">>> send \n");
            sendHeartbeat();
        }
    }

}
