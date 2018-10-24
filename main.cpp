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
    uint32_t serial_recv_count = 0;
    uint8_t sec_event = 0;

    int serial_index = 0;
    int serial_length = 0;
    int buf_length = 0;

    int rdcnt = 0;

    uint8_t state_flag = 0;

    uint8_t mav_ver = 2;

    uint8_t buf[MAXBUF];
    uint8_t soc_buf[MAXBUF];

    char receive_buffer[15000];

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
                uint8_t serial_buf[1024];

                // Get data from Serial
                rdcnt = read(_serial_fd, serial_buf, 1024);

                // Setup UDP Socket
                gcAddr.sin_family = AF_INET;
                gcAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
                gcAddr.sin_port = htons(14550);

                // Send data to QGC through UDP
                int sendUdpLen = sendto(_socket_fd, serial_buf, rdcnt, 0, (struct sockaddr *)&gcAddr, sizeof(struct sockaddr_in));

                int i = 0;
                for(i=0;i<rdcnt;i++){
                  printf("%02x ", serial_buf[i]);
                }
                printf("\n");
                //printf("GCS <<< Mobius\n");
              }
            else if (fds[1].revents & POLLIN) {      // by UDP Socket
              memset(soc_buf, 0, MAXBUF);

              // Get data from UDP(QGC)
              int recvLen = recvfrom(_socket_fd, (void *)soc_buf, MAXBUF, 0, (struct sockaddr *)&gcAddr, &fromLen);

              // Pass data to Serial
              int sentLen = write(_serial_fd, soc_buf, recvLen);

                printf("GCS >>> Mobius\n");
            }
        }
        else {
            printf(">>> send \n");
            sendHeartbeat();
        }
    }
}
