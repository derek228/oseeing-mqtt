#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <net/if.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include "ini-parse.h"
#include "../leds/pwm_message.h"
#include "led_control.h"
#include "ethernet.h"
#include <arpa/inet.h>

char device_macno[18]={0};
char fake_macno[18]={0};

int check_ip() {
    int sockfd;
    struct ifreq ifr;
    struct sockaddr_in *sin;
    char ipaddr[16]={0};
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation error");
        return 0;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    strncpy(ifr.ifr_name, ETHERNET_DEVICE_NAME, IFNAMSIZ - 1);

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) < 0) {
        perror("IOCTL error");
        close(sockfd);
        return 0;
    }

    sin = (struct sockaddr_in *)&ifr.ifr_addr;
    sprintf(ipaddr,"%s\n",inet_ntoa(sin->sin_addr));
    printf("IP Address: %s\n", ipaddr);// inet_ntoa(sin->sin_addr));

    close(sockfd);

    return 1;
}

int is_connected(char *url) {
    // 要ping的主机名或IP地址
    char *ptr_start, *ptr_end;
    char dns[50];
    char command[70];
    // 在字符串中查找 "http://" 的起始位置
    ptr_start = strstr(url, "http://");
    if (ptr_start != NULL) {
        // 在 "http://" 之后查找下一个斜杠 "/" 的位置
        ptr_start = ptr_start + strlen("http://");
        ptr_end = strchr(ptr_start, '/');
        if (ptr_end != NULL) {
            // 将找到的部分复制到提取的 URL 中
            strncpy(dns, ptr_start, ptr_end - ptr_start);
            dns[ptr_end - ptr_start] = '\0'; // 添加字符串结束符
            //printf("Extracted URL: %s\n", dns);
        }
    }

    // 构造ping命令
    sprintf(command, "ping -c 1 %s > /dev/null 2>&1", dns);
    //printf("command = %s\n", command);
    // 执行ping命令并获取输出
    if (system(command) == 0) {
        //printf("Ping successful!\n");
        return 0;
    } else {
        //printf("Ping failed!\n");
        return -1;
    }

    return 0;
}

void eth_mac_config() {
    int fd;
    struct ifreq ifr;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }

    strncpy(ifr.ifr_name, ETHERNET_DEVICE_NAME, IFNAMSIZ);

    if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
        perror("ioctl");
        exit(1);
    }

    sprintf(device_macno,"%02x:%02x:%02x:%02x:%02x:%02x",
            (unsigned char)ifr.ifr_hwaddr.sa_data[0],
            (unsigned char)ifr.ifr_hwaddr.sa_data[1],
            (unsigned char)ifr.ifr_hwaddr.sa_data[2],
            (unsigned char)ifr.ifr_hwaddr.sa_data[3],
            (unsigned char)ifr.ifr_hwaddr.sa_data[4],
            (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

    close(fd);
	printf("Onboard MAC Address : %s\n", device_macno);
    
}
void eth_set_fake_mac(char *no) {
	strncpy(fake_macno, no, 18);
	fake_macno[17]='\0';
}
char *eth_get_mac() {
	if (fake_macno[0]!=0) return fake_macno;
	else return device_macno;
}
int ethernet_init() {
	if (check_ip()==1) {
		printf("ETH connetced\n");
		return 0;
	}
		printf("Start DHCP \n");
	if (system("udhcpc -n")) {
		return -1;
	}
	else 
		return 0;
}
