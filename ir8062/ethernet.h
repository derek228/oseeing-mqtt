#ifndef __ETHERNET_H__
#define __ETHERNET_H__
int ethernet_init();
char *eth_get_mac();
void eth_mac_config();
void eth_set_fake_mac(char *no);
int is_connected(char *dns);
#endif
