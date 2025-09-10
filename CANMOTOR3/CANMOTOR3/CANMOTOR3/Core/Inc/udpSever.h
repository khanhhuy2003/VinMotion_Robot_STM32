/*
 * udpSever.h
 *
 *  Created on: Apr 3, 2025
 *      Author: letie
 */

#ifndef INC_UDPSERVER_H_
#define INC_UDPSERVER_H_

void udpServer_init(void);
void udp_send_buffer(const u8_t *data, u16_t len);
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port);

#endif /* INC_UDPSERVER_H_ */
