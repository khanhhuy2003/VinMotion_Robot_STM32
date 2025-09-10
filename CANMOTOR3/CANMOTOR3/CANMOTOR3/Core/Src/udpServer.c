/*
 * udpServer.c
 *
 *  Created on: Apr 3, 2025
 *      Author: letie
 */


#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "udpSever.h"
#include <string.h>
#include <stdbool.h>

struct pbuf *udpRxBuf = NULL;
struct pbuf *udpTxBuf = NULL;
struct udp_pcb *udpServerPcb = NULL;
ip_addr_t udpClientAddr;
u16_t udpClientPort;

uint32_t last_time_rx_udp = 0;
char RxUDP[300];
//extern LowCmdPacket lowCmdPacket;

extern bool g_receive_udp;
int udp_count = 0;

int rc=0;
/* IMPLEMENTATION FOR UDP Server :   source:https://www.geeksforgeeks.org/udp-server-client-implementation-c/

 1. Create UDP socket.
 2. Bind the socket to server address.
 3. Wait until datagram packet arrives from client.
 4. Process the datagram packet and send a reply to client.
 5. Go back to Step 3.
 */

void udpServer_init(void) {
	// UDP Control Block structure
	//struct udp_pcb *upcb;
	err_t err;

	/* 1. Create a new UDP control block  */
	udpServerPcb = udp_new();

	/* 2. Bind the upcb to the local port */
	ip_addr_t myIPADDR;
	IP_ADDR4(&myIPADDR, 192, 168, 1, 10);
	err = udp_bind(udpServerPcb, &myIPADDR, 8888);  // 8888 is the server UDP port

	ip_addr_t ClientIPADDR;
	IP_ADDR4(&ClientIPADDR, 192, 168, 1, 20);
	ip_addr_set(&udpClientAddr, &ClientIPADDR);
	udpClientPort = 12346;

	/* 3. Set a receive callback for the upcb */
	if (err == ERR_OK) {
		udp_recv(udpServerPcb, udp_receive_callback, NULL);
	} else {
		udp_remove(udpServerPcb);
	}
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port) {

	if (p != NULL) {
		udp_count++;
		memcpy(RxUDP, p->payload, p->len);
		RxUDP[p->len] = '\0';
		pbuf_free(p);
        struct pbuf *tx_buf;
        const char *reply_msg = "ACK from STM32";

        tx_buf = pbuf_alloc(PBUF_TRANSPORT, strlen(reply_msg), PBUF_RAM);
        if (tx_buf != NULL) {
            pbuf_take(tx_buf, reply_msg, strlen(reply_msg));
            udp_sendto(upcb, tx_buf, addr, port); // gửi lại đúng địa chỉ và port của client
            pbuf_free(tx_buf);
        }

        pbuf_free(p);


//		last_time_rx_udp = HAL_GetTick();
//
//		g_receive_udp = true;
//		rc++;
		//memcpy(&lowCmdPacket, RxUDP, sizeof(LowCmdPacket));
	}

}

void udp_send_buffer(const u8_t *data, u16_t len) {
	if (udpServerPcb == NULL || data == NULL || len == 0)
		return;

	if (udpTxBuf != NULL) {
		pbuf_free(udpTxBuf);
		udpTxBuf = NULL;
	}

	udpTxBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	if (udpTxBuf == NULL)
		return;

	pbuf_take(udpTxBuf, data, len);

	ip_addr_t ClientIPADDR;
	IP_ADDR4(&ClientIPADDR, 192, 168, 1, 20);  //IP của máy tính

	ip_addr_set(&udpClientAddr, &ClientIPADDR);
	udpClientPort = 12346;
	udpServerPcb = udpServerPcb;

	udp_connect(udpServerPcb, &udpClientAddr, udpClientPort);
	udp_send(udpServerPcb, udpTxBuf);
	udp_disconnect(udpServerPcb);

	pbuf_free(udpTxBuf);
	udpTxBuf = NULL;
}

