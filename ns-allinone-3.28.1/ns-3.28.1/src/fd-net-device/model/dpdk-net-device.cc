/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "dpdk-net-device.h"
#include "ns3/log.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/simulator.h"
#include "ns3/system-thread.h"
#include "ns3/system-condition.h"
#include "ns3/system-mutex.h"

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/signal.h>
#include <unistd.h>

#include <poll.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_common.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_cycles.h>

#define MAX_PKT_BURST 32 //define the maximum packet burst size
#define MEMPOOL_CACHE_SIZE 256 //define the cache size for the memory pool

// Configurable number of RX/TX ring descriptors
#define RTE_TEST_RX_DESC_DEFAULT 1024
#define RTE_TEST_TX_DESC_DEFAULT 1024
static uint16_t nb_rxd = RTE_TEST_RX_DESC_DEFAULT;
static uint16_t nb_txd = RTE_TEST_TX_DESC_DEFAULT;

struct rte_mempool *l2fwd_pktmbuf_pool = NULL;

static struct rte_eth_dev_tx_buffer *tx_buffer[RTE_MAX_ETHPORTS];

static struct rte_eth_conf port_conf = {};

static volatile bool force_quit;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DPDKNetDevice");

NS_OBJECT_ENSURE_REGISTERED (DPDKNetDevice);

TypeId
DPDKNetDevice::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DPDKNetDevice")
    .SetParent<FdNetDevice> ()
    .SetGroupName ("FdNetDevice")
    .AddConstructor<DPDKNetDevice> ()
  ;
  return tid;
}

DPDKNetDevice::DPDKNetDevice ()
{
  NS_LOG_FUNCTION (this);
}

void
DPDKNetDevice::CheckAllPortsLinkStatus(void)
{
#define CHECK_INTERVAL 100 /* 100ms */
#define MAX_CHECK_TIME 90 /* 9s (90 * 100ms) in total */
	uint8_t count, all_ports_up, print_flag = 1;
	struct rte_eth_link link;

	printf("\nChecking link status\n");
	fflush(stdout);
	for (count = 0; count <= MAX_CHECK_TIME; count++) {

		all_ports_up = 1;
		
    if (force_quit)
			return;
		if ((1 << m_portId) == 0)
			continue;
		memset(&link, 0, sizeof(link));
		rte_eth_link_get(m_portId, &link);
		/* print link status if flag set */
		if (print_flag == 1) {
			if (link.link_status)
				printf(
				"Port%d Link Up. Speed %u Mbps - %s\n",
					m_portId, link.link_speed,
			(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
				("full-duplex") : ("half-duplex\n"));
			else
				printf("Port %d Link Down\n", m_portId);
			continue;
		}
		/* clear all_ports_up flag if any link down */
		if (link.link_status == ETH_LINK_DOWN) {
			all_ports_up = 0;
			break;
		}
		
    /* after finally printing all link status, get out */
		if (print_flag == 1)
			break;

		if (all_ports_up == 0) {
			printf(".");
			fflush(stdout);
			rte_delay_ms(CHECK_INTERVAL);
		}

		/* set the print_flag if all ports up or timeout */
		if (all_ports_up == 1 || count == (MAX_CHECK_TIME - 1)) {
			print_flag = 1;
			printf("done\n");
		}
	}
}

void
DPDKNetDevice::SignalHandler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\nSignal %d received, preparing to exit...\n",
				signum);
		force_quit = true;
	}
}

void
DPDKNetDevice::InitDPDK (int argc, char** argv)
{

  // Initialize DPDK EAL
  int ret = rte_eal_init(argc, argv);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
    }

  force_quit = false;
  signal(SIGINT, SignalHandler);
	signal(SIGTERM, SignalHandler);

  // Bind device to DPDK
  std::string command;
  printf("Binding %s to driver uio_pci_generic\n", command.c_str());
  command.append("$RTE_SDK/usertools/dpdk-devbind.py --force ");
  command.append("--bind=uio_pci_generic ");
  command.append(m_deviceName.c_str());
  printf("Executing %s\n", command.c_str());
  if (system(command.c_str()))
    {
      rte_exit(EXIT_FAILURE, "Execution failed - bye\n");
    }

  // wait for the device to bind to DPDK
  sleep (5);  /* 5 second */

  unsigned nb_ports = rte_eth_dev_count();
	if (nb_ports == 0)
    {
      rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");
    }

  // Get port id of the device
  if (rte_eth_dev_get_port_by_name (m_deviceName.c_str(), &m_portId) != 0)
    {
      printf("Cannot get port id for %s\n", m_deviceName.c_str());
      rte_exit(EXIT_FAILURE, "Cannot get port id - bye\n");
    }
  printf("Port id for %s is %d\n", m_deviceName.c_str(), m_portId);

  // Set number of logical cores to 1
  unsigned int nb_lcores = 1;

  unsigned int nb_mbufs = RTE_MAX(nb_ports * (nb_rxd + nb_txd + MAX_PKT_BURST +
		nb_lcores * MEMPOOL_CACHE_SIZE), 8192U);

  printf("nb-mbufs------------%d\n",nb_mbufs);

	/* create the mbuf pool */
	l2fwd_pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", nb_mbufs,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());

	if (l2fwd_pktmbuf_pool == NULL)
    {
      rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");
    }
  
  printf("mbuf pool initialization successful\n");

  // Initialize port
	port_conf.rxmode = {};
	port_conf.rxmode.split_hdr_size = 0;
	port_conf.rxmode.ignore_offload_bitfield = 1;
	port_conf.rxmode.offloads = DEV_RX_OFFLOAD_CRC_STRIP;
	port_conf.txmode = {};
  port_conf.txmode.mq_mode = ETH_MQ_TX_NONE;
  
  struct rte_eth_rxconf rxq_conf;
  struct rte_eth_txconf txq_conf;
  struct rte_eth_conf local_port_conf = port_conf;
  struct rte_eth_dev_info dev_info;

  /* init port */
  printf("Initializing port %u... ", m_portId);
  fflush(stdout);
  rte_eth_dev_info_get(m_portId, &dev_info);
  if (dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
    local_port_conf.txmode.offloads |=
      DEV_TX_OFFLOAD_MBUF_FAST_FREE;
  ret = rte_eth_dev_configure(m_portId, 1, 1, &local_port_conf);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
          ret, m_portId);
    }

  ret = rte_eth_dev_adjust_nb_rx_tx_desc(m_portId, &nb_rxd, &nb_txd);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE,
          "Cannot adjust number of descriptors: err=%d, port=%u\n",
          ret, m_portId);
    }

  /* init one RX queue */
  fflush(stdout);
  rxq_conf = dev_info.default_rxconf;
  rxq_conf.offloads = local_port_conf.rxmode.offloads;
  ret = rte_eth_rx_queue_setup(m_portId, 0, nb_rxd,
              rte_eth_dev_socket_id(m_portId),
              &rxq_conf,
              l2fwd_pktmbuf_pool);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "rte_eth_rx_queue_setup:err=%d, port=%u\n",
          ret, m_portId);
    }

  /* init one TX queue on each port */
  fflush(stdout);
  txq_conf = dev_info.default_txconf;
  txq_conf.txq_flags = ETH_TXQ_FLAGS_IGNORE;
  txq_conf.offloads = local_port_conf.txmode.offloads;
  ret = rte_eth_tx_queue_setup(m_portId, 0, nb_txd,
      rte_eth_dev_socket_id(m_portId),
      &txq_conf);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup:err=%d, port=%u\n",
        ret, m_portId);
    }

  /* Initialize TX buffers */
  tx_buffer[m_portId] = (rte_eth_dev_tx_buffer*) rte_zmalloc_socket("tx_buffer",
      RTE_ETH_TX_BUFFER_SIZE(MAX_PKT_BURST), 0,
      rte_eth_dev_socket_id(m_portId));
  if (tx_buffer[m_portId] == NULL)
    {
      rte_exit(EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
          m_portId);
    }

  rte_eth_tx_buffer_init(tx_buffer[m_portId], MAX_PKT_BURST);

  // ret = rte_eth_tx_buffer_set_err_callback(tx_buffer[m_portId],
  //     rte_eth_tx_buffer_count_callback,
  //     &port_statistics[m_portId].dropped);
  // if (ret < 0)
  //   rte_exit(EXIT_FAILURE,
  //   "Cannot set error callback for tx buffer on port %u\n",
  //       m_portId);

  /* Start device */
  ret = rte_eth_dev_start(m_portId);
  if (ret < 0)
    rte_exit(EXIT_FAILURE, "rte_eth_dev_start:err=%d, port=%u\n",
        ret, m_portId);

  printf("done: \n");

  rte_eth_promiscuous_enable(m_portId);

  // /* initialize port stats */
  // memset(&port_statistics, 0, sizeof(port_statistics));

  CheckAllPortsLinkStatus();

}

void
DPDKNetDevice::SetDeviceName (std::string deviceName)
{
  m_deviceName = deviceName;
}

} // namespace ns3