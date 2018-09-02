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
#include <unistd.h>

#include <poll.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_common.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>

#define MAX_PKT_BURST 32 //define the maximum packet burst size
#define MEMPOOL_CACHE_SIZE 256 //define the cache size for the memory pool

// Configurable number of RX/TX ring descriptors
#define RTE_TEST_RX_DESC_DEFAULT 1024
#define RTE_TEST_TX_DESC_DEFAULT 1024
static uint16_t nb_rxd = RTE_TEST_RX_DESC_DEFAULT;
static uint16_t nb_txd = RTE_TEST_TX_DESC_DEFAULT;

struct rte_mempool *l2fwd_pktmbuf_pool = NULL;

#define MAX_RX_QUEUE_PER_LCORE 16
struct lcore_queue_conf {
	unsigned n_rx_port;
	unsigned rx_port_list[MAX_RX_QUEUE_PER_LCORE];
} __rte_cache_aligned;
struct lcore_queue_conf lcore_queue_conf[RTE_MAX_LCORE];

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
DPDKNetDevice::InitDPDK (int argc, char** argv)
{
  // Initialize DPDK EAL
  int ret = rte_eal_init(argc, argv);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
    }

  // Bind device to DPDK
  std::string command;
  printf("Binding %s to driver uio_pci_generic\n", command.c_str());
  command.append("$RTE_SDK/usertools/dpdk-devbind.py ");
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
  unsigned int nb_lcores = 0;

  struct lcore_queue_conf *qconf;
  qconf = NULL;
  unsigned rx_lcore_id = 1; //setting the lcore_id=1 directly as we have only 1 logical core as of now

  if (qconf != &lcore_queue_conf[rx_lcore_id]) 
    {
			/* Assigned a new logical core in the loop above. */
			qconf = &lcore_queue_conf[rx_lcore_id];
			printf("qconf set\n");
      nb_lcores++;
		}

  qconf->rx_port_list[qconf->n_rx_port] = m_portId;
	qconf->n_rx_port++;
	printf("Lcore %u: RX port %u\n", rx_lcore_id, m_portId);

  unsigned int nb_mbufs = RTE_MAX(nb_ports * (nb_rxd + nb_txd + MAX_PKT_BURST +
		nb_lcores * MEMPOOL_CACHE_SIZE), 8192U);

  printf("nb-mbufs------------%d\n",nb_mbufs);

	/* create the mbuf pool */
	l2fwd_pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", nb_mbufs,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_socket_id());

	if (l2fwd_pktmbuf_pool == NULL)
    {
      rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");
    }
  else
    {
      printf("mbuf pool initialization successful\n");
    }
}

void
DPDKNetDevice::SetDeviceName (std::string deviceName)
{
  m_deviceName = deviceName;
}

} // namespace ns3