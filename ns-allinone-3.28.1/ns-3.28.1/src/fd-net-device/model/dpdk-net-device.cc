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

#include <poll.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_common.h>

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
  // if (system(command.c_str()))
  //   {
  //     rte_exit(EXIT_FAILURE, "Execution failed - bye\n");
  //   }

  // // Get port id of the device
  // int status = rte_eth_dev_get_port_by_name (m_deviceName.c_str(), &m_portId);
  // if (status != 0)
  //   {
  //     printf("Cannot get port id for %s\n", m_deviceName.c_str());
  //     rte_exit(EXIT_FAILURE, "Cannot get port id - bye\n");
  //   }

  unsigned nb_ports = rte_eth_dev_count();
	if (nb_ports == 0)
    {
      rte_exit(EXIT_FAILURE, "No Ethernet ports - bye\n");
    }

  // unsigned int nb_mbufs = RTE_MAX(nb_ports * (nb_rxd + nb_txd + MAX_PKT_BURST +
	// 	nb_lcores * MEMPOOL_CACHE_SIZE), 8192U);

	// /* create the mbuf pool */
	// l2fwd_pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", nb_mbufs,
	// 	MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE,
	// 	rte_socket_id());
	// if (l2fwd_pktmbuf_pool == NULL)
  //   {
  //     rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");
  //   }
}

void
DPDKNetDevice::SetDeviceName (std::string deviceName)
{
  m_deviceName = deviceName;
}

} // namespace ns3