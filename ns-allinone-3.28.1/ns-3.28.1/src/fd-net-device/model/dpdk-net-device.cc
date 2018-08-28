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
}

} // namespace ns3