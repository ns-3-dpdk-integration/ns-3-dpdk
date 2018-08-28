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

} // namespace ns3