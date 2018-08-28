/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 */

#ifndef DPDK_NET_DEVICE_H
#define DPDK_NET_DEVICE_H

#include "fd-net-device.h"

namespace ns3 {

class Node;
class NetDeviceQueueInterface;
class SystemThread;
class NetDeviceQueue;


/**
 * \ingroup fd-net-device
 *
 * \brief a NetDevice to read/write network traffic from/into a dpdk enabled port.
 *
 * A DPDKNetDevice object will read and write frames/packets from/to a dpdk enabled port.
 *
 */
class DPDKNetDevice : public FdNetDevice
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor for the NetmapNetDevice.
   */
  DPDKNetDevice ();
};

} // 

#endif /* DPDK_NET_DEVICE_H */