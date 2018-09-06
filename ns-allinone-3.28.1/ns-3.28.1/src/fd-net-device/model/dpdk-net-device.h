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

  /**
   * Check the link status of all ports in up to 9s, and print them finally
   */
  void CheckAllPortsLinkStatus(void);

  /**
   * Initialize DPDK.
   * Initializes EAL.
   *
   * \param argc DPDK EAL args count.
   * \param argv DPDK EAL args list.
   */
  void InitDPDK (int argc, char** argv);

  /**
   * Set device name.
   * 
   * \param deviceName The device name.
   */
  void SetDeviceName(std::string deviceName);

  /**
   * A signal handler for SIGINT and SIGTERM signals.
   */
  static void SignalHandler(int signum);

protected:

  /**
   * The port number of the device to be used.
   */
  uint16_t m_portId;

  /**
   * The device name;
   */
  std::string m_deviceName;
};

} // 

#endif /* DPDK_NET_DEVICE_H */