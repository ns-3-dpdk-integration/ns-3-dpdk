/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 */

#ifndef DPDK_NET_DEVICE_H
#define DPDK_NET_DEVICE_H


#include "fd-net-device.h"

#include <rte_ring.h>
#include <rte_mempool.h>

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
   * 
   * \paran signum The signal number.
   */
  static void SignalHandler(int signum);

  /**
   * A function to set the rte_ring size value.
   * 
   * \paran ringSize Size of the ring.
   */
  void SetRteRingSize(int ringSize);

  /**
   * A function to handle rx & tx operations.
   */
  static int LaunchCore(void *arg);

  void PrintCheck();

  void HandleTx();

  void HandleRx();

  bool IsLinkUp (void) const;



protected:

  /**
   * Spin up the device
   */
  void StartDevice (void);

  /**
   * Tear down the device
   */
  void StopDevice (void);

  //   /**
  //  * Write packet data to device.
  //  * \param buffer The data.
  //  * \param length The data length.
  //  * \return The size of data written.
  //  */
  ssize_t Write (uint8_t *buffer, size_t length);

  // /**
  //  * Read packet data from device.
  //  * \param buffer Buffer the data to be read to.
  //  * \return The size of data read.
  //  */
  ssize_t Read (uint8_t *buffer);

  /**
   * The port number of the device to be used.
   */
  uint16_t m_portId;

  /**
   * The device name;
   */
  std::string m_deviceName;

private:

  static volatile bool m_forceQuit;           //!< Condition variable for DPDK to stop
  int m_ringSize;                             //!< Size of tx and rx ring         
  struct rte_ring *m_txRing;                  //!< Instance of rte ring for transmission
  struct rte_ring *m_rxRing;                  //!< Instance of rte ring for receival
  struct rte_mempool *m_mempool;              //!< packet mempool

};

} // 

#endif /* DPDK_NET_DEVICE_H */