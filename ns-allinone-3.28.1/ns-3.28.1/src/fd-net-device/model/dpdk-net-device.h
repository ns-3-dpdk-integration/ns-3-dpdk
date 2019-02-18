/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 */

#ifndef DPDK_NET_DEVICE_H
#define DPDK_NET_DEVICE_H

#include "fd-net-device.h"

#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_cycles.h>

#include <unordered_map>

struct rte_eth_dev_tx_buffer;
struct rte_mbuf;

namespace ns3
{

  class Node;
  class DpdkNetDevice;

  class NetDeviceQueueInterface;
  class NetDeviceQueue;

  /**
 * \ingroup fd-net-device
 * \brief This class performs the actual data reading from the DpdkNetDevice.
 */
  class DpdkNetDeviceReader : public Object
  {
  public:
    DpdkNetDeviceReader();

    /**
   * Set size of the read buffer.
   */
    void SetBufferSize(uint32_t bufferSize);

    /**
   * Set the device.
   */
    void SetFdNetDevice(Ptr<DpdkNetDevice> device);

    /**
   * The asynchronous function which performs read operation from DpdkNetDevice.
   */
    void Run(void);

    /**
   * Start a new read thread.
   *
   * \param [in] readCallback A callback to invoke when new data is
   * available.
   */
    void Start(Callback<void, uint8_t *, ssize_t> readCallback);

    /**
   * Stop the read thread and reset internal state. This does not
   * close the file descriptor used for reading.
   */
    void Stop(void);

  protected:
    /**
   * \brief A structure representing data read.
   */
    struct Data
    {
      /** Default constructor, with null buffer and zero length. */
      Data() : m_buf(0),
               m_len(0)
      {
      }
      /**
     * Construct from a buffer of a given length.
     *
     * \param [in] buf The buffer.
     * \param [in] len The size of the buffer, in bytes.
     */
      Data(uint8_t *buf, ssize_t len) : m_buf(buf),
                                        m_len(len)
      {
      }
      /** The read data buffer. */
      uint8_t *m_buf;
      /** The size of the read data buffer, in bytes. */
      ssize_t m_len;
    };

  private:
    DpdkNetDeviceReader::Data DoRead(void);

    Ptr<DpdkNetDevice> m_device;

    /** Signal the read thread to stop. */
    bool m_stop;

    uint32_t m_bufferSize; //!< size of the read buffer

    /** The main thread callback function to invoke when we have data. */
    Callback<void, uint8_t *, ssize_t> m_readCallback;

    /** The thread doing the read, created and launched by Start(). */
    Ptr<SystemThread> m_readThread;
  };

  /**
 * \ingroup fd-net-device
 *
 * \brief a NetDevice to read/write network traffic from/into a Dpdk enabled port.
 *
 * A DpdkNetDevice object will read and write frames/packets from/to a Dpdk enabled port.
 *
 */
  class DpdkNetDevice : public FdNetDevice
  {
  public:
    /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
    static TypeId GetTypeId(void);

    /**
   * Constructor for the NetmapNetDevice.
   */
    DpdkNetDevice();

    /**
   * Check the link status of all ports in up to 9s, and print them finally
   */
    void CheckAllPortsLinkStatus(void);

    /**
   * Initialize Dpdk.
   * Initializes EAL.
   *
   * \param argc Dpdk EAL args count.
   * \param argv Dpdk EAL args list.
   */
    void InitDpdk(int argc, char **argv);

    /**
   * Set device name.
   *
   * \param deviceName The device name.
   */
    void SetDeviceName(std::string deviceName);

    /**
   * A signal handler for SIGINT and SIGTERM signals.
   *
   * \param signum The signal number.
   */
    static void SignalHandler(int signum);

    /**
   * A function to set the rte_ring size value.
   *
   * \param ringSize Size of the ring.
   */
    void SetRteRingSize(int ringSize);

    /**
   * A function to set the timeout value.
   *
   * \param timeout Timeout value.
   */
    void SetTimeout (int timeout);

    /**
   * A function to handle rx & tx operations.
   */
    static int LaunchCore(void *arg);

    /**
   * Transmit packets in burst from the rte_ring to the nic.
   */
    void HandleTx();

    /**
   * Receive packets in burst from the nic to the rte_ring.
   */
    void HandleRx();

    /**
   * Check the status of the link.
   * \return Status of the link - up/down as true/false.
   */
    bool IsLinkUp(void) const;

    /**
   * Launches a function LaunchCore to provide a dedicated core to handle incoming packets from the NIC.
   */
    void StartSimulation(void);

    /**
   * Runs and Destroys the ns-3 Simulator.
   */
    void _StartSimulation(void);

     /**
   * Free the given packet buffer.
   */
    virtual void FreeBuffer (uint8_t* buf);
    
    /**
   * Allocate packet buffer.
   */
    virtual uint8_t* AllocateBuffer(size_t len);

    /**
   * Read packet data from device.
   * \param buffer Buffer the data to be read to.
   * \return The size of data read.
   */
    struct rte_mbuf* Read();

    virtual void NotifyNewAggregate (void);

  protected:
    /**
   * Spin up the device
   */
        void StartDevice(void);

    /**
   * Tear down the device
   */
    void StopDevice(void);

    /**
   * Write packet data to device.
   * \param buffer The data.
   * \param length The data length.
   * \return The size of data written.
   */
    ssize_t Write(uint8_t *buffer, size_t length);

    /**
   * The port number of the device to be used.
   */
    uint16_t m_portId;

    /**
   * The device name;
   */
    std::string m_deviceName;

  private:
  /**
   * Reader for the file descriptor.
   */
    Ptr<DpdkNetDeviceReader> m_reader;

    static volatile bool m_forceQuit; //!< Condition variable for Dpdk to stop
    int m_ringSize;                   //!< Size of tx and rx ring
    struct rte_ring *m_txRing;        //!< Instance of rte ring for transmission
    struct rte_ring *m_rxRing;        //!< Instance of rte ring for receival
    struct rte_mempool *m_mempool;    //!< Pakcet memory pool
    struct rte_eth_dev_tx_buffer *m_txBuffer;
    struct rte_eth_dev_tx_buffer *m_rxBuffer;
    int m_rxBufferHead;
    uint64_t m_nextTxTsc;
    uint64_t m_txTimeout;
    struct rte_mbuf* m_lastRxPkt;
    EventId m_txEvent;

    Ptr<NetDeviceQueueInterface> m_queueInterface; //!< NetDevice queue interface
    Ptr<NetDeviceQueue> m_queue;                   //!< NetDevice queue
  };

} //

#endif /* DPDK_NET_DEVICE_H */