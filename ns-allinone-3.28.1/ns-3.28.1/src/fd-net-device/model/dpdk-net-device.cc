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
#include <rte_port.h>

#define MAX_PKT_BURST 64 //define the maximum packet burst size
#define MEMPOOL_CACHE_SIZE 256 //define the cache size for the memory pool

#define DEFAULT_RING_SIZE 256 //default rte ring size for tx and rx
#define MAX_TX_BURST 1 //maximum no of packets transmitted from rte_ring to nic
#define MAX_RX_BURST 1 //maximum no of packets read from nic to rte_ring

#define RTE_TEST_RX_DESC_DEFAULT 1024 //number of RX ring descriptors
#define RTE_TEST_TX_DESC_DEFAULT 1024 //number of TX ring descriptors

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DpdkNetDevice");

NS_OBJECT_ENSURE_REGISTERED (DpdkNetDevice);

volatile bool DpdkNetDevice::m_forceQuit = false;

DpdkNetDeviceReader::DpdkNetDeviceReader ()
  : m_stop (false),
    m_bufferSize (65536)
{
}

void
DpdkNetDeviceReader::SetBufferSize (uint32_t bufferSize)
{
  NS_LOG_FUNCTION (this << bufferSize);
  m_bufferSize = bufferSize;
}

void
DpdkNetDeviceReader::SetFdNetDevice (Ptr<DpdkNetDevice> device)
{
  NS_LOG_FUNCTION (this << device);

  if (device != 0)
    {
      m_device = device;
    }
}

DpdkNetDeviceReader::Data DpdkNetDeviceReader::DoRead (void)
{
  // NS_LOG_FUNCTION (this); because this is called infinitely

  uint8_t* buf = NULL;
  size_t len = -1;

  if (m_device)
    {
      // clock_t begin = clock();
      std::pair<uint8_t*, size_t> pktData = m_device->Read ();
      buf = pktData.first;
      len = pktData.second;
      // clock_t end = clock();
      // double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
      // printf("FdNetDevice::Read %f\n", time_spent * 1000000.0);
    }

  if (len <= 0)
    {
      m_device->FreeBuffer (buf);
    }

  return DpdkNetDeviceReader::Data (buf, len);
}

void
DpdkNetDeviceReader::Run (void)
{
  NS_LOG_FUNCTION (this);

  while (!m_stop)
    {
      struct DpdkNetDeviceReader::Data data = DoRead ();
      // reading stops when m_len is zero
      if (data.m_len == 0)
        {
          break;
        }
      // the callback is only called when m_len is positive (data
      // is ignored if m_len is negative)
      else if (data.m_len > 0)
        {
          m_readCallback (data.m_buf, data.m_len);
        }
    }
}

void
DpdkNetDeviceReader::Start (Callback<void, uint8_t *, ssize_t> readCallback)
{
  NS_LOG_FUNCTION (this);

  m_readCallback = readCallback;
  m_readThread = Create<SystemThread> (MakeCallback (&DpdkNetDeviceReader::Run, this));
  m_readThread->Start ();
}

void
DpdkNetDeviceReader::Stop ()
{
  NS_LOG_FUNCTION (this);

  m_stop = true;
  // join the read thread
  if (m_readThread != 0)
    {
      m_readThread->Join ();
      m_readThread = 0;
    }
  m_readCallback.Nullify ();
  m_stop = false;
}

TypeId
DpdkNetDevice::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DpdkNetDevice")
    .SetParent<FdNetDevice> ()
    .SetGroupName ("FdNetDevice")
    .AddConstructor<DpdkNetDevice> ()
  ;
  return tid;
}

DpdkNetDevice::DpdkNetDevice ()
{
  NS_LOG_FUNCTION (this);
  m_ringSize = DEFAULT_RING_SIZE;
  m_mempool = NULL;
  m_txTimeout = rte_get_timer_hz() / 2048;
  m_nextTxTsc = rte_rdtsc() + m_txTimeout;
  m_rxBufferHead = 0;
  SetFileDescriptor(1);
}

void
DpdkNetDevice::SetDeviceName (std::string deviceName)
{
  NS_LOG_FUNCTION (this);

  m_deviceName = deviceName;
}

void
DpdkNetDevice::StartDevice (void)
{
  NS_LOG_FUNCTION (this);
  //
  // A similar story exists for the node ID.  We can't just naively do a
  // GetNode ()->GetId () since GetNode is going to give us a Ptr<Node> which
  // is reference counted.  We need to stash away the node ID for use in the
  // read thread.
  //
  m_nodeId = GetNode ()->GetId ();

  m_reader = Create<DpdkNetDeviceReader> ();
  // 22 bytes covers 14 bytes Ethernet header with possible 8 bytes LLC/SNAP
  m_reader->SetFdNetDevice (this);
  m_reader->SetBufferSize (m_mtu + 22);
  m_reader->Start (MakeCallback (&FdNetDevice::ReceiveCallback, this));

  NotifyLinkUp ();
}

void
DpdkNetDevice::StopDevice (void)
{
  NS_LOG_FUNCTION (this);

  ns3::FdNetDevice::StopDevice ();
  m_reader->Stop ();
  m_forceQuit = true;
  rte_ring_free (m_txRing);
  rte_ring_free (m_rxRing);
}

void
DpdkNetDevice::CheckAllPortsLinkStatus (void)
{
  NS_LOG_FUNCTION (this);

  #define CHECK_INTERVAL 100 /* 100ms */
  #define MAX_CHECK_TIME 90 /* 9s (90 * 100ms) in total */
  uint8_t count, allPortsUp, printFlag = 0;
  struct rte_eth_link link;

  fflush (stdout);
  for (count = 0; count <= MAX_CHECK_TIME; count++)
    {

      allPortsUp = 1;

      if (m_forceQuit)
        {
          return;
        }
      if ((1 << m_portId) == 0)
        {
          continue;
        }
      memset (&link, 0, sizeof(link));
      rte_eth_link_get (m_portId, &link);
      /* print link status if flag set */
      if (printFlag == 1)
        {
          if (link.link_status)
            {
              continue;
            }
          else
            {
              printf ("Port %d Link Down\n", m_portId);
            }
          continue;
        }
      /* clear allPortsUp flag if any link down */
      if (link.link_status == ETH_LINK_DOWN)
        {
          allPortsUp = 0;
          break;
        }

      /* after finally printing all link status, get out */
      if (printFlag == 1)
        {
          break;
        }

      if (allPortsUp == 0)
        {
          fflush (stdout);
          rte_delay_ms (CHECK_INTERVAL);
        }

      /* set the printFlag if all ports up or timeout */
      if (allPortsUp == 1 || count == (MAX_CHECK_TIME - 1))
        {
          printFlag = 1;
        }
    }
}

void
DpdkNetDevice::SignalHandler (int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    {
      printf ("\n\nSignal %d received, preparing to exit...\n",
              signum);
      m_forceQuit = true;
    }
}

void
DpdkNetDevice::HandleTx ()
{
  int queueId = 0, nbTx, ret;
  void** txBuffer;

  txBuffer = (void**) malloc (MAX_TX_BURST * sizeof(struct rte_mbuf*));
  clock_t begin = clock();
  nbTx = rte_ring_dequeue_burst (m_txRing, txBuffer, MAX_TX_BURST, NULL);
  clock_t end = clock();
  double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
  printf("TxRingCheck %f\n", time_spent * 1000000.0);

  if (nbTx == 0)
    {
      return;
    }
  do
  {
    begin = clock();
    ret = rte_eth_tx_burst(m_portId, queueId, (struct rte_mbuf **)txBuffer, nbTx);
    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Transmission %f\n", time_spent * 1000000.0);
    txBuffer += ret;
    nbTx -= ret;
    printf("Transmitted %d pkts\n", ret);
    }
  while ( nbTx > 0 );

}

void
DpdkNetDevice::HandleRx ()
{
  int queueId = 0;
  struct rte_mbuf* rxBuffer[MAX_RX_BURST];
  int nbRxNic;

  nbRxNic = rte_eth_rx_burst (m_portId, queueId, rxBuffer, MAX_RX_BURST);

  if (nbRxNic != 0)
    {
      rte_ring_enqueue_burst (m_rxRing, (void **) rxBuffer, nbRxNic, NULL);
    }
}

void DpdkNetDevice::_StartSimulation(void)
{
  Simulator::Run();
  Simulator::Destroy();
}

void DpdkNetDevice::StartSimulation(void)
{
  rte_eal_mp_remote_launch(LaunchCore, this, CALL_MASTER);
}

int
DpdkNetDevice::LaunchCore (void *arg)
{
  DpdkNetDevice *dpdkNetDevice = (DpdkNetDevice*) arg;
  unsigned lcoreId;
  lcoreId = rte_lcore_id ();
  if (lcoreId != 1)
    {
      dpdkNetDevice->_StartSimulation();
      return 0;
    }

  // while (!m_forceQuit)
    // {

      // clock_t begin = clock();
      // // dpdkNetDevice->HandleTx ();
      // clock_t end = clock();
      // double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
      // // printf("DpdkNetDevice::HandleTx %f\n", time_spent * 1000000.0);

      // begin = clock();
      // // dpdkNetDevice->HandleRx ();
      // end = clock();
      // time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
      // // printf("DpdkNetDevice::HandleRx %f\n", time_spent * 1000000.0);

      // we use a period to check and notify of 200 us; it is a value close to
      // the interrupt coalescence period of a real device
      // usleep (2);
    // }

  return 0;
}

bool
DpdkNetDevice::IsLinkUp (void) const
{
  struct rte_eth_link link;
  memset (&link, 0, sizeof(link));
  rte_eth_link_get (m_portId, &link);
  if (link.link_status)
    {
      return true;
    }
  return false;
}

void
DpdkNetDevice::InitDpdk (int argc, char** argv)
{
  NS_LOG_FUNCTION (this << argc << argv);

  NS_LOG_INFO ("Binding device to DPDK");
  std::string command;
  command.append ("$RTE_SDK/usertools/dpdk-devbind.py --force ");
  command.append ("--bind=uio_pci_generic ");
  command.append (m_deviceName.c_str ());
  printf ("Executing: %s\n", command.c_str ());
  if (system (command.c_str ()))
    {
      rte_exit (EXIT_FAILURE, "Execution failed - bye\n");
    }

  // wait for the device to bind to Dpdk
  sleep (5);  /* 5 seconds */

  NS_LOG_INFO ("Initialize DPDK EAL");
  int ret = rte_eal_init (argc, argv);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE, "Invalid EAL arguments\n");
    }

  m_forceQuit = false;
  signal (SIGINT, SignalHandler);
  signal (SIGTERM, SignalHandler);

  unsigned nbPorts = rte_eth_dev_count ();
  if (nbPorts == 0)
    {
      rte_exit (EXIT_FAILURE, "No Ethernet ports - bye\n");
    }

  NS_LOG_INFO ("Get port id of the device");
  if (rte_eth_dev_get_port_by_name (m_deviceName.c_str (), &m_portId) != 0)
    {
      rte_exit (EXIT_FAILURE, "Cannot get port id - bye\n");
    }

  // Set number of logical cores to 1
  unsigned int nbLcores = 1;
  static uint16_t nbRxd = RTE_TEST_RX_DESC_DEFAULT;
  static uint16_t nbTxd = RTE_TEST_TX_DESC_DEFAULT;

  unsigned int nbMbufs = RTE_MAX (nbPorts * (nbRxd + nbTxd + MAX_PKT_BURST +
                                             nbLcores * MEMPOOL_CACHE_SIZE), 8192U);

  NS_LOG_INFO ("Create the mbuf pool");
  m_mempool = rte_pktmbuf_pool_create ("mbuf_pool", nbMbufs,
                                       MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id ());

  if (m_mempool == NULL)
    {
      rte_exit (EXIT_FAILURE, "Cannot init mbuf pool\n");
    }

  NS_LOG_INFO ("Initialize port");
  static struct rte_eth_conf portConf = {};
  portConf.rxmode = {};
  portConf.rxmode.split_hdr_size = 0;
  portConf.rxmode.ignore_offload_bitfield = 1;
  portConf.rxmode.offloads = DEV_RX_OFFLOAD_CRC_STRIP;
  portConf.txmode = {};
  portConf.txmode.mq_mode = ETH_MQ_TX_NONE;

  struct rte_eth_rxconf reqConf;
  struct rte_eth_txconf txqConf;
  struct rte_eth_conf localPortConf = portConf;
  struct rte_eth_dev_info devInfo;

  fflush (stdout);
  rte_eth_dev_info_get (m_portId, &devInfo);
  if (devInfo.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
    {
      localPortConf.txmode.offloads |=
        DEV_TX_OFFLOAD_MBUF_FAST_FREE;
    }
  ret = rte_eth_dev_configure (m_portId, 1, 1, &localPortConf);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
                ret, m_portId);
    }

  ret = rte_eth_dev_adjust_nb_rx_tx_desc (m_portId, &nbRxd, &nbTxd);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE,
                "Cannot adjust number of descriptors: err=%d, port=%u\n",
                ret, m_portId);
    }

  NS_LOG_INFO ("Initialize one Rx queue");
  fflush (stdout);
  reqConf = devInfo.default_rxconf;
  reqConf.offloads = localPortConf.rxmode.offloads;
  ret = rte_eth_rx_queue_setup (m_portId, 0, nbRxd,
                                rte_eth_dev_socket_id (m_portId),
                                &reqConf,
                                m_mempool);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE, "rte_eth_rx_queue_setup:err=%d, port=%u\n",
                ret, m_portId);
    }

  NS_LOG_INFO ("Initialize one Tx queue per port");
  fflush (stdout);
  txqConf = devInfo.default_txconf;
  txqConf.txq_flags = ETH_TXQ_FLAGS_IGNORE;
  txqConf.offloads = localPortConf.txmode.offloads;
  ret = rte_eth_tx_queue_setup (m_portId, 0, nbTxd,
                                rte_eth_dev_socket_id (m_portId),
                                &txqConf);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE, "rte_eth_tx_queue_setup:err=%d, port=%u\n",
                ret, m_portId);
    }

  NS_LOG_INFO ("Initialize Tx buffers");
  m_txBuffer = (rte_eth_dev_tx_buffer*) rte_zmalloc_socket ("tx_buffer",
                                    RTE_ETH_TX_BUFFER_SIZE (MAX_PKT_BURST), 0,
                                    rte_eth_dev_socket_id (m_portId));
  NS_LOG_INFO ("Initialize Rx buffers");
  m_rxBuffer = (rte_eth_dev_tx_buffer*) rte_zmalloc_socket ("rx_buffer",
                                    RTE_ETH_TX_BUFFER_SIZE (MAX_PKT_BURST), 0,
                                    rte_eth_dev_socket_id (m_portId));
  if (m_txBuffer == NULL)
    {
      rte_exit (EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
                m_portId);
    }

  rte_eth_tx_buffer_init (m_txBuffer, MAX_PKT_BURST);

  NS_LOG_INFO ("Start the device");
  ret = rte_eth_dev_start (m_portId);
  if (ret < 0)
    {
      rte_exit (EXIT_FAILURE, "rte_eth_dev_start:err=%d, port=%u\n",
                ret, m_portId);
    }

  rte_eth_promiscuous_enable (m_portId);

  // /* initialize port stats */
  // memset(&port_statistics, 0, sizeof(port_statistics));

  CheckAllPortsLinkStatus ();

  NS_LOG_INFO ("Initialize rte_rings for Tx/Rx intermediate packet processing");
  m_txRing = rte_ring_create ("TX", m_ringSize, SOCKET_ID_ANY, 0);
  if (m_txRing == NULL)
    {
      rte_exit (EXIT_FAILURE, "Error in creating Tx ring.\n");
    }
  else
    {
      NS_LOG_LOGIC ("Tx rte_ring created successfully: " << m_txRing);
    }

  m_rxRing = rte_ring_create ("RX", m_ringSize, SOCKET_ID_ANY, 0);
  if (m_rxRing == NULL)
    {
      rte_exit (EXIT_FAILURE, "Error in creating Rx ring.\n");
    }
  else
    {
      NS_LOG_LOGIC ("Rx rte_ring created successfully: " << m_rxRing);
    }

  NS_LOG_INFO ("Launching core threads");
}

void
DpdkNetDevice::SetRteRingSize (int ringSize)
{
  NS_LOG_FUNCTION (this << ringSize);

  m_ringSize = ringSize;
}

uint8_t*
DpdkNetDevice::AllocateBuffer (size_t len)
{
  struct rte_mbuf *pkt = rte_pktmbuf_alloc(m_mempool);
  if (!pkt)
  {
    return NULL;
  }
  uint8_t *buf = rte_pktmbuf_mtod(pkt, uint8_t *);
  return buf;
}

void
DpdkNetDevice::FreeBuffer (uint8_t* buf)
{
  struct rte_mbuf *pkt;

  if (!buf)
    return;
  pkt = (struct rte_mbuf *)RTE_PTR_SUB(buf, sizeof(rte_mbuf) + RTE_PKTMBUF_HEADROOM);

  rte_pktmbuf_free(pkt);
}

ssize_t
DpdkNetDevice::Write(uint8_t *buffer, size_t length)
{
  struct rte_mbuf *pkt;

  if (buffer == NULL) {
    return -1;
  }

  pkt = (struct rte_mbuf *)RTE_PTR_SUB(buffer, 
                                sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM);

  pkt->pkt_len = length;
  pkt->data_len = length;

  int queueId = 0;

  // begin = clock();
  rte_eth_tx_buffer(m_portId, queueId, m_txBuffer, pkt);
  // end = clock();
  // time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
  // printf("Write::tx_buffer %f\n", time_spent * 1000000.0);

  uint64_t cur_tsc = rte_rdtsc();

  m_txTimeout = rte_get_timer_hz() / 2048;
  
  if (cur_tsc >= m_nextTxTsc)
  {
    rte_eth_tx_buffer_flush(m_portId, queueId, m_txBuffer);
    m_nextTxTsc = cur_tsc + m_txTimeout; 
  }

  return length;
}


std::pair<uint8_t*, size_t>
DpdkNetDevice::Read ()
{
  struct rte_mbuf *pkt = NULL;

  if (m_rxBuffer->length == 0 || m_rxBufferHead == m_rxBuffer->length)
  {
    m_rxBufferHead = 0;
    int queueId = 0;
    m_rxBuffer->length = rte_eth_rx_burst(m_portId, queueId, m_rxBuffer->pkts, MAX_PKT_BURST);
    if (m_rxBuffer->length == 0) {
      return std::make_pair((uint8_t*)NULL, -1);
    }
  }

  pkt = m_rxBuffer->pkts[m_rxBufferHead++];

  uint8_t* buf = NULL;
  buf = rte_pktmbuf_mtod(pkt, uint8_t *);

  return std::make_pair(buf, pkt->pkt_len);
}

} // namespace ns3
