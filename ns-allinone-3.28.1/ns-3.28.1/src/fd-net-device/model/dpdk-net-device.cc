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

  if (likely(m_device))
    {
      // clock_t begin = clock();
      // std::pair<uint8_t*, size_t> pktData = m_device->Read ();
      // buf = pktData.first;
      // len = pktData.second;
      struct rte_mbuf* pkt = m_device->Read ();
      if (pkt) {
        buf = rte_pktmbuf_mtod(pkt, uint8_t *);
        len = pkt->pkt_len;
      }
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
  rte_eth_stats_reset(0);

  while (likely(!m_stop))
    {
      m_device->HandleTx();
    }
}

void
DpdkNetDeviceReader::Start (Callback<void, uint8_t *, ssize_t> readCallback)
{
  NS_LOG_FUNCTION (this);

  m_readCallback = readCallback;
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
  m_txTimeout = 2000;
  m_nextTxTsc = rte_rdtsc() + m_txTimeout;
  m_rxBufferHead = 0;
  m_lastRxPkt = NULL;
  m_queue = 0;
  SetFileDescriptor(1);
}

void
DpdkNetDevice::SetTimeout (int timeout)
{
  printf("Setting DPDK timeout = %d us\n", timeout);
  m_txTimeout = timeout;
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

  rte_eth_stats_reset(m_portId);
  NotifyLinkUp ();
}

void
DpdkNetDevice::StopDevice (void)
{
  NS_LOG_FUNCTION (this);

  ns3::FdNetDevice::StopDevice ();
  Simulator::Cancel (m_txEvent);
  m_reader->Stop ();
  m_forceQuit = true;
  
  rte_eal_wait_lcore(1);

  rte_eth_dev_stop (m_portId);
  rte_eth_dev_close (m_portId);
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
  int queueId = 0;
  // $$
  int sent = rte_eth_tx_buffer_flush (m_portId, queueId, m_txBuffer);
  if(sent>0){
    // m_queue->NotifyTransmittedBytes(sent*length);
    m_queue->Wake();
  }
}

// $$
void
DpdkNetDevice::TxRequeueErrCallback (struct rte_mbuf **unsent, uint16_t count, void *userdata)
{
  struct rte_eth_dev_tx_buffer * txBuffer = (rte_eth_dev_tx_buffer*) userdata;
  int queueId = 0;
  int portId = 0;
  for(int i=0;i<count;i++)
  {
    rte_eth_tx_buffer(portId, queueId, txBuffer, unsent[i]);
  }
}

void
DpdkNetDevice::HandleRx ()
{
  int queueId = 0;
  m_rxBuffer->length = rte_eth_rx_burst(m_portId, queueId, m_rxBuffer->pkts, MAX_PKT_BURST);

  for(uint16_t i = 0; i < m_rxBuffer->length; i++)
  {
    struct rte_mbuf *pkt = NULL;
    pkt = m_rxBuffer->pkts[i];

    if (!pkt)
    {
      continue;
    }

    uint8_t * buf = rte_pktmbuf_mtod(pkt, uint8_t *);
    size_t length = pkt->data_len;
    FdNetDevice::ReceiveCallback(buf,length);
  }

  m_rxBuffer->length = 0; 
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
      return 0;
    }

  rte_eth_stats_reset(0);

  while (!m_forceQuit)
    {
      dpdkNetDevice->HandleRx ();
    }

  return 0;
}

bool
DpdkNetDevice::IsLinkUp (void) const
{
  return true;
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
  unsigned int nbLcores = 2;
  static uint16_t nbRxd = RTE_TEST_RX_DESC_DEFAULT;
  static uint16_t nbTxd = RTE_TEST_TX_DESC_DEFAULT;

  unsigned int nbMbufs = RTE_MAX (nbPorts * (nbRxd + nbTxd + MAX_PKT_BURST +
                                             MAX_PKT_BURST +
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
  if (m_txBuffer == NULL || m_rxBuffer == NULL)
    {
      rte_exit (EXIT_FAILURE, "Cannot allocate buffer for rx/tx on port %u\n",
                m_portId);
    }

  rte_eth_tx_buffer_init (m_txBuffer, MAX_PKT_BURST);
  rte_eth_tx_buffer_init (m_rxBuffer, MAX_PKT_BURST);

  // $$
  // Setting the callback function to re-queue the packets after burst from m_txBuffer
  rte_eth_tx_buffer_set_err_callback(m_txBuffer, TxRequeueErrCallback, m_txBuffer);

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

  rte_eth_stats_reset(m_portId);

  NS_LOG_INFO ("Launching core threads");
  rte_eal_mp_remote_launch (LaunchCore, this, CALL_MASTER);
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

  try {
    rte_pktmbuf_free(pkt);
  } catch (...) {
    NS_LOG_ERROR ("Cant free buffer " << buf);
  }
}


void
DpdkNetDevice::NotifyNewAggregate (void)
{
  NS_LOG_FUNCTION (this);
  if (m_queueInterface == 0)
    {
      Ptr<NetDeviceQueueInterface> ndqi = this->GetObject<NetDeviceQueueInterface> ();
      //verify that it's a valid netdevice queue interface and that
      //the netdevice queue interface was not set before
      if (ndqi != 0)
        {
          m_queueInterface = ndqi;
        }
    }
  NetDevice::NotifyNewAggregate ();
  if (m_queueInterface) {
    m_queueInterface->SetTxQueuesN (1);
    m_queueInterface->CreateTxQueues ();
    m_queue = m_queueInterface->GetTxQueue (0);
    if (m_queue) {
      // m_queue->SetWakeCallback ( MakeCallback (&DpdkNetDevice::HandleRx, this) );
      m_queue->Start();
    }
  }
}

ssize_t
DpdkNetDevice::Write(uint8_t *buffer, size_t length)
{
  struct rte_mbuf ** pkt = new struct rte_mbuf*[1];
  int queueId = 0;
  // $$
  int pkt_buffered_res = -1;

  if (buffer == NULL || m_txBuffer->length == MAX_PKT_BURST) {
    return -1;
  }

  // $$
  if (m_queue->IsStopped ()){
    // The device queue is stopped and we cannot write other packets
    return -1;
  }

  pkt[0] = (struct rte_mbuf *)RTE_PTR_SUB(buffer, 
                                sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM);

  pkt[0]->pkt_len = length;
  pkt[0]->data_len = length;
  pkt_buffered_res = rte_eth_tx_buffer(m_portId, queueId, m_txBuffer, pkt[0]);

  // $$
  if(pkt_buffered_res == 0){
    // If response if zero it means that the packet was successfully buffered and that the m_txBuffer was not burst
    // We notify the NetDeviceQueue of the amount of data queued(=length bytes)
    m_queue->NotifyQueuedBytes(length);
  }

  // $$
  if(pkt_buffered_res > 0){
    // If the response is greater than zero, it means that the buffer was burst and it sent those many packets
    m_queue->NotifyQueuedBytes(length);
    m_queue->NotifyTransmittedBytes(pkt_buffered_res*length);
  }

  // $$
  if (m_txBuffer->length == 1) {
    // If this is a first packet in buffer, schedule a tx.
    Simulator::Cancel (m_txEvent);
    m_txEvent = Simulator::Schedule ( Time ( MicroSeconds(m_txTimeout) ), &DpdkNetDevice::HandleTx, this);
  }

  // $$
  if(m_txBuffer->length == MAX_PKT_BURST){
    // The Buffer is now full, we will stop the m_queue
    m_queue->Stop();
  }

  return length;
}


struct rte_mbuf*
DpdkNetDevice::Read ()
{
  struct rte_mbuf *pkt = NULL;

  if (unlikely(m_rxBuffer->length == 0 || m_rxBufferHead == m_rxBuffer->length))
  {
    return NULL;
  }

  pkt = m_rxBuffer->pkts[m_rxBufferHead++];
  return pkt;
}

} // namespace ns3
