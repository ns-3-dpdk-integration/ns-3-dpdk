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

#define MAX_PKT_BURST 32 //define the maximum packet burst size
#define MEMPOOL_CACHE_SIZE 256 //define the cache size for the memory pool

#define DEFAULT_RING_SIZE 256 //default rte ring size for tx and rx
#define MAX_TX_BURST 32 //maximum no of packets transmitted from rte_ring to nic
#define MAX_RX_BURST 32 //maximum no of packets read from nic to rte_ring

// Configurable number of RX/TX ring descriptors
#define RTE_TEST_RX_DESC_DEFAULT 1024
#define RTE_TEST_TX_DESC_DEFAULT 1024
static uint16_t nb_rxd = RTE_TEST_RX_DESC_DEFAULT;
static uint16_t nb_txd = RTE_TEST_TX_DESC_DEFAULT;

static struct rte_eth_dev_tx_buffer *tx_buffer[RTE_MAX_ETHPORTS];

static struct rte_eth_conf port_conf = {};

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DpdkNetDevice");

NS_OBJECT_ENSURE_REGISTERED (DpdkNetDevice);

volatile bool DpdkNetDevice::m_forceQuit = false;

DpdkNetDeviceReader::DpdkNetDeviceReader ()
  : m_stop(false),
    m_bufferSize(65536)
{
}

void
DpdkNetDeviceReader::SetBufferSize (uint32_t bufferSize)
{
  NS_LOG_FUNCTION (this << bufferSize);
  m_bufferSize = bufferSize;
}

void
DpdkNetDeviceReader::SetFdNetDevice (Ptr<FdNetDevice> device)
{
  NS_LOG_FUNCTION (this << device);

  if (device != 0)
    {
      m_device = device;
    }
}

DpdkNetDeviceReader::Data DpdkNetDeviceReader::DoRead (void)
{
  NS_LOG_FUNCTION (this);

  uint8_t *buf = (uint8_t *)malloc (m_bufferSize);
  NS_ABORT_MSG_IF (buf == 0, "malloc() failed");

  ssize_t len = 0;

  if (m_device)
    {
      len = m_device->Read (buf);
    }

  if (len <= 0)
    {
      free (buf);
      buf = 0;
    }
  return DpdkNetDeviceReader::Data (buf, len);
}

void
DpdkNetDeviceReader::Run (void)
{
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
  m_readCallback = readCallback;
  m_readThread = Create<SystemThread> (MakeCallback (&DpdkNetDeviceReader::Run, this));
  m_readThread->Start ();
}

void
DpdkNetDeviceReader::Stop ()
{
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
  SetFileDescriptor(1);
}

void
DpdkNetDevice::SetDeviceName (std::string deviceName)
{
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
  m_reader->Stop();
  m_forceQuit = true;
  rte_ring_free(m_txRing);
  rte_ring_free(m_rxRing);
}

void
DpdkNetDevice::CheckAllPortsLinkStatus(void)
{
#define CHECK_INTERVAL 100 /* 100ms */
#define MAX_CHECK_TIME 90 /* 9s (90 * 100ms) in total */
	uint8_t count, all_ports_up, print_flag = 0;
	struct rte_eth_link link;

	printf("\nChecking link status\n");
	fflush(stdout);
	for (count = 0; count <= MAX_CHECK_TIME; count++) {

		all_ports_up = 1;
		
    if (m_forceQuit)
			return;
		if ((1 << m_portId) == 0)
			continue;
		memset(&link, 0, sizeof(link));
		rte_eth_link_get(m_portId, &link);
		/* print link status if flag set */
		if (print_flag == 1) {
			if (link.link_status)
				printf(
				"Port%d Link Up. Speed %u Mbps - %s\n",
					m_portId, link.link_speed,
			(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
				("full-duplex") : ("half-duplex\n"));
			else
				printf("Port %d Link Down\n", m_portId);
			continue;
		}
		/* clear all_ports_up flag if any link down */
		if (link.link_status == ETH_LINK_DOWN) {
			all_ports_up = 0;
			break;
		}
		
    /* after finally printing all link status, get out */
		if (print_flag == 1)
			break;

		if (all_ports_up == 0) {
			printf(".");
			fflush(stdout);
			rte_delay_ms(CHECK_INTERVAL);
		}

		/* set the print_flag if all ports up or timeout */
		if (all_ports_up == 1 || count == (MAX_CHECK_TIME - 1)) {
			print_flag = 1;
			printf("done\n");
		}
	}
}

void
DpdkNetDevice::SignalHandler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\nSignal %d received, preparing to exit...\n",
				signum);
		m_forceQuit = true;
	}
}

void
DpdkNetDevice::HandleTx()
{
  int queueId = 0, nb_tx, ret;
  void** tx_buffer;
  
  // int ringno = rte_ring_count(m_txRing);
  // printf("No of entries in ring %d\n",ringno);  
  tx_buffer = (void**) malloc(MAX_TX_BURST * sizeof(struct rte_mbuf*));
  nb_tx = rte_ring_dequeue_burst(m_txRing, tx_buffer, MAX_TX_BURST, NULL);
  // printf("dequeue bulk done %d\n",nb_tx);
  
  if(nb_tx == 0)
      return;
  do {
    ret = rte_eth_tx_burst(m_portId, queueId, (struct rte_mbuf**) tx_buffer, nb_tx);
    tx_buffer += ret;
    nb_tx -= ret;
    printf("transmitted %d packets, %d  \n",ret,nb_tx);
  } while( nb_tx > 0 );
  
}

void
DpdkNetDevice::HandleRx()
{
  int queueId = 0;
  struct rte_mbuf* rx_buffer[MAX_RX_BURST];
  int nb_rx_nic, nb_rx = -1;
  
  nb_rx_nic = rte_eth_rx_burst(m_portId, queueId, rx_buffer, MAX_RX_BURST);
  
  if(nb_rx_nic!=0) {
    // printf("%d packets read from nic\n", nb_rx_nic);
    nb_rx = rte_ring_enqueue_burst(m_rxRing, (void **) rx_buffer, nb_rx_nic, NULL);
  }
  // if (nb_rx_nic > 0) {
  //   printf("%d packets read from nic\n", nb_rx_nic);
  //   nb_rx = rte_ring_enqueue(m_rxRing, (void**) rx_buffer);
  // }
  
  if(nb_rx > 0)
  {
    // printf("%d packets received from nic\n", nb_rx);
    // printf("rx ring size %d\n", rte_ring_count(m_rxRing));
  }
}

int
DpdkNetDevice::LaunchCore(void *arg)
{
  DpdkNetDevice *dpdkNetDevice = (DpdkNetDevice*) arg;
  unsigned lcore_id;
	lcore_id = rte_lcore_id();	
  if(lcore_id != 1)
    return 0;

  dpdkNetDevice->PrintCheck();
  while(!m_forceQuit)
  {
    // printf("calling HandleTx\n");
    dpdkNetDevice->HandleTx();    
    // printf("called HandleTx\n");
    // dpdkNetDevice->PrintCheck();

    dpdkNetDevice->HandleRx();

    // we use a period to check and notify of 200 us; it is a value close to the interrupt coalescence period of a real device
    usleep(200);
  }
  
  return 0;  
}

void
DpdkNetDevice::PrintCheck()
{
  printf("hello world\n");
}

bool 
DpdkNetDevice::IsLinkUp (void) const
{
  struct rte_eth_link link;
  memset(&link, 0, sizeof(link));
	rte_eth_link_get(m_portId, &link);
  if (link.link_status)
		return true;
  return false;
}


void
DpdkNetDevice::InitDpdk (int argc, char** argv)
{

  // Initialize Dpdk EAL
  int ret = rte_eal_init(argc, argv);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
    }

  m_forceQuit = false;
  signal(SIGINT, SignalHandler);
	signal(SIGTERM, SignalHandler);

  // Bind device to Dpdk
  std::string command;
  printf("Binding %s to driver uio_pci_generic\n", command.c_str());
  command.append("$RTE_SDK/usertools/dpdk-devbind.py --force ");
  command.append("--bind=igb_uio ");
  command.append(m_deviceName.c_str());
  printf("Executing %s\n", command.c_str());
  if (system(command.c_str()))
    {
      rte_exit(EXIT_FAILURE, "Execution failed - bye\n");
    }

  // wait for the device to bind to Dpdk
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
  unsigned int nb_lcores = 1;

  unsigned int nb_mbufs = RTE_MAX(nb_ports * (nb_rxd + nb_txd + MAX_PKT_BURST +
		nb_lcores * MEMPOOL_CACHE_SIZE), 8192U);

  printf("nb-mbufs------------%d\n",nb_mbufs);

	/* create the mbuf pool */
	m_mempool = rte_pktmbuf_pool_create("mbuf_pool", nb_mbufs,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());

	if (m_mempool == NULL)
    {
      rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");
    }
  
  printf("mbuf pool initialization successful\n");

  // Initialize port
	port_conf.rxmode = {};
	port_conf.rxmode.split_hdr_size = 0;
	port_conf.rxmode.ignore_offload_bitfield = 1;
	port_conf.rxmode.offloads = DEV_RX_OFFLOAD_CRC_STRIP;
	port_conf.txmode = {};
  port_conf.txmode.mq_mode = ETH_MQ_TX_NONE;
  
  struct rte_eth_rxconf rxq_conf;
  struct rte_eth_txconf txq_conf;
  struct rte_eth_conf local_port_conf = port_conf;
  struct rte_eth_dev_info dev_info;

  /* init port */
  printf("Initializing port %u... ", m_portId);
  fflush(stdout);
  rte_eth_dev_info_get(m_portId, &dev_info);
  if (dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
    {
      local_port_conf.txmode.offloads |=
        DEV_TX_OFFLOAD_MBUF_FAST_FREE;
    }
  ret = rte_eth_dev_configure(m_portId, 1, 1, &local_port_conf);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
          ret, m_portId);
    }

  ret = rte_eth_dev_adjust_nb_rx_tx_desc(m_portId, &nb_rxd, &nb_txd);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE,
          "Cannot adjust number of descriptors: err=%d, port=%u\n",
          ret, m_portId);
    }

  /* init one RX queue */
  fflush(stdout);
  rxq_conf = dev_info.default_rxconf;
  rxq_conf.offloads = local_port_conf.rxmode.offloads;
  ret = rte_eth_rx_queue_setup(m_portId, 0, nb_rxd,
              rte_eth_dev_socket_id(m_portId),
              &rxq_conf,
              m_mempool);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "rte_eth_rx_queue_setup:err=%d, port=%u\n",
          ret, m_portId);
    }

  /* init one TX queue on each port */
  fflush(stdout);
  txq_conf = dev_info.default_txconf;
  txq_conf.txq_flags = ETH_TXQ_FLAGS_IGNORE;
  txq_conf.offloads = local_port_conf.txmode.offloads;
  ret = rte_eth_tx_queue_setup(m_portId, 0, nb_txd,
      rte_eth_dev_socket_id(m_portId),
      &txq_conf);
  if (ret < 0)
    {
      rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup:err=%d, port=%u\n",
        ret, m_portId);
    }

  /* Initialize TX buffers */
  tx_buffer[m_portId] = (rte_eth_dev_tx_buffer*) rte_zmalloc_socket("tx_buffer",
      RTE_ETH_TX_BUFFER_SIZE(MAX_PKT_BURST), 0,
      rte_eth_dev_socket_id(m_portId));
  if (tx_buffer[m_portId] == NULL)
    {
      rte_exit(EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
          m_portId);
    }

  rte_eth_tx_buffer_init(tx_buffer[m_portId], MAX_PKT_BURST);

  // ret = rte_eth_tx_buffer_set_err_callback(tx_buffer[m_portId],
  //     rte_eth_tx_buffer_count_callback,
  //     &port_statistics[m_portId].dropped);
  // if (ret < 0)
  //   rte_exit(EXIT_FAILURE,
  //   "Cannot set error callback for tx buffer on port %u\n",
  //       m_portId);

  /* Start device */
  ret = rte_eth_dev_start(m_portId);
  if (ret < 0)
    rte_exit(EXIT_FAILURE, "rte_eth_dev_start:err=%d, port=%u\n",
        ret, m_portId);

  printf("done: \n");

  rte_eth_promiscuous_enable(m_portId);

  // /* initialize port stats */
  // memset(&port_statistics, 0, sizeof(port_statistics));

  CheckAllPortsLinkStatus();

  // initialize 2 rings for transmission and receival of packets
  m_txRing = rte_ring_create("TX", m_ringSize, rte_socket_id(), RING_F_SP_ENQ | RING_F_SC_DEQ);
  if (m_txRing == NULL)
    rte_exit(EXIT_FAILURE, "Error in creating Tx ring.\n");
  else 
    printf("Tx ring created successfully.\n");

  m_rxRing = rte_ring_create("RX", m_ringSize, rte_socket_id(), RING_F_SP_ENQ | RING_F_SC_DEQ);
  if (m_rxRing == NULL)
    rte_exit(EXIT_FAILURE, "Error in creating Rx ring.\n");
  else
    printf("Rx ring created successfully.\n");

  rte_eal_mp_remote_launch(LaunchCore, this, CALL_MASTER);
}

void 
DpdkNetDevice::SetRteRingSize(int ringSize)
{
  m_ringSize = ringSize;
}


ssize_t
DpdkNetDevice::Write(uint8_t *buffer, size_t length)
{
  struct rte_mbuf *pkt;
//  char *data;

  pkt = rte_pktmbuf_alloc(m_mempool); 
  pkt->data_len = length;
  pkt->pkt_len = length;

  printf("dpdknetdevice write\n");
  char* pkt_data = rte_pktmbuf_mtod_offset(pkt, char*, 0);
  memcpy(pkt_data, buffer, length);
//  data = rte_pktmbuf_append(pkt, length);
//  if (data != NULL)
//    memcpy(data, buffer, length);
//  else {
//    printf("Unable to memcpy\n");
//    return -1; // Unable to append length in rte_pktmbuf_append()
//  }

  if(rte_ring_enqueue(m_txRing, pkt)) {
    printf("Unable to enqueue\n");
    return -1;
  }

  printf("Written %d bytes\n", (int) length);

  return length;
}


ssize_t
DpdkNetDevice::Read(uint8_t *buffer)
{
  // printf("READ called\n");
  void *item;
  struct rte_mbuf *pkt;
  uint8_t *dataBuffer;
  int length;

  if(rte_ring_dequeue(m_rxRing, &item) != 0)
  {
    // printf("Unable to dequeue\n");
    return -1;
  }

  pkt = (struct rte_mbuf*) item;
  // printf("RTE RING Dequeue done\n");
  
  dataBuffer = new uint8_t[pkt->pkt_len]; 
  dataBuffer = (uint8_t *) rte_pktmbuf_read(pkt, 0, pkt->pkt_len, dataBuffer);
  
  if(dataBuffer == NULL)
    printf("rtepktmbuf read not working-mbuf too small\n");

  memcpy(buffer, dataBuffer, pkt->pkt_len);

  length = pkt->pkt_len;
  rte_pktmbuf_free(pkt);

  return length;
}

} // namespace ns3
