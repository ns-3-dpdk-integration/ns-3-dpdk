/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 *
 */

#include "ns3/abort.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DpdkEmulationPingExample");

static void
StatsSampling (Ptr<QueueDisc> qdisc, Ptr<NetDevice> device, double samplingPeriod)
{
  Simulator::Schedule (Seconds (samplingPeriod), &StatsSampling, qdisc, device, samplingPeriod);
}

static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
}

int
main (int argc, char *argv[])
{
  NS_LOG_INFO ("Dpdk Emulation Ping Example");

  std::string deviceName ("0000:00:11.0");
  std::string macClient("08:00:27:b8:57:37");
  // ping a real host connected back-to-back through the ethernet interfaces
  std::string remote ("192.168.43.20");

  double samplingPeriod = 0.5; // s
  bool bql = false;

  CommandLine cmd;
  cmd.AddValue ("deviceName", "Device name", deviceName);
  cmd.AddValue ("remote", "Remote IP address (dotted decimal only please)", remote);
  cmd.AddValue ("bql", "Enable byte queue limits", bql);
  cmd.Parse (argc, argv);

  Ipv4Address remoteIp (remote.c_str ());
  // the OS IP for the eth0 interfaces is 10.0.1.1, and we set the ns-3 IP for eth0 to 10.0.1.11
  Ipv4Address localIp ("192.168.43.44");
  NS_ABORT_MSG_IF (localIp == "1.2.3.4", "You must change the local IP address before running this example");

  Ipv4Mask localMask ("255.255.255.0");

  //
  // Since we are using a real piece of hardware we need to use the realtime
  // simulator.
  //
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  //
  // Since we are going to be talking to real-world machines, we need to enable
  // calculation of checksums in our protocols.
  //
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // In such a simple topology, the use of the helper API can be a hindrance
  // so we drop down into the low level API and do it manually.
  //
  // First we need a single node.
  //
  NS_LOG_INFO ("Create Node");
  Ptr<Node> node = CreateObject<Node> ();

  //
  // Create an emu device, allocate a MAC address and point the device to the
  // Linux device name.  The device needs a transmit queueing discipline so
  // create a droptail queue and give it to the device.  Finally, "install"
  // the device into the node.
  //
  // Do understand that the ns-3 allocated MAC address will be sent out over
  // your network since the emu net device will spoof it.  By default, this
  // address will have an Organizationally Unique Identifier (OUI) of zero.
  // The Internet Assigned Number Authority IANA
  //
  //  http://www.iana.org/assignments/ethernet-numbers
  //
  // reports that this OUI is unassigned, and so should not conflict with
  // real hardware on your net.  It may raise all kinds of red flags in a
  // real environment to have packets from a device with an obviously bogus
  // OUI flying around.  Be aware.
  //
  NS_LOG_INFO ("Create Device");
  EmuFdNetDeviceHelper emu;

  // set the dpdk emulation mode
  char **ealArgv = new char*[20];
  ealArgv[0] = new char[20];
  strcpy (ealArgv[0], "");
  ealArgv[1] = new char[20];
  strcpy (ealArgv[1], "-l");
  ealArgv[2] = new char[20];
  strcpy (ealArgv[2], "0,1");
  ealArgv[3] = new char[20];
  strcpy (ealArgv[3], "-d");
  ealArgv[4] = new char[20];
  strcpy (ealArgv[4], "librte_pmd_e1000.so");
  ealArgv[5] = new char[20];
  strcpy (ealArgv[5], "-d");
  ealArgv[6] = new char[20];
  strcpy (ealArgv[6], "librte_mempool_ring.so");
  emu.SetDpdkMode (7, ealArgv);
  emu.SetDeviceName (deviceName);
  NetDeviceContainer devices = emu.Install (node);
  Ptr<NetDevice> device = devices.Get (0);
  device->SetAttribute ("Address", Mac48AddressValue (macClient.c_str ()));

  //Ptr<Queue> queue = CreateObject<DropTailQueue> ();
  //device->SetQueue (queue);
  //node->AddDevice (device);

  //
  // Add a default internet stack to the node.  This gets us the ns-3 versions
  // of ARP, IPv4, ICMP, UDP and TCP.
  //
  NS_LOG_INFO ("Add Internet Stack");
  InternetStackHelper internetStackHelper;
  internetStackHelper.Install (node);

  NS_LOG_INFO ("Create IPv4 Interface");
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  uint32_t interface = ipv4->AddInterface (device);
  Ipv4InterfaceAddress address = Ipv4InterfaceAddress (localIp, localMask);
  ipv4->AddAddress (interface, address);
  ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  //
  // When the ping application sends its ICMP packet, it will happily send it
  // down the ns-3 protocol stack.  We set the IP address of the destination
  // to the address corresponding to example.com above.  This address is off
  // our local network so we have got to provide some kind of default route
  // to ns-3 to be able to get that ICMP packet forwarded off of our network.
  //
  // You have got to provide an IP address of a real host that you can send
  // real packets to and have them forwarded off of your local network.  One
  // thing you could do is a 'netstat -rn' command and find the IP address of
  // the default gateway on your host and add it below, replacing the
  // "1.2.3.4" string.
  //
  Ipv4Address gateway ("192.168.43.20");
  NS_ABORT_MSG_IF (gateway == "1.2.3.4", "You must change the gateway IP address before running this example");

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
  staticRouting->SetDefaultRoute (gateway, interface);

  //
  // Create the ping application.  This application knows how to send
  // ICMP echo requests.  Setting up the packet sink manually is a bit
  // of a hassle and since there is no law that says we cannot mix the
  // helper API with the low level API, let's just use the helper.
  //
  NS_LOG_INFO ("Create V4Ping Appliation");
  Ptr<V4Ping> app = CreateObject<V4Ping> ();
  app->SetAttribute ("Remote", Ipv4AddressValue (remoteIp));
  app->SetAttribute ("Verbose", BooleanValue (true) );
  app->SetAttribute ("Interval", TimeValue (Seconds (samplingPeriod)));
  node->AddApplication (app);
  app->SetStartTime (Seconds (1.0));
  app->SetStopTime (Seconds (21.0));

  //
  // Give the application a name.  This makes life much easier when constructing
  // config paths.
  //
  Names::Add ("app", app);

  //
  // Hook a trace to print something when the response comes back.
  //
  Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  //
  // Enable a promiscuous pcap trace to see what is coming and going on our device.
  //
  emu.EnablePcap ("dpdk-emu-ping", device, true);

  //
  // Now, do the actual emulation.
  //
  NS_LOG_INFO ("Run Emulation.");
  Simulator::Stop (Seconds (22.0));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}
