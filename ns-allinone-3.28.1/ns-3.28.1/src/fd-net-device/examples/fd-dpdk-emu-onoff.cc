/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Universita' degli Studi di Napoli Federico II
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:
 *
 */

/**
 * TODO: Add description.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EmuFdNetDeviceSaturationExample");

// static void
// StatsSampling(Ptr<QueueDisc> qdisc, Ptr<NetDevice> device, double samplingPeriod)
// {
//     Simulator::Schedule(Seconds(samplingPeriod), &StatsSampling, qdisc, device, samplingPeriod);
//     Ptr<NetmapNetDevice> d = DynamicCast<NetmapNetDevice>(device);

//     std::cout << qdisc->GetNPackets() << " packets in the traffic-control queue disc" << std::endl;
//     if (d)
//     {
//         std::cout << d->GetBytesInNetmapTxRing() << " bytes inflight in the netmap tx ring" << std::endl;
//     }
// }

static void
PingRtt(std::string context, Time rtt)
{
    NS_LOG_UNCOND("Received Response with RTT = " << rtt);
}

int main(int argc, char *argv[])
{
    uint16_t sinkPort = 8000;
    uint32_t packetSize = 1400; // bytes
    std::string dataRate("950Mb/s");
    bool serverMode = false;

    std::string deviceName("eno1");
    std::string client("10.0.1.11");
    std::string server("10.0.1.22");
    std::string netmask("255.255.255.0");
    std::string macClient("00:00:00:00:00:01");
    std::string macServer("00:00:00:00:00:02");

    std::string transportProt = "Udp";
    std::string socketType;

    bool dpdkMode = true;
    bool ping = false;

    double samplingPeriod = 0.5; // s

    CommandLine cmd;
    cmd.AddValue("deviceName", "Device name", deviceName);
    cmd.AddValue("client", "Local IP address (dotted decimal only please)", client);
    cmd.AddValue("server", "Remote IP address (dotted decimal only please)", server);
    cmd.AddValue("localmask", "Local mask address (dotted decimal only please)", netmask);
    cmd.AddValue("serverMode", "1:true, 0:false, default client", serverMode);
    cmd.AddValue("mac-client", "Mac Address for Server Client : 00:00:00:00:00:01", macClient);
    cmd.AddValue("mac-server", "Mac Address for Server Default : 00:00:00:00:00:02", macServer);
    cmd.AddValue("data-rate", "Data rate defaults to 1000Mb/s", dataRate);
    cmd.AddValue("transportPort", "Transport protocol to use: Tcp, Udp", transportProt);
    cmd.AddValue("dpdkMode", "Enable the netmap emulation mode", dpdkMode);
    cmd.AddValue("ping", "Enable server ping client side", ping);
    cmd.Parse(argc, argv);

    Ipv4Address remoteIp;
    Ipv4Address localIp;
    Mac48AddressValue localMac;

    if (serverMode)
    {
        remoteIp = Ipv4Address(client.c_str());
        localIp = Ipv4Address(server.c_str());
        localMac = Mac48AddressValue(macServer.c_str());
    }
    else
    {
        remoteIp = Ipv4Address(server.c_str());
        localIp = Ipv4Address(client.c_str());
        localMac = Mac48AddressValue(macClient.c_str());
    }

    if (transportProt.compare("Tcp") == 0)
    {
        socketType = "ns3::TcpSocketFactory";
    }
    else
    {
        socketType = "ns3::UdpSocketFactory";
    }

    Ipv4Mask localMask(netmask.c_str());

    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));

    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    NS_LOG_INFO("Create Node");
    Ptr<Node> node = CreateObject<Node>();

    NS_LOG_INFO("Create Device");
    EmuFdNetDeviceHelper emu;

    // set the dpdk emulation mode
    if (dpdkMode)
    {
        // set the dpdk emulation mode
        char **ealArgv = new char *[20];
        ealArgv[0] = new char[20];
        strcpy(ealArgv[0], "");
        ealArgv[1] = new char[20];
        strcpy(ealArgv[1], "-l");
        ealArgv[2] = new char[20];
        strcpy(ealArgv[2], "0,1");
        ealArgv[3] = new char[20];
        strcpy(ealArgv[3], "-d");
        ealArgv[4] = new char[20];
        strcpy(ealArgv[4], "librte_pmd_e1000.so");
        ealArgv[5] = new char[20];
        strcpy(ealArgv[5], "-d");
        ealArgv[6] = new char[20];
        strcpy(ealArgv[6], "librte_mempool_ring.so");
        emu.SetDpdkMode(7, ealArgv);
    }

    emu.SetDeviceName(deviceName);
    NetDeviceContainer devices = emu.Install(node);
    Ptr<NetDevice> device = devices.Get(0);
    device->SetAttribute("Address", localMac);

    NS_LOG_INFO("Add Internet Stack");
    InternetStackHelper internetStackHelper;
    internetStackHelper.SetIpv4StackInstall(true);
    internetStackHelper.Install(node);

    // TODO: Add sampling code.
    // we enable the stats sampling client side only (we send traffic from client to server)
    // if (!serverMode)
    // {
    //     Simulator::Schedule(Seconds(samplingPeriod), &StatsSampling, qdiscs.Get(0), device, samplingPeriod);
    // }

    NS_LOG_INFO("Create IPv4 Interface");
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    uint32_t interface = ipv4->AddInterface(device);
    Ipv4InterfaceAddress address = Ipv4InterfaceAddress(localIp, localMask);
    ipv4->AddAddress(interface, address);
    ipv4->SetMetric(interface, 1);
    ipv4->SetUp(interface);

    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));

    if (serverMode)
    {
        Address sinkLocalAddress(InetSocketAddress(localIp, sinkPort));
        PacketSinkHelper sinkHelper(socketType, sinkLocalAddress);
        ApplicationContainer sinkApp = sinkHelper.Install(node);
        sinkApp.Start(Seconds(1));
        sinkApp.Stop(Seconds(30.0));

        emu.EnablePcap("fd-server", device);
    }
    else
    {
        // add traffic generator
        AddressValue remoteAddress(InetSocketAddress(remoteIp, sinkPort));
        OnOffHelper onoff(socketType, Address());
        onoff.SetAttribute("Remote", remoteAddress);
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetAttribute("DataRate", DataRateValue(dataRate));
        onoff.SetAttribute("PacketSize", UintegerValue(packetSize));

        ApplicationContainer clientApps = onoff.Install(node);
        clientApps.Start(Seconds(6.0));
        clientApps.Stop(Seconds(26.0));

        if (ping)
        {
            // add ping application
            Ptr<V4Ping> app = CreateObject<V4Ping>();
            app->SetAttribute("Remote", Ipv4AddressValue(remoteIp));
            app->SetAttribute("Verbose", BooleanValue(true));
            app->SetAttribute("Interval", TimeValue(Seconds(samplingPeriod)));
            node->AddApplication(app);
            app->SetStartTime(Seconds(5.0));
            app->SetStopTime(Seconds(27.0));

            Config::Connect("/Names/app/Rtt", MakeCallback(&PingRtt));
        }

        emu.EnablePcap("fd-client", device);
    }

    Simulator::Stop(Seconds(30));

    Ptr<DpdkNetDevice> dpdkNetDevice = StaticCast<DpdkNetDevice>(device);
    Simulator::Run();
    Simulator::Destroy();
    // dpdkNetDevice->StartSimulation();

    return 0;
}
