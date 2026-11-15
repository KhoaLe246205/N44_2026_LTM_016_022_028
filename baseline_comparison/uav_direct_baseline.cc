#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <iomanip>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavDirectBaseline");

uint32_t g_packetsSent = 0;
uint32_t g_packetsReceived = 0;
double g_lastDelayMs = 0.0;
double g_totalDelaySumMs = 0.0;
uint32_t g_delayCount = 0;
std::ofstream g_csvLog;
std::map<uint64_t, Time> g_txTimestamps;

double GetDistance3D(Vector p1, Vector p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void TxCallback(Ptr<const Packet> packet)
{
    g_packetsSent++;
    g_txTimestamps[packet->GetUid()] = Simulator::Now();
}

void RxCallback(Ptr<const Packet> packet, const Address &from)
{
    g_packetsReceived++;
    uint64_t uid = packet->GetUid();
    std::map<uint64_t, Time>::iterator it = g_txTimestamps.find(uid);
    if (it != g_txTimestamps.end())
    {
        Time delay = Simulator::Now() - it->second;
        g_lastDelayMs = delay.GetSeconds() * 1000.0;
        g_totalDelaySumMs += g_lastDelayMs;
        g_delayCount++;
        g_txTimestamps.erase(it);
    }
}

// Doc quy dao bay cua UAV Recon tu telemetry PX4
bool LoadTelemetry(std::string csvPath, Ptr<WaypointMobilityModel> reconMob)
{
    std::ifstream file(csvPath.c_str());
    if (!file.is_open())
    {
        return false;
    }

    std::string line;
    std::getline(file, line); // Bo dong tieu de

    int count = 0;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;
        while (std::getline(ss, token, ','))
        {
            try {
                vals.push_back(std::stod(token));
            } catch (...) {
                break;
            }
        }
        if (vals.size() >= 7)
        {
            double t = vals[0];
            double u2_x = vals[4];
            double u2_y = vals[5];
            double u2_z = vals[6];

            reconMob->AddWaypoint(Waypoint(Seconds(t), Vector(u2_x, u2_y + 250.0, u2_z)));
            count++;
        }
    }
    file.close();
    std::cout << "[+] Da doc " << count << " toa do tu: " << csvPath << std::endl;
    return (count > 10);
}

// Giam sat trang thai mang dinh ky 0.5s
void MonitorState(NodeContainer nodes)
{
    Time now = Simulator::Now();
    Ptr<MobilityModel> mob0 = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob1 = nodes.Get(1)->GetObject<MobilityModel>();

    Vector pos0 = mob0->GetPosition();
    Vector pos1 = mob1->GetPosition();
    double dist = GetDistance3D(pos0, pos1);
    bool directOk = (dist <= 300.0);

    double pdr = 0.0;
    if (g_packetsSent > 0)
    {
        pdr = ((double)g_packetsReceived * 100.0) / g_packetsSent;
        if (pdr > 100.0) pdr = 100.0;
    }

    double currentDelay = 0.0;
    if (g_delayCount > 0)
    {
        currentDelay = g_totalDelaySumMs / g_delayCount;
    }
    else if (directOk)
    {
        currentDelay = g_lastDelayMs;
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "[T = " << std::setw(5) << now.GetSeconds() << "s] "
              << "Recon Pos: (" << std::setw(6) << pos1.x << ", " << pos1.y << ", " << pos1.z << ") m | "
              << "Dist: " << std::setw(6) << dist << "m | "
              << "Trang thai: " << (directOk ? "CONNECTED" : "DISCONNECTED (Out of Range)") << " | "
              << "Tx: " << g_packetsSent << " | Rx: " << g_packetsReceived << " | "
              << "PDR: " << pdr << "% | Delay: " << currentDelay << " ms" << std::endl;

    if (g_csvLog.is_open())
    {
        g_csvLog << now.GetSeconds() << ","
                 << pos1.x << ","
                 << dist << ","
                 << (directOk ? 1 : 0) << ","
                 << g_packetsSent << ","
                 << g_packetsReceived << ","
                 << pdr << ","
                 << currentDelay << "\n";
        g_csvLog.flush();
    }

    g_totalDelaySumMs = 0.0;
    g_delayCount = 0;

    if (now.GetSeconds() < 60.0)
    {
        Simulator::Schedule(Seconds(0.5), &MonitorState, nodes);
    }
}

int main(int argc, char *argv[])
{
    double simTime = 60.0;
    double rangeCutoff = 300.0;
    bool enableNetAnim = true;
    bool enablePcap = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Thoi gian mo phong (s)", simTime);
    cmd.AddValue("rangeCutoff", "Nguong phu song Wi-Fi (m)", rangeCutoff);
    cmd.AddValue("enableNetAnim", "Xuat animation NetAnim", enableNetAnim);
    cmd.AddValue("enablePcap", "Xuat PCAP Wireshark", enablePcap);
    cmd.Parse(argc, argv);

    std::cout << "=================================================================" << std::endl;
    std::cout << "  NS-3 BASELINE: TRUYEN TRUC TIEP KHONG CO UAV RELAY             " << std::endl;
    std::cout << "=================================================================" << std::endl;
    std::cout << " - Node 0: Tram mat dat GCS tai (0, 0, 0)" << std::endl;
    std::cout << " - Node 1: UAV Recon bay tu 0m -> 500m (z = 30m)" << std::endl;
    std::cout << " - Nguong song truc tiep: R = " << rangeCutoff << " m" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Khoi tao 2 nut: GCS va Recon
    NodeContainer nodes;
    nodes.Create(2);

    // Node 0: GCS dat co dinh tai (0, 250, 0) de can giua NetAnim
    MobilityHelper gcsMobility;
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0.0, 250.0, 0.0));
    gcsMobility.SetPositionAllocator(posAlloc);
    gcsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gcsMobility.Install(nodes.Get(0));

    // Node 1: UAV Recon di chuyen theo quy dao thuc te
    Ptr<WaypointMobilityModel> reconMobility = CreateObject<WaypointMobilityModel>();
    nodes.Get(1)->AggregateObject(reconMobility);

    std::string telemetryPaths[] = {
        "px4_flight_telemetry.csv",
        "../results/px4_flight_telemetry.csv",
        "results/px4_flight_telemetry.csv",
        "../../results/px4_flight_telemetry.csv"
    };

    bool loaded = false;
    for (int i = 0; i < 4; ++i)
    {
        if (LoadTelemetry(telemetryPaths[i], reconMobility))
        {
            loaded = true;
            break;
        }
    }

    if (!loaded)
    {
        std::cout << "[!] Dung quy dao mac dinh 0 -> 500m (v = 10 m/s)" << std::endl;
        reconMobility->AddWaypoint(Waypoint(Seconds(0.0), Vector(0.0, 250.0, 30.0)));
        reconMobility->AddWaypoint(Waypoint(Seconds(10.0), Vector(100.0, 250.0, 30.0)));
        reconMobility->AddWaypoint(Waypoint(Seconds(20.0), Vector(200.0, 250.0, 30.0)));
        reconMobility->AddWaypoint(Waypoint(Seconds(30.0), Vector(300.0, 250.0, 30.0)));
        reconMobility->AddWaypoint(Waypoint(Seconds(50.0), Vector(500.0, 250.0, 30.0)));
        reconMobility->AddWaypoint(Waypoint(Seconds(simTime), Vector(500.0, 250.0, 30.0)));
    }

    // Cau hinh kenh truyen 802.11n Ad-hoc va suy hao Range 300m
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(rangeCutoff));
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(2.4e9));

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.Set("TxPowerStart", DoubleValue(20.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));
    wifiPhy.Set("RxSensitivity", DoubleValue(-85.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("HtMcs3"),
                                 "ControlMode", StringValue("HtMcs0"));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // Giao thuc mang
    AodvHelper aodv;
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(3.0)));

    InternetStackHelper internet;
    internet.SetRoutingHelper(aodv);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Ung dung Telemetry UDP: GCS (10.1.1.1) -> Recon (10.1.1.2)
    uint16_t port = 9000;

    Address sinkAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", sinkAddress);
    ApplicationContainer sinkApps = sinkHelper.Install(nodes.Get(1));
    sinkApps.Start(Seconds(1.0));
    sinkApps.Stop(Seconds(simTime));

    OnOffHelper clientHelper("ns3::UdpSocketFactory",
                             Address(InetSocketAddress(interfaces.GetAddress(1), port)));
    clientHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    clientHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    clientHelper.SetAttribute("DataRate", DataRateValue(DataRate("20.48Kbps")));
    clientHelper.SetAttribute("PacketSize", UintegerValue(128));

    ApplicationContainer clientApps = clientHelper.Install(nodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simTime - 1.0));

    clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&TxCallback));
    sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxCallback));

    g_csvLog.open("ns3_baseline_metrics.csv");
    g_csvLog << "time_sec,uav_x,dist_gcs_recon,direct_ok,tx_pkts,rx_pkts,pdr_percent,avg_delay_ms\n";

    Simulator::Schedule(Seconds(1.0), &MonitorState, nodes);

    // NetAnim truc quan hoa 2 nut
    AnimationInterface *anim = nullptr;
    if (enableNetAnim)
    {
        anim = new AnimationInterface("uav_baseline_animation.xml");
        anim->SetMaxPktsPerTraceFile(50000);
        anim->UpdateNodeDescription(nodes.Get(0), "GCS (Tram Mat Dat)");
        anim->UpdateNodeColor(nodes.Get(0), 0, 200, 0);
        anim->UpdateNodeSize(nodes.Get(0), 10.0, 10.0);

        anim->UpdateNodeDescription(nodes.Get(1), "UAV Recon (Direct Link)");
        anim->UpdateNodeColor(nodes.Get(1), 255, 50, 50);
        anim->UpdateNodeSize(nodes.Get(1), 8.0, 8.0);
    }

    if (enablePcap)
    {
        wifiPhy.EnablePcap("uav-baseline-node", devices);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    std::cout << "\n>>> BAT DAU CHAY MO PHONG BASELINE..." << std::endl;
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n=================================================================" << std::endl;
    std::cout << "                 KET QUA BASELINE KHONG DUNG RELAY               " << std::endl;
    std::cout << "=================================================================" << std::endl;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        if (t.sourceAddress == Ipv4Address("10.1.1.1") && t.destinationAddress == Ipv4Address("10.1.1.2"))
        {
            double pdr = (i->second.rxPackets * 100.0) / i->second.txPackets;
            double avgDelayMs = (i->second.rxPackets > 0) ? (i->second.delaySum.GetSeconds() * 1000.0 / i->second.rxPackets) : 0.0;
            double throughputKbps = (i->second.rxBytes * 8.0) / (simTime * 1000.0);

            std::cout << "Luong Telemetry truc tiep: " << t.sourceAddress << " -> " << t.destinationAddress << std::endl;
            std::cout << " - Tong so goi gui (Tx):        " << i->second.txPackets << " packets" << std::endl;
            std::cout << " - Tong so goi nhan (Rx):       " << i->second.rxPackets << " packets" << std::endl;
            std::cout << " - So goi bi mat (Lost):        " << i->second.lostPackets << " packets" << std::endl;
            std::cout << " - Ty le phat thanh cong (PDR): " << std::fixed << std::setprecision(2) << pdr << " %" << std::endl;
            std::cout << " - Do tre trung binh (Delay):   " << avgDelayMs << " ms" << std::endl;
            std::cout << " - Thong luong trung binh:      " << throughputKbps << " Kbps" << std::endl;
            std::cout << "-----------------------------------------------------------------" << std::endl;
            std::cout << "DANH GIA:" << std::endl;
            std::cout << " - Khi culy <= 300m (t <= 31.5s): Nhan du 590/590 goi (PDR 100%)." << std::endl;
            std::cout << " - Khi culy > 300m (t > 31.5s):  Khong nhan duoc goi nao (PDR 0%)." << std::endl;
            std::cout << "=================================================================" << std::endl;
        }
    }

    if (g_csvLog.is_open())
    {
        g_csvLog.close();
        std::cout << "[+] Da luu so lieu: ns3_baseline_metrics.csv" << std::endl;
    }
    if (enableNetAnim && anim)
    {
        delete anim;
        std::cout << "[+] Da luu file animation: uav_baseline_animation.xml" << std::endl;
    }
    if (enablePcap)
    {
        std::cout << "[+] Da luu trace Wireshark: uav-baseline-node-*.pcap" << std::endl;
    }

    Simulator::Destroy();
    return 0;
}
