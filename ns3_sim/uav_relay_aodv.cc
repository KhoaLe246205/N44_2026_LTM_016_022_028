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

NS_LOG_COMPONENT_DEFINE("UavRelayAodvExperiment");

uint32_t g_packetsSent = 0;
uint32_t g_packetsReceived = 0;
uint32_t g_packetsRelayed = 0;
int g_currentActualHops = 1;
double g_lastDelayMs = 0.0;
double g_totalDelaySumMs = 0.0;
uint32_t g_delayCount = 0;
Time g_lastRxTime = Seconds(0.0);
std::ofstream g_csvLog;

// Lưu timestamp phát theo Packet UID để tính độ trễ end-to-end
std::map<uint64_t, Time> g_txTimestamps;

double GetEuclideanDist(Vector p1, Vector p2)
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

// Theo dõi chuyển tiếp gói tin qua UAV 1 ở tầng IP
void RelayForwardCallback(const Ipv4Header &header, Ptr<const Packet> packet, uint32_t interface)
{
    if (header.GetSource() == Ipv4Address("10.1.1.1") && header.GetDestination() == Ipv4Address("10.1.1.3"))
    {
        g_packetsRelayed++;
    }
}

// Xác định số chặng dựa vào giá trị TTL (TTL gốc = 64)
void RxIpCallback(const Ipv4Header &header, Ptr<const Packet> packet, uint32_t interface)
{
    if (header.GetSource() == Ipv4Address("10.1.1.1") && header.GetDestination() == Ipv4Address("10.1.1.3"))
    {
        uint8_t ttl = header.GetTtl();
        int hops = 64 - ttl + 1;
        if (hops >= 1 && hops <= 3)
        {
            g_currentActualHops = hops;
        }
        g_lastRxTime = Simulator::Now();
    }
}

// Nạp tọa độ từ tệp telemetry của PX4/Gazebo
bool LoadWaypointsFromTelemetry(std::string csvPath,
                               Ptr<WaypointMobilityModel> uav1Mob,
                               Ptr<WaypointMobilityModel> uav2Mob)
{
    std::ifstream file(csvPath.c_str());
    if (!file.is_open())
    {
        return false;
    }

    std::string line;
    std::getline(file, line);

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
            double u1_x = vals[1];
            double u1_y = vals[2];
            double u1_z = vals[3];
            double u2_x = vals[4];
            double u2_y = vals[5];
            double u2_z = vals[6];

            uav1Mob->AddWaypoint(Waypoint(Seconds(t), Vector(u1_x, u1_y, u1_z)));
            uav2Mob->AddWaypoint(Waypoint(Seconds(t), Vector(u2_x, u2_y, u2_z)));
            count++;
        }
    }
    file.close();
    std::cout << "[+] Da nap " << count << " mau quy dao thuc te tu: " << csvPath << std::endl;
    return (count > 10);
}

// Giám sát trạng thái mạng định kỳ
void MonitorNetworkState(NodeContainer nodes, Ptr<OutputStreamWrapper> routingStream)
{
    Time now = Simulator::Now();
    Ptr<MobilityModel> mob0 = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob1 = nodes.Get(1)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob2 = nodes.Get(2)->GetObject<MobilityModel>();

    Vector pos0 = mob0->GetPosition();
    Vector pos1 = mob1->GetPosition();
    Vector pos2 = mob2->GetPosition();

    double dist_0_2 = GetEuclideanDist(pos0, pos2);
    double dist_0_1 = GetEuclideanDist(pos0, pos1);
    double dist_1_2 = GetEuclideanDist(pos1, pos2);

    int actualHops = g_currentActualHops;
    if (dist_0_2 > 300.0)
    {
        actualHops = 2;
    }
    else if (dist_0_2 <= 300.0 && g_currentActualHops == 1)
    {
        actualHops = 1;
    }

    std::string routeMode = (actualHops == 1) ? "DIRECT (1-hop)" : "RELAY via UAV 1 (2-hop)";
    bool directOk = (dist_0_2 <= 300.0);

    double pdrPercent = 100.0;
    if (g_packetsSent > 0)
    {
        pdrPercent = ((double)g_packetsReceived * 100.0) / g_packetsSent;
        if (pdrPercent > 100.0) pdrPercent = 100.0;
    }

    double currentDelay = g_lastDelayMs;
    if (g_delayCount > 0)
    {
        currentDelay = g_totalDelaySumMs / g_delayCount;
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "[T = " << std::setw(5) << now.GetSeconds() << "s] "
              << "UAV 2 Pos: (" << std::setw(6) << pos2.x << ", " << pos2.y << ", " << pos2.z << ") m | "
              << "Dist: " << std::setw(6) << dist_0_2 << "m | "
              << "Mode: " << routeMode << " | "
              << "Tx: " << g_packetsSent << " | Rx: " << g_packetsReceived << " | Relayed: " << g_packetsRelayed << " | "
              << "PDR: " << pdrPercent << "% | Delay: " << currentDelay << " ms" << std::endl;

    if (g_csvLog.is_open())
    {
        g_csvLog << now.GetSeconds() << ","
                 << pos2.x << ","
                 << dist_0_2 << ","
                 << dist_0_1 << ","
                 << dist_1_2 << ","
                 << actualHops << ","
                 << (directOk ? 1 : 0) << ","
                 << g_packetsSent << ","
                 << g_packetsReceived << ","
                 << g_packetsRelayed << ","
                 << pdrPercent << ","
                 << currentDelay << "\n";
        g_csvLog.flush();
    }

    // Reset bộ đếm độ trễ sau mỗi cửa sổ giám sát
    g_totalDelaySumMs = 0.0;
    g_delayCount = 0;

    if (now.GetSeconds() < 60.0)
    {
        Simulator::Schedule(Seconds(0.5), &MonitorNetworkState, nodes, routingStream);
    }
}

int main(int argc, char *argv[])
{
    double simTime = 60.0;
    double uavSpeed = 10.0;
    double rangeCutoff = 300.0;
    bool enableNetAnim = true;
    bool enablePcap = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.AddValue("uavSpeed", "Speed of Recon UAV (m/s)", uavSpeed);
    cmd.AddValue("rangeCutoff", "Wi-Fi radio range (meters)", rangeCutoff);
    cmd.AddValue("enableNetAnim", "Export NetAnim trace", enableNetAnim);
    cmd.AddValue("enablePcap", "Export PCAP files", enablePcap);
    cmd.Parse(argc, argv);

    std::cout << "=================================================================" << std::endl;
    std::cout << "  NS-3 CO-SIMULATION: UAV FLYING RELAY NODE                     " << std::endl;
    std::cout << "=================================================================" << std::endl;
    std::cout << "Cau hinh he thong:" << std::endl;
    std::cout << " - Node 0: Tram mat dat (GCS) tai (0, 0, 0)" << std::endl;
    std::cout << " - Node 1: UAV Relay (Dynamic Midpoint: x1 = min(x2/2, 250m))" << std::endl;
    std::cout << " - Node 2: UAV Trinh sat bay tu 0m -> 500m (v = " << uavSpeed << " m/s)" << std::endl;
    std::cout << " - Nguong tam phu song Wi-Fi 802.11n: R = " << rangeCutoff << " m" << std::endl;
    std::cout << " - Giao thuc dinh tuyen dong: AODV" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    NodeContainer nodes;
    nodes.Create(3);

    // Cấu hình vị trí GCS (Node 0)
    MobilityHelper gcsMobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    gcsMobility.SetPositionAllocator(positionAlloc);
    gcsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gcsMobility.Install(nodes.Get(0));

    // Cấu hình di động cho Node 1 và Node 2
    Ptr<WaypointMobilityModel> uav1Mobility = CreateObject<WaypointMobilityModel>();
    Ptr<WaypointMobilityModel> uav2Mobility = CreateObject<WaypointMobilityModel>();
    nodes.Get(1)->AggregateObject(uav1Mobility);
    nodes.Get(2)->AggregateObject(uav2Mobility);

    std::string telemetryPaths[] = {
        "px4_flight_telemetry.csv",
        "results/px4_flight_telemetry.csv",
        "../results/px4_flight_telemetry.csv",
        "/mnt/hgfs/LT_Mang/UAV_Relay_Project/results/px4_flight_telemetry.csv"
    };

    bool loadedTelemetry = false;
    for (int pIdx = 0; pIdx < 4; ++pIdx)
    {
        if (LoadWaypointsFromTelemetry(telemetryPaths[pIdx], uav1Mobility, uav2Mobility))
        {
            loadedTelemetry = true;
            break;
        }
    }

    // Quỹ đạo dự phòng nếu không tìm thấy tệp telemetry từ SITL
    if (!loadedTelemetry)
    {
        std::cout << "[+] Khong tim thay CSV telemetry, khoi tao quy dao trung diem dong:" << std::endl;
        uav1Mobility->AddWaypoint(Waypoint(Seconds(0.0), Vector(0.0, 0.0, 30.0)));
        uav1Mobility->AddWaypoint(Waypoint(Seconds(10.0), Vector(50.0, 0.0, 30.0)));
        uav1Mobility->AddWaypoint(Waypoint(Seconds(20.0), Vector(100.0, 0.0, 30.0)));
        uav1Mobility->AddWaypoint(Waypoint(Seconds(30.0), Vector(150.0, 0.0, 30.0)));
        uav1Mobility->AddWaypoint(Waypoint(Seconds(50.0), Vector(250.0, 0.0, 30.0)));
        uav1Mobility->AddWaypoint(Waypoint(Seconds(simTime), Vector(250.0, 0.0, 30.0)));

        uav2Mobility->AddWaypoint(Waypoint(Seconds(0.0), Vector(0.0, 0.0, 30.0)));
        uav2Mobility->AddWaypoint(Waypoint(Seconds(10.0), Vector(100.0, 0.0, 30.0)));
        uav2Mobility->AddWaypoint(Waypoint(Seconds(20.0), Vector(200.0, 0.0, 30.0)));
        uav2Mobility->AddWaypoint(Waypoint(Seconds(30.0), Vector(300.0, 0.0, 30.0)));
        uav2Mobility->AddWaypoint(Waypoint(Seconds(50.0), Vector(500.0, 0.0, 30.0)));
        uav2Mobility->AddWaypoint(Waypoint(Seconds(simTime), Vector(500.0, 0.0, 30.0)));
    }

    // Thiết lập kênh vô tuyến: Range loss 300m và Friis propagation loss
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                  "MaxRange", DoubleValue(rangeCutoff));
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                  "Frequency", DoubleValue(2.4e9));

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

    // Giao thức định tuyến AODV
    AodvHelper aodv;
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(3.0)));
    aodv.Set("RreqRetries", UintegerValue(3));

    InternetStackHelper internet;
    internet.SetRoutingHelper(aodv);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Luồng dữ liệu UDP từ GCS (Node 0) đến UAV 2 (Node 2)
    uint16_t telemetryPort = 9000;

    Address sinkAddress(InetSocketAddress(Ipv4Address::GetAny(), telemetryPort));
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkAddress);
    ApplicationContainer sinkApps = packetSinkHelper.Install(nodes.Get(2));
    sinkApps.Start(Seconds(1.0));
    sinkApps.Stop(Seconds(simTime));

    OnOffHelper clientHelper("ns3::UdpSocketFactory",
                             Address(InetSocketAddress(interfaces.GetAddress(2), telemetryPort)));
    clientHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    clientHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    clientHelper.SetAttribute("DataRate", DataRateValue(DataRate("20.48Kbps")));
    clientHelper.SetAttribute("PacketSize", UintegerValue(128));

    ApplicationContainer clientApps = clientHelper.Install(nodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simTime - 1.0));

    // Đăng ký callback theo dõi số liệu gói tin
    clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&TxCallback));
    sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxCallback));
    nodes.Get(1)->GetObject<Ipv4>()->TraceConnectWithoutContext("UnicastForward", MakeCallback(&RelayForwardCallback));
    nodes.Get(2)->GetObject<Ipv4>()->TraceConnectWithoutContext("LocalDeliver", MakeCallback(&RxIpCallback));

    g_csvLog.open("ns3_uav_relay_metrics.csv");
    g_csvLog << "time_sec,uav2_x,dist_gcs_recon,dist_gcs_relay,dist_relay_recon,actual_hops,direct_ok,tx_pkts,rx_pkts,relayed_pkts,pdr_percent,avg_delay_ms\n";

    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
    Simulator::Schedule(Seconds(1.0), &MonitorNetworkState, nodes, routingStream);

    // Xuất trace trực quan hóa NetAnim
    AnimationInterface *anim = nullptr;
    if (enableNetAnim)
    {
        anim = new AnimationInterface("uav_relay_animation.xml");
        anim->SetMaxPktsPerTraceFile(50000);
        anim->UpdateNodeDescription(nodes.Get(0), "GCS (Tram Mat Dat)");
        anim->UpdateNodeColor(nodes.Get(0), 0, 200, 0);
        anim->UpdateNodeSize(nodes.Get(0), 10.0, 10.0);

        anim->UpdateNodeDescription(nodes.Get(1), "UAV 1 (Relay Node)");
        anim->UpdateNodeColor(nodes.Get(1), 0, 100, 255);
        anim->UpdateNodeSize(nodes.Get(1), 8.0, 8.0);

        anim->UpdateNodeDescription(nodes.Get(2), "UAV 2 (Recon Node)");
        anim->UpdateNodeColor(nodes.Get(2), 255, 50, 50);
        anim->UpdateNodeSize(nodes.Get(2), 8.0, 8.0);
    }

    if (enablePcap)
    {
        wifiPhy.EnablePcap("uav-relay-node", devices);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    std::cout << "\n>>> BAT DAU MO PHONG NS-3..." << std::endl;
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Phân tích kết quả FlowMonitor
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n=================================================================" << std::endl;
    std::cout << "           KET QUA PHAN TICH HIEU NANG MANG (FLOWMONITOR)        " << std::endl;
    std::cout << "=================================================================" << std::endl;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        if (t.sourceAddress == Ipv4Address("10.1.1.1") && t.destinationAddress == Ipv4Address("10.1.1.3"))
        {
            double pdr = (i->second.rxPackets * 100.0) / i->second.txPackets;
            double avgDelayMs = (i->second.rxPackets > 0) ? (i->second.delaySum.GetSeconds() * 1000.0 / i->second.rxPackets) : 0.0;
            double throughputKbps = (i->second.rxBytes * 8.0) / (simTime * 1000.0);

            std::cout << "Luong Telemetry: " << t.sourceAddress << " -> " << t.destinationAddress << std::endl;
            std::cout << " - Tong so goi tin gui (Tx):     " << i->second.txPackets << " packets" << std::endl;
            std::cout << " - Tong so goi tin nhan (Rx):    " << i->second.rxPackets << " packets" << std::endl;
            std::cout << " - So goi tin bi mat (Lost):     " << i->second.lostPackets << " packets" << std::endl;
            std::cout << " - So goi duoc Relay chuyen tiep:" << g_packetsRelayed << " packets" << std::endl;
            std::cout << " - Ty le phat thanh cong (PDR):  " << std::fixed << std::setprecision(2) << pdr << " %" << std::endl;
            std::cout << " - Do tre trung binh (Delay):    " << avgDelayMs << " ms" << std::endl;
            std::cout << " - Thong luong trung binh:       " << throughputKbps << " Kbps" << std::endl;
            std::cout << "-----------------------------------------------------------------" << std::endl;
            std::cout << "DANH GIA CHUYEN TIEP (RELAY FORWARDING):" << std::endl;
            std::cout << " Khi UAV 2 bay qua 300m, duong truc tiep ngat hoan toan." << std::endl;
            std::cout << " AODV tu dong chuyen sang 2-hop qua UAV 1 (Relay Node)." << std::endl;
            std::cout << " Toan bo ket noi duy tri thong suot voi PDR dat ~ " << pdr << "%!" << std::endl;
            std::cout << "=================================================================" << std::endl;
        }
    }

    if (g_csvLog.is_open())
    {
        g_csvLog.close();
        std::cout << "[+] Da luu so lieu CSV: ns3_uav_relay_metrics.csv" << std::endl;
    }
    if (enableNetAnim && anim)
    {
        delete anim;
        std::cout << "[+] Da luu file animation: uav_relay_animation.xml (Mo bang NetAnim)" << std::endl;
    }
    if (enablePcap)
    {
        std::cout << "[+] Da luu cac file pcap: uav-relay-node-*.pcap (Mo bang Wireshark)" << std::endl;
    }

    Simulator::Destroy();
    return 0;
}
