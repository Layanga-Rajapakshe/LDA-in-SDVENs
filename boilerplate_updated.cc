#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wave-module.h"
#include <cmath>
#include <cstdlib>  // for abs()

using namespace ns3;

// Required by NS_LOG_INFO / NS_LOG_WARN — must appear once per translation unit
NS_LOG_COMPONENT_DEFINE("SdVenSimulation");

// -----------------------------------------------------------------
// Constants (fix #1: define max/large/total_size that were missing)
// -----------------------------------------------------------------
#define MAX_NODES   256
#define LARGE_VAL   999999
#define MAX_ATTACKS 100

// -----------------------------------------------------------------
// Global declarations for the SD-VEN components
// -----------------------------------------------------------------
NodeContainer controller_Node, RSU_Nodes, Vehicle_Nodes;
NodeContainer dsrc_Nodes;                        // fix #7: will be populated in main()
NetDeviceContainer rsuDevices, vehicleDevices;
NetDeviceContainer wifidevices;                  // fix #8: assigned from wifi.Install()
Ipv4InterfaceContainer rsuInterfaces, vehicleInterfaces;
ApplicationContainer apps, RSU_apps;

uint32_t N_Vehicles = 3;
uint32_t N_RSUs     = 2;
std::vector<uint32_t> vehicle_to_rsu_map;

uint32_t lldp_i2i_pkt_size = 64;
uint32_t lldp_i2v_pkt_size = 64;

int attack_number = 0;
int attacker_node_id[MAX_ATTACKS] = {0};  // fix #1: sized with defined constant

// Routing algorithm selector (fix #5: was undefined)
int routing_algorithm = 0;

// Adjacency matrix placeholder (fix #4: was undefined)
double adjacencyMatrix[MAX_NODES][MAX_NODES] = {{0.0}};

// Empty neighbour-set array (fix #1: max → MAX_NODES, large → LARGE_VAL)
uint32_t empty_neighborset[MAX_NODES];

uint32_t total_size = 0;           // fix #2: define total_size
double   sum_of_nodeids = 0.0;

// -----------------------------------------------------------------
// Helper: unit step  u(x) = 1 if x >= 0, else 0  (fix #4)
// -----------------------------------------------------------------
static inline int unit_step(double x, int /*unused*/)
{
    return (x >= 0.0) ? 1 : 0;
}

// -----------------------------------------------------------------
void initialize_empty()
{
    for (uint32_t i = 0; i < MAX_NODES; i++)
        empty_neighborset[i] = LARGE_VAL;
}

void nodeid_sum()
{
    sum_of_nodeids = 0.0;
    // total_size is set in main() before this runs
    for (uint32_t i = 2; i < total_size + 2; i++)
        sum_of_nodeids += i;
}

// -----------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------
void PhaseI_I2IDiscovery();
void PhaseII_I2VDiscovery();
void PhaseIII_V2VDiscovery();

void arch2_assign_vehicles_to_rsus();
void arch2_i2i_controller_send_packetout();
void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index);
void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index);
void arch2_i2v_vehicle_respond(uint32_t vehicle_index);
void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index);
void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index);
void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index);

// Low-level send helpers
void p2p_data_broadcast(Ptr<Application> app, Ptr<Node> node);
void centralized_dsrc_data_broadcast(Ptr<NetDevice> dev, Ptr<Node> node,
                                     uint32_t idx, int channel);

// fix #11: single, consistent signature used everywhere
void centralized_dsrc_data_unicast(Ptr<Node> source_node,
                                   uint32_t  node_index,
                                   uint32_t  destination,
                                   int       channel);

// Stubs that were referenced in the original centralized_dsrc_data_unicast
void encrypt_dsrc_data_unicast(uint32_t src, uint32_t dst, int channel);
void send_dsrc_data_unicast(Ptr<Node> node, uint32_t src,
                            uint32_t dst, int channel);

// =================================================================
// main()
// =================================================================
int main(int argc, char *argv[])
{
    double simTime = 20.0;

    CommandLine cmd;
    cmd.AddValue("N_Vehicles", "Number of vehicles", N_Vehicles);
    cmd.AddValue("N_RSUs",     "Number of RSUs",     N_RSUs);
    cmd.Parse(argc, argv);

    // fix #2: set total_size before calling nodeid_sum()
    total_size = N_Vehicles + N_RSUs + 1; // +1 for controller

    initialize_empty();
    nodeid_sum();

    // ------------------------------------------------------------------
    // Node creation
    // ------------------------------------------------------------------
    controller_Node.Create(1);
    Vehicle_Nodes.Create(N_Vehicles);

    // fix #3: N_Vehicles (not N_vehicles — case corrected)
    if (N_Vehicles > 0)
        RSU_Nodes.Create(N_RSUs);

    // fix #7: populate dsrc_Nodes (RSUs + Vehicles share the wireless medium)
    dsrc_Nodes.Add(Vehicle_Nodes);
    dsrc_Nodes.Add(RSU_Nodes);

    // ------------------------------------------------------------------
    // Wired backbone (CSMA)
    // ------------------------------------------------------------------
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1000Mbps"));
    csma.SetChannelAttribute("Delay",    TimeValue(MicroSeconds(10)));

    NodeContainer csma_nodes;
    csma_nodes.Add(controller_Node);
    csma_nodes.Add(RSU_Nodes);
    NetDeviceContainer csmaDevices = csma.Install(csma_nodes);

    // ------------------------------------------------------------------
    // Wireless (WAVE / DSRC)
    // ------------------------------------------------------------------
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    Wifi80211pHelper wifi = Wifi80211pHelper::Default();
    NqosWaveMacHelper mac = NqosWaveMacHelper::Default();

    // fix #12: Vehicles first, then RSUs — indices 0..(N_Vehicles-1) are vehicles,
    //          indices N_Vehicles..(N_Vehicles+N_RSUs-1) are RSUs.
    //          arch2 functions use (N_Vehicles + rsu_index) to reach RSU devices.
    NodeContainer wirelessNodes;
    wirelessNodes.Add(Vehicle_Nodes);   // indices [0 .. N_Vehicles-1]
    wirelessNodes.Add(RSU_Nodes);       // indices [N_Vehicles .. N_Vehicles+N_RSUs-1]

    // fix #8: assign to the global wifidevices, not a local variable
    wifidevices = wifi.Install(phy, mac, wirelessNodes);

    // ------------------------------------------------------------------
    // Protocol stack
    // fix #14: install stack only once per node group; no overlap
    // ------------------------------------------------------------------
    InternetStackHelper stack;
    stack.Install(controller_Node);
    stack.Install(RSU_Nodes);
    stack.Install(Vehicle_Nodes);

    // ------------------------------------------------------------------
    // IP addressing
    // ------------------------------------------------------------------
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(csmaDevices);

    address.SetBase("192.168.1.0", "255.255.255.0");
    address.Assign(wifidevices);

    // ------------------------------------------------------------------
    // Mobility — set explicit initial positions (fix #15)
    // ------------------------------------------------------------------
    MobilityHelper mobility;

    // RSUs: fixed positions along a road
    Ptr<ListPositionAllocator> rsuPos = CreateObject<ListPositionAllocator>();
    
    for (uint32_t r = 0; r < N_RSUs; r++)
        rsuPos->Add(Vector(r * 500.0, 0.0, 0.0));   // 500 m apart

    mobility.SetPositionAllocator(rsuPos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(RSU_Nodes);

    // Vehicles: moving along the road
    Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
    for (uint32_t v = 0; v < N_Vehicles; v++)
        vehPos->Add(Vector(v * 100.0, 5.0, 0.0));   // 100 m apart, offset lane

    mobility.SetPositionAllocator(vehPos);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(Vehicle_Nodes);

    // Give each vehicle a speed of 15 m/s in the +x direction
    for (uint32_t v = 0; v < N_Vehicles; v++)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            Vehicle_Nodes.Get(v)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(15.0, 0.0, 0.0));
    }

    // ------------------------------------------------------------------
    // fix #10: assign vehicles to RSUs before any Phase function runs
    // ------------------------------------------------------------------
    arch2_assign_vehicles_to_rsus();

    // ------------------------------------------------------------------
    // Scheduling
    // ------------------------------------------------------------------
    Simulator::Schedule(Seconds(1.0), &PhaseI_I2IDiscovery);
    Simulator::Schedule(Seconds(2.0), &PhaseII_I2VDiscovery);

    for (double t = 3.0; t < simTime; t += 2.0)
        Simulator::Schedule(Seconds(t), &PhaseIII_V2VDiscovery);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}

// =================================================================
// Phase I — I2I Discovery
// =================================================================
void PhaseI_I2IDiscovery()
{
    NS_LOG_UNCOND("Phase I: Controller initiating LLDP Packet-Out for I2I Discovery...");
    arch2_i2i_controller_send_packetout();

    for (uint32_t r = 0; r < N_RSUs; r++)
        Simulator::Schedule(Seconds(0.05), &arch2_i2i_rsu_forward_to_neighbors, r);
}

// =================================================================
// Phase II — I2V Discovery
// =================================================================
void PhaseII_I2VDiscovery()
{
    NS_LOG_UNCOND("Phase II: RSU encapsulating LLDP for I2V Discovery...");
    for (uint32_t r = 0; r < N_RSUs; r++)
    {
        Simulator::Schedule(Seconds(0.0),  &arch2_i2v_rsu_encapsulate_and_forward, r);
        // Each vehicle served by this RSU responds after a small delay
        for (uint32_t v = 0; v < N_Vehicles; v++)
        {
            if (vehicle_to_rsu_map[v] == r)
                Simulator::Schedule(Seconds(0.05), &arch2_i2v_vehicle_respond, v);
        }
    }
}

// =================================================================
// Phase III — V2V Discovery
// =================================================================
void PhaseIII_V2VDiscovery()
{
    NS_LOG_UNCOND("Phase III: Initiating Cooperative V2V Discovery Rebroadcast...");
    for (uint32_t r = 0; r < N_RSUs; r++)
        Simulator::Schedule(Seconds(0.0), &arch2_v2v_rsu_inject_lldp, r);

    for (uint32_t v = 0; v < N_Vehicles; v++)
    {
        Simulator::Schedule(Seconds(0.05), &arch2_v2v_vehicle_rebroadcast, v);
        Simulator::Schedule(Seconds(0.10), &arch2_v2v_neighbor_confirm, v);
    }
}

// =================================================================
// Architecture 2 helpers
// =================================================================
void arch2_assign_vehicles_to_rsus()
{
    vehicle_to_rsu_map.resize(N_Vehicles, 0);
    if (N_RSUs == 0) return;

    for (uint32_t v = 0; v < Vehicle_Nodes.GetN(); v++)
    {
        Ptr<MobilityModel> vMob = Vehicle_Nodes.Get(v)->GetObject<MobilityModel>();
        Vector vPos = vMob->GetPosition();
        double   minDist = 1e18;
        uint32_t bestRSU = 0;

        for (uint32_t r = 0; r < RSU_Nodes.GetN(); r++)
        {
            Ptr<MobilityModel> rMob = RSU_Nodes.Get(r)->GetObject<MobilityModel>();
            Vector rPos = rMob->GetPosition();
            double dist = std::sqrt(std::pow(vPos.x - rPos.x, 2) +
                                    std::pow(vPos.y - rPos.y, 2));
            if (dist < minDist) { minDist = dist; bestRSU = r; }
        }
        vehicle_to_rsu_map[v] = bestRSU;
    }
    NS_LOG_INFO("[ARCH2-INIT] Vehicle-to-RSU assignment complete");
}

void arch2_i2i_controller_send_packetout()
{
    NS_LOG_INFO("[I2I-1] t=" << Simulator::Now().GetSeconds()
                << "s Controller → all RSUs: LLDP Packet-Out ("
                << lldp_i2i_pkt_size << "B) via CSMA");

    // apps[0] = controller application (populated externally or via stub)
    if (apps.GetN() > 0)
        p2p_data_broadcast(apps.Get(0), controller_Node.Get(0));
    else
        NS_LOG_WARN("[I2I-1] Controller app container is empty — skipping send");
}

void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index)
{
    NS_LOG_INFO("[I2I-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " → neighbor RSUs: LLDP broadcast via CSMA");

    if (RSU_apps.GetN() > rsu_index)
        p2p_data_broadcast(RSU_apps.Get(rsu_index), RSU_Nodes.Get(rsu_index));
    else
        NS_LOG_WARN("[I2I-2] RSU app container too small for index " << rsu_index);
}

void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index)
{
    NS_LOG_INFO("[I2V-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " → Vehicles: encapsulated LLDP ("
                << lldp_i2v_pkt_size << "B) via DSRC ch178");

    // fix #12: RSU device index = N_Vehicles + rsu_index (vehicles are first)
    centralized_dsrc_data_broadcast(
        wifidevices.Get(N_Vehicles + rsu_index),
        RSU_Nodes.Get(rsu_index),
        N_Vehicles + rsu_index,
        0);
}

void arch2_i2v_vehicle_respond(uint32_t vehicle_index)
{
    NS_LOG_INFO("[I2V-3] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index
                << " → RSU " << vehicle_to_rsu_map[vehicle_index]
                << ": LLDP response via DSRC");

    uint32_t rsu_idx = vehicle_to_rsu_map[vehicle_index];

    // fix #11: unified signature (Ptr<Node>, uint32_t, uint32_t, int)
    centralized_dsrc_data_unicast(
        dsrc_Nodes.Get(vehicle_index),
        vehicle_index,
        N_Vehicles + rsu_idx,
        0);
}

void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index)
{
    NS_LOG_INFO("[V2V-1] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " injecting LLDP for V2V discovery via DSRC");

    centralized_dsrc_data_broadcast(
        wifidevices.Get(N_Vehicles + rsu_index),
        RSU_Nodes.Get(rsu_index),
        N_Vehicles + rsu_index,
        1);
}

void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index)
{
    bool lldp_is_malicious = false;
    for (int a = 0; a < attack_number; a++)
    {
        if (attacker_node_id[a] ==
                static_cast<int>(N_Vehicles + vehicle_to_rsu_map[vehicle_index]))
        {
            lldp_is_malicious = true;
            break;
        }
    }

    // Uncomment to enable FL-GAN filtering:
    // if (lldp_is_malicious) {
    //     NS_LOG_INFO("[FL-GAN] Vehicle " << vehicle_index << " dropped malicious LLDP");
    //     return;
    // }
    (void)lldp_is_malicious; // suppress unused-variable warning while filtering is off

    NS_LOG_INFO("[V2V-2] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index
                << " rebroadcasting LLDP to neighbors via DSRC");

    centralized_dsrc_data_broadcast(
        wifidevices.Get(vehicle_index),
        dsrc_Nodes.Get(vehicle_index),
        vehicle_index,
        1);
}

void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index)
{
    NS_LOG_INFO("[V2V-3] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << neighbor_vehicle_index
                << " (neighbor) → RSU "
                << vehicle_to_rsu_map[neighbor_vehicle_index]
                << ": V2V confirmation");

    uint32_t rsu_idx = vehicle_to_rsu_map[neighbor_vehicle_index];

    centralized_dsrc_data_unicast(
        dsrc_Nodes.Get(neighbor_vehicle_index),
        neighbor_vehicle_index,
        N_Vehicles + rsu_idx,
        1);
}

// =================================================================
// centralized_dsrc_data_unicast  (fix #11: single definition)
// =================================================================
void centralized_dsrc_data_unicast(Ptr<Node> source_node,
                                   uint32_t  node_index,
                                   uint32_t  destination,
                                   int       channel)
{
    // Bounds check on adjacency matrix
    if (node_index >= MAX_NODES || destination >= MAX_NODES)
    {
        NS_LOG_WARN("centralized_dsrc_data_unicast: index out of bounds");
        return;
    }

    uint32_t link_may_exist;
    if (attack_number == 1 || attack_number == 6)
        link_may_exist = static_cast<uint32_t>(
            std::abs(unit_step(400.0 - adjacencyMatrix[node_index][destination], 0)));
    else
        link_may_exist = static_cast<uint32_t>(
            std::abs(unit_step(350.0 - adjacencyMatrix[node_index][destination], 0)));

    if (routing_algorithm == 4)
    {
        if (link_may_exist == 1)
        {
            Simulator::Schedule(Seconds(0.0),
                &encrypt_dsrc_data_unicast, node_index, destination, channel);
            Simulator::Schedule(Seconds(0.0002),
                &send_dsrc_data_unicast, source_node, node_index, destination, channel);
        }
    }
    else
    {
        Simulator::Schedule(Seconds(0.0002),
            &send_dsrc_data_unicast, source_node, node_index, destination, channel);
    }
}

// =================================================================
// Low-level stubs (replace with real implementations from LDA.cc)
// =================================================================
// void p2p_data_broadcast(Ptr<Application> /*app*/, Ptr<Node> /*node*/)
// {
//     // TODO: implement wired CSMA broadcast using UdpClientHelper / raw sockets
//     NS_LOG_INFO("[STUB] p2p_data_broadcast called");
// }

// void centralized_dsrc_data_broadcast(Ptr<NetDevice> /*dev*/, Ptr<Node> /*node*/,
//                                      uint32_t /*idx*/, int /*channel*/)
// {
//     // TODO: implement 802.11p broadcast using WaveNetDevice or raw WAVE socket
//     NS_LOG_INFO("[STUB] centralized_dsrc_data_broadcast called");
// }

// void encrypt_dsrc_data_unicast(uint32_t src, uint32_t dst, int /*channel*/)
// {
//     // TODO: apply encryption / signing to the outgoing LLDP frame
//     NS_LOG_INFO("[STUB] encrypt_dsrc_data_unicast " << src << " → " << dst);
// }

// void send_dsrc_data_unicast(Ptr<Node> /*node*/, uint32_t src,
//                             uint32_t dst, int /*channel*/)
// {
//     // TODO: actually transmit the unicast DSRC packet
//     NS_LOG_INFO("[STUB] send_dsrc_data_unicast " << src << " → " << dst);
// }