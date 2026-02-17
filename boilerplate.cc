#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

// Global declarations for the SD-VEN components
NodeContainer controller_Node, RSU_Nodes, Vehicle_Nodes;
NetDeviceContainer rsuDevices, vehicleDevices;
Ipv4InterfaceContainer rsuInterfaces, vehicleInterfaces;

// Function Prototypes for the 3-Phase Discovery
void PhaseI_I2IDiscovery();
void PhaseII_I2VDiscovery();
void PhaseIII_V2VDiscovery();

int main(int argc, char *argv[])
{
    // --- Initial Boilerplate and Configuration ---
    uint32_t N_Vehicles = 3;
    uint32_t N_RSUs = 2;
    double simTime = 20.0;
    
    CommandLine cmd;
    cmd.AddValue("N_Vehicles", "Number of vehicles", N_Vehicles);
    cmd.AddValue("N_RSUs", "Number of RSUs", N_RSUs);
    cmd.Parse(argc, argv);

    // Node Creation
    controller_Node.Create(1);
    RSU_Nodes.Create(N_RSUs);
    Vehicle_Nodes.Create(N_Vehicles);

    // --- Infrastructure Setup (Wired Backbone) ---
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1000Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
    
    NodeContainer backboneNodes;
    backboneNodes.Add(controller_Node);
    backboneNodes.Add(RSU_Nodes);
    NetDeviceContainer backboneDevices = csma.Install(backboneNodes);

    // --- Wireless Setup (WAVE/DSRC for V2V/I2V) ---
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    
    Wifi80211pHelper wifi = Wifi80211pHelper::Default();
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    
    NodeContainer wirelessNodes;
    wirelessNodes.Add(RSU_Nodes);
    wirelessNodes.Add(Vehicle_Nodes);
    NetDeviceContainer wirelessDevices = wifi.Install(phy, mac, wirelessNodes);

    // Protocol Stack
    InternetStackHelper stack;
    stack.Install(controller_Node);
    stack.Install(RSU_Nodes);
    stack.Install(Vehicle_Nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(backboneDevices);
    address.SetBase("192.168.1.0", "255.255.255.0");
    address.Assign(wirelessDevices);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(RSU_Nodes);
    mobility.Install(Vehicle_Nodes);

    // --- Custom Hybrid LLDP Scheduling ---
    
    // Phase I: Bootstrapping - I2I Discovery (Scheduled at start)
    Simulator::Schedule(Seconds(1.0), &PhaseI_I2IDiscovery);

    // Phase II: Discovery transitions to I2V (Scheduled after I2I stabilizes)
    Simulator::Schedule(Seconds(2.0), &PhaseII_I2VDiscovery);

    // Phase III: Highly dynamic V2V Discovery (Periodic updates)
    for (double t = 3.0; t < simTime; t += 2.0)
    {
        Simulator::Schedule(Seconds(t), &PhaseIII_V2VDiscovery);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}

// --- Phase I: Infrastructure-to-Infrastructure (I2I) Discovery ---
void PhaseI_I2IDiscovery()
{
    NS_LOG_UNCOND("Phase I: Controller initiating LLDP Packet-Out for I2I Discovery...");
    // Controller sends LLDP probe to RSUs via wired backbone
    // In ns-3, this is simulated by sending a custom packet from Controller to RSU IP
    // Logic: If Packet-In returns from neighbor, link is validated
}

// --- Phase II: Infrastructure-to-Vehicle (I2V) Discovery ---
void PhaseII_I2VDiscovery()
{
    NS_LOG_UNCOND("Phase II: RSU encapsulating LLDP for I2V Discovery...");
    // RSU receives Packet-Out from Controller
    // RSU encapsulates and broadcasts wirelessly to Vehicles
    // Vehicle extracts and returns LLDP to RSU immediately
}

// --- Phase III: Vehicle-to-Vehicle (V2V) Discovery ---
void PhaseIII_V2VDiscovery()
{
    NS_LOG_UNCOND("Phase III: Initiating Cooperative V2V Discovery Rebroadcast...");
    // RSU injects LLDP into wireless medium
    // Target Vehicle performs local cooperative rebroadcast
    // Neighboring Vehicles overhear and report confirmation back to RSU
}