#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wave-module.h"

using namespace ns3;

// Global declarations for the SD-VEN components
NodeContainer controller_Node, RSU_Nodes, Vehicle_Nodes;
NodeContainer dsrc_Nodes;
NetDeviceContainer rsuDevices, vehicleDevices, wifidevices;
Ipv4InterfaceContainer rsuInterfaces, vehicleInterfaces;
ApplicationContainer apps, RSU_apps;

uint32_t N_Vehicles = 3;
uint32_t N_RSUs = 2;
std::vector<uint32_t> vehicle_to_rsu_map;
uint32_t lldp_i2i_pkt_size = 64;
uint32_t lldp_i2v_pkt_size = 64;
uint32_t lldp_v2v_pkt_size   = 256;
double data_transmission_period = 1.0/data_transmission_frequency; 
const int total_size = 16;
int attack_number = 0;
int attacker_node_id[100];

uint32_t empty_neighborset[max]; 
double arch2_i2i_offset       = 0.002;   
double arch2_i2i_recv_offset  = 0.004;   
double arch2_i2v_offset       = 0.006;   
double arch2_i2v_recv_offset  = 0.008;   
double arch2_v2v_offset       = 0.010;   
double arch2_v2v_recv_offset  = 0.014;     
  

void initialize_empty()
{
	for (uint32_t i =0; i < max; i++)
	{
		empty_neighborset[i] = large;
	}
}

double sum_of_nodeids = 0;
void nodeid_sum()
{
	sum_of_nodeids = 0;
	for (uint32_t i=2;i<total_size+2;i++)
	{
		sum_of_nodeids = sum_of_nodeids + i;
	}
}

// Function Prototypes for the 3-Phase Discovery
void PhaseI_I2IDiscovery();
void PhaseII_I2VDiscovery();
void PhaseIII_V2VDiscovery();

// Architecture 2 Function Prototypes
void arch2_assign_vehicles_to_rsus();
void arch2_i2i_controller_send_packetout();
void arch2_i2v_controller_send_packetout(uint32_t rsu_index);
void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index);
void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index);
void arch2_i2v_vehicle_respond(uint32_t vehicle_index);
void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index);
void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index);
void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index);

// Placeholder function prototypes (implement based on LDA.cc)
void p2p_data_broadcast(Ptr<Application> app, Ptr<Node> node);
void centralized_dsrc_data_broadcast(Ptr<NetDevice> dev, Ptr<Node> node, uint32_t idx, int channel);
void centralized_dsrc_data_unicast(Ptr<Node> node, uint32_t src_idx, uint32_t dst_idx, int channel);

int main(int argc, char *argv[])
{
    initialize_empty(); // Initialize empty neighbor set values
    nodeid_sum(); // Calculate sum of node ids for optimization calculations

    // --- Initial Boilerplate and Configuration ---
    // N_Vehicles and N_RSUs are now global
    double simTime = 20.0;
    
    CommandLine cmd;
    cmd.AddValue("N_Vehicles", "Number of vehicles", N_Vehicles);
    cmd.AddValue("N_RSUs", "Number of RSUs", N_RSUs);
    // cmd.AddValue ("data_transmission_frequency", "data_transmission_frequency", data_transmission_frequency);
    // cmd.AddValue ("simTime", "simTime", simTime);
    // cmd.AddValue ("attack_number", "attack_number", attack_number);
    // cmd.AddValue ("attack_percentage", "attack_percentage", attack_percentage);
    cmd.Parse(argc, argv);

    // Node Creation
    controller_Node.Create(1);
    Vehicle_Nodes.Create(N_Vehicles);

    if(N_vehicles > 0)
    {
        RSU_Nodes.Create(N_RSUs);
    }

    // --- Infrastructure Setup (Wired Backbone) ---
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1000Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
    
    NodeContainer csma_nodes;

    // Ipv4AddressHelper address;
    // Ipv4InterfaceContainer csmaInterfaces;
    // NetDeviceContainer csmaDevices;
    // InternetStackHelper stack;

    csma_nodes.Add(controller_Node);
    csma_nodes.Add(RSU_Nodes);
    NetDeviceContainer csmaDevices = csma.Install(csma_nodes);

    // --- Wireless Setup (WAVE/DSRC for V2V/I2V) ---
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    
    Wifi80211pHelper wifi = Wifi80211pHelper::Default();
    NqosWaveMacHelper mac = NqosWaveMacHelper::Default();
    
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
    address.Assign(csmaDevices);
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
    if (architecture == 2)
{
  

    Simulator::Schedule(Seconds(1.0),   arch2_assign_vehicles_to_rsus);


    for (uint32_t i = 0; i < total_size + 2; i++)
    {
        uplink_last[i]   = 0.0;
        last_downlink[i] = 0.0;
    }

    // ── Per-round loop ──────────────────────────────────────────
    double t_start = 6.00;

    for (double t = t_start; t < simTime - 1; t += data_transmission_period)
    {
        // ── Standard resets ──
        // Simulator::Schedule(Seconds(t), arch2_reset_round_counters);
        // Simulator::Schedule(Seconds(t), reset_packet_timestamps);
        // Simulator::Schedule(Seconds(t), reset_confusion_matrix);
        // Simulator::Schedule(Seconds(t), update_mobility);
        // Simulator::Schedule(Seconds(t), generate_F_and_E);
        // Simulator::Schedule(Seconds(t), set_dsrc_initial_timestamp);
        // Simulator::Schedule(Seconds(t), set_ethernet_initial_timestamp);


        // PHASE I – I2I Discovery
        Simulator::Schedule(Seconds(t + arch2_i2i_offset),
            arch2_i2i_controller_send_packetout);

        for (uint32_t r = 0; r < N_RSUs; r++)
        {
            Simulator::Schedule(Seconds(t + arch2_i2i_offset + 0.0001*r),
                arch2_i2i_rsu_forward_to_neighbors, r);

        }

        for (uint32_t r = 0; r < N_RSUs; r++)
        {
            Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.0001*r),
                arch2_i2v_controller_send_packetout, r);

            Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.0002*r),
                arch2_i2v_rsu_encapsulate_and_forward, r);
        }

        for (uint32_t v = 0; v < N_Vehicles; v++)
        {
            Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.003 + 0.0001*v),
                arch2_i2v_vehicle_respond, v);
        }


        // PHASE III – V2V Discovery
        for (uint32_t r = 0; r < N_RSUs; r++)
        {
            Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.0001*r),
                arch2_v2v_rsu_inject_lldp, r);
        }

        for (uint32_t v = 0; v < N_Vehicles; v++)
        {
            Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.002 + 0.0001*v),
                arch2_v2v_vehicle_rebroadcast, v);

            // Neighbors: schedule all other vehicles in same RSU zone
            Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.003 + 0.0001*v),
                arch2_v2v_neighbor_confirm, v);
        }

    }
 } // end per-round loop


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

void centralized_dsrc_data_unicast(Ptr <Node> source_node, uint32_t node_index, uint32_t destination, uint32_t port_id)
{
	uint32_t link_may_exist;
	if((attack_number == 1) || (attack_number == 6))
	{
		link_may_exist = abs(unit_step(400 - adjacencyMatrix[node_index][destination], 0));
	}
	else
	{
		link_may_exist = abs(unit_step(350 - adjacencyMatrix[node_index][destination], 0));
	}
	
	
	if(routing_algorithm == 4)
	{
		if(link_may_exist == 1)
		{
				Simulator::Schedule(Seconds(0), encrypt_dsrc_data_unicast, node_index, destination, port_id); 
				Simulator::Schedule(Seconds(0.0002), send_dsrc_data_unicast, source_node, node_index, destination, port_id);
		}
	}
	else
	{
		Simulator::Schedule(Seconds(0.0002), send_dsrc_data_unicast, source_node, node_index, destination, port_id);
	}
}

// --- Architecture 2: Vehicle-to-RSU Assignment ---
void arch2_assign_vehicles_to_rsus()
{
    vehicle_to_rsu_map.resize(N_Vehicles, 0);
    if (N_RSUs == 0) return;

    for (uint32_t v = 0; v < Vehicle_Nodes.GetN(); v++)
    {
        Ptr<MobilityModel> vMob =
            Vehicle_Nodes.Get(v)->GetObject<MobilityModel>();
        Vector vPos = vMob->GetPosition();
        double   minDist = 1e18;
        uint32_t bestRSU = 0;

        for (uint32_t r = 0; r < RSU_Nodes.GetN(); r++)
        {
            Ptr<MobilityModel> rMob =
                RSU_Nodes.Get(r)->GetObject<MobilityModel>();
            Vector rPos = rMob->GetPosition();
            double dist = sqrt(pow(vPos.x-rPos.x,2) + pow(vPos.y-rPos.y,2));
            if (dist < minDist) { minDist = dist; bestRSU = r; }
        }
        vehicle_to_rsu_map[v] = bestRSU;
    }
    NS_LOG_INFO("[ARCH2-INIT] Vehicle-to-RSU assignment complete");
}

// --- Architecture 2: I2I Controller Send Packet-Out ---
void arch2_i2i_controller_send_packetout()
{
    // Controller sends LLDP Packet-Out to all RSUs via CSMA wired backhaul.
    // In ns3: modeled as a UDP unicast from controller to each RSU app.
    NS_LOG_INFO("[I2I-1] t=" << Simulator::Now().GetSeconds()
                << "s Controller → all RSUs: LLDP Packet-Out ("
                << lldp_i2i_pkt_size << "B) via CSMA");

    Ptr<Application> ctrl_app = apps.Get(0); // controller = apps[0]

    for (uint32_t r = 0; r < N_RSUs; r++)
    {
        Ptr<Node> rsu_node = RSU_Nodes.Get(r);
        // Reuse existing wired unicast: controller → RSU via CSMA subnet 10.1.1.0/24
        // p2p_data_broadcast sends from ctrl_app toward rsu_node destination
        p2p_data_broadcast(ctrl_app, controller_Node.Get(0));
    }
}

// --- Architecture 2: I2I RSU Forward to Neighbors ---
void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index)
{
    // RSU broadcasts LLDP to neighboring RSUs via CSMA (wired).
    // Sequence number 2 in I2I diagram.
    NS_LOG_INFO("[I2I-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " → neighbor RSUs: LLDP broadcast via CSMA");

    Ptr<Application> rsu_app = RSU_apps.Get(rsu_index);
    Ptr<Node> rsu_node = RSU_Nodes.Get(rsu_index);

    // Broadcast on CSMA — reaches all nodes on 10.1.1.0/24
    p2p_data_broadcast(rsu_app, rsu_node);
}

// --- Architecture 2: I2V RSU Encapsulate and Forward ---
void arch2_i2v_controller_send_packetout(uint32_t rsu_index)
{
    // Sequence 1: Controller sends LLDP Packet-Out to target RSU
    NS_LOG_INFO("[I2V-1] t=" << Simulator::Now().GetSeconds()
                << "s Controller → RSU " << rsu_index
                << ": LLDP Packet-Out for I2V discovery");

    Ptr<SimpleUdpApplication> ctrl_app =
        DynamicCast<SimpleUdpApplication>(apps.Get(0));
    p2p_data_broadcast(ctrl_app, controller_Node.Get(0));
}
void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index)
{
    NS_LOG_INFO("[I2V-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " → Vehicles: encapsulated LLDP ("
                << lldp_i2v_pkt_size << "B) via DSRC ch178");

    centralized_dsrc_data_broadcast(
        wifidevices.Get(N_Vehicles + rsu_index),
        RSU_Nodes.Get(rsu_index),
        N_Vehicles + rsu_index,
        0   // channel 178 (control channel)
    );
}

// --- Architecture 2: I2V Vehicle Respond ---
void arch2_i2v_vehicle_respond(uint32_t vehicle_index)
{
    NS_LOG_INFO("[I2V-3] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index
                << " → RSU " << vehicle_to_rsu_map[vehicle_index]
                << ": LLDP response via DSRC");

    uint32_t rsu_idx = vehicle_to_rsu_map[vehicle_index];

    centralized_dsrc_data_unicast(
        dsrc_Nodes.Get(vehicle_index),        
        vehicle_index,                         
        N_Vehicles + rsu_idx,                  
        0
    );
}

// --- Architecture 2: V2V RSU Inject LLDP ---
void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index)
{
    // Sequence 1: RSU injects LLDP into wireless medium selectively.
    NS_LOG_INFO("[V2V-1] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " injecting LLDP for V2V discovery via DSRC");

    centralized_dsrc_data_broadcast(
        wifidevices.Get(N_Vehicles + rsu_index),
        RSU_Nodes.Get(rsu_index),
        N_Vehicles + rsu_index,
        1   // use a service channel (e.g., ch172) to distinguish from I2V
    );
}

// --- Architecture 2: V2V Vehicle Rebroadcast ---
void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index)
{
    bool lldp_is_malicious = false;
    for (uint32_t a = 0; a < (uint32_t)(attack_number); a++)
    {
        // If the RSU that sent this is an attacker, the LLDP is malicious
        if (attacker_node_id[a] == (int)(N_Vehicles + vehicle_to_rsu_map[vehicle_index]))
        {
            lldp_is_malicious = true;
            break;
        }
    }

    // Uncomment to enable FL-GAN filtering:
    // if (lldp_is_malicious)
    // {
    //     NS_LOG_INFO("[FL-GAN VEHICLE] t=" << Simulator::Now().GetSeconds()
    //                 << "s Vehicle " << vehicle_index
    //                 << " local GAN detected malicious LLDP — not rebroadcasting");
    //     return;
    // }

    NS_LOG_INFO("[V2V-2] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index
                << " rebroadcasting LLDP to neighbors via DSRC");

    // Broadcast to all neighboring vehicles
    centralized_dsrc_data_broadcast(
        wifidevices.Get(vehicle_index),
        dsrc_Nodes.Get(vehicle_index),
        vehicle_index,
        1
    );
}

// --- Architecture 2: V2V Neighbor Confirm ---
void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index)
{
    // Sequence 3: Neighboring vehicle detects rebroadcast,
    // sends confirmation back to its serving RSU.
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
        1
    );
}

// --- Placeholder Implementations (Replace with actual from LDA.cc) ---
void p2p_data_broadcast(Ptr<Application> app, Ptr<Node> node)
{
    // TODO: Implement based on LDA.cc p2p_data_broadcast
    NS_LOG_INFO("p2p_data_broadcast called");
}

void centralized_dsrc_data_broadcast(Ptr<NetDevice> dev, Ptr<Node> node, uint32_t idx, int channel)
{
    // TODO: Implement based on LDA.cc centralized_dsrc_data_broadcast
    NS_LOG_INFO("centralized_dsrc_data_broadcast called");
}

void centralized_dsrc_data_unicast(Ptr<Node> node, uint32_t src_idx, uint32_t dst_idx, int channel)
{
    // TODO: Implement based on LDA.cc centralized_dsrc_data_unicast
    NS_LOG_INFO("centralized_dsrc_data_unicast called");
}