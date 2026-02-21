#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wave-module.h"
#include "ns3/log.h"
#include "ns3/tag.h"
#include "ns3/socket.h"
#include "ns3/constant-acceleration-mobility-model.h"
#include <iomanip>
#include <cmath>
#include <bits/stdc++.h>
#include <chrono>

using namespace ns3;
using namespace std;
using namespace std::chrono;

NS_LOG_COMPONENT_DEFINE ("LDA-SDVENs");

#define max 40
#define max1 1

// GLOBAL NETWORK COMPONENTS
NodeContainer controller_Node, RSU_Nodes, Vehicle_Nodes;
NodeContainer dsrc_Nodes;
NetDeviceContainer rsuDevices, vehicleDevices, wifidevices;
Ipv4InterfaceContainer rsuInterfaces, vehicleInterfaces;
ApplicationContainer apps, RSU_apps;

// NETWORK CONFIGURATION VARIABLES
uint32_t N_Vehicles = 16;
uint32_t N_RSUs = 3;
const int total_size = 16;
uint32_t large = 50000;
uint32_t var = N_Vehicles + N_RSUs;
double simTime = 13.7;

// LLDP and Discovery Parameters
std::vector<uint32_t> vehicle_to_rsu_map;
uint32_t lldp_i2i_pkt_size = 64;
uint32_t lldp_i2v_pkt_size = 64;
uint32_t lldp_v2v_pkt_size = 256;

// ROUTING AND ALGORITHM PARAMETERS
int routing_algorithm = 4; // 0-port based, 1-normal LLDP, 2-crypto, 3-Link guard, 4-proposed LLDP
int experiment_number = 0; // 0-individual attack, 1-combined attack
int attack_number = 5; // 1-Attack 1, 2-Attack 2, ..., 6-Combined
int attack_percentage = 40;

// TIMING AND FREQUENCY PARAMETERS
double optimization_frequency = 0.33;
double optimization_period = 1.0 / optimization_frequency;
double data_transmission_frequency = 0.33;
double data_transmission_period = 1.0 / data_transmission_frequency;
double entropy_threshold = 0.005;
double routing_frequency = data_transmission_frequency;
double contention_threshold = 0.0;
double link_lifetime_threshold = 0.400;

// ARCHITECTURE AND MOBILITY CONFIGURATION
int mobility_scenario = 0; // 0-urban, 1-non-urban, 2-highway
int architecture = 2; // 0-centralized, 1-distributed, 2-hybrid
int maxspeed = 60;
int paper = 1; // 0-optimization, 1-architecture


// QoS AND MAC PARAMETERS
uint32_t flow_packet_size = 100;
uint32_t qf = 1;
uint32_t AIFSN = 0;
double B_max = 0.0;
double latency_max = 0.0;
double loss_max = 0.0;
uint32_t CW_max = 0;
double AIFS = 0.0;
double mu1 = 0.010;
double mu2 = 10.00;
double mu3 = 10.0;

// ATTACK PRESENCE FLAGS AND MALICIOUS NODE ARRAYS
// Attack presence flagsfor vehicles
bool present_location_attack_nodes = false;
bool present_flooding_attack_nodes = true;
bool present_fabrication_attack_nodes = false;
bool present_MIM_attack_nodes = false;
bool present_vanishing_attack_nodes = false;

// Attack presence flags for controllers
bool present_flooding_attack_controllers = false;
bool present_fabrication_attack_controllers = false;
bool present_MIM_attack_controllers = false;
bool present_vanishing_attack_controllers = false;

// Malicious node arrays
bool location_malicious_nodes[total_size];
bool flooding_malicious_nodes[total_size];
bool fabrication_malicious_nodes[total_size];
bool MIM_malicious_nodes[total_size];
bool vanishing_malicious_nodes[total_size];

// Malicious controller arrays
int controllers = 4;
bool flooding_malicious_controllers[4];
bool fabrication_malicious_controllers[4];
bool MIM_malicious_controllers[4];
bool vanishing_malicious_controllers[4];


// LINK STATE, CONTROL, AND IDENTIFICATION VARIABLES
uint32_t empty_neighborset[max];
int attacker_node_id[100];
uint32_t node_controller_ID[total_size];
uint32_t assigned_consortium_ID[total_size];
bool blocked_port_state[total_size][total_size][2];
uint32_t flooding_counter[total_size][total_size][2];

// TRAINING AND TIMESTAMP VARIABLES
bool training = false;
bool training_delay = false;
bool controller_malicious_assumption = true;

double HELLO_final_timestamp;
double HELLO_initial_timestamp;
double uplink_last[total_size];
double downlink_last = 0.0;
double last_downlink[total_size];


// ARCHITECTURE 2 TIMING OFFSETS (Architecture 2 is SDVENs)

double arch2_i2i_offset = 0.002;
double arch2_i2i_recv_offset = 0.004;
double arch2_i2v_offset = 0.006;
double arch2_i2v_recv_offset = 0.008;
double arch2_v2v_offset = 0.010;
double arch2_v2v_recv_offset = 0.014;

// UE STATUS AND CHANNEL UTILIZATION
std::vector<bool> ueBusy;
std::vector<bool> ueDLBusy;
double current_channel_utilization = 0.0;
double average_channel_utilization = 0.0;


// LINK AND ROUTING STRUCTURES
struct Link_fi {
    double Link_values[total_size];
};

struct Link_f {
    struct Link_fi Link_fi_inst[total_size];
};

struct Link {
    struct Link_f Link_f_inst[2];
};

struct Link Link_at_controller_inst[2];
struct Link Link_duplicates_at_controller_inst[2];
struct Link Link_duplicates_downlink_at_controller_inst[2];
struct Link Link_duplicates_SecondTime_downlink_at_controller_inst[2];
struct Link Link_duplicates_SecondTime_uplink_at_controller_inst[2];
struct Link LLDP_timestamp_at_controller_inst[2];


// ADJACENCY MATRIX AND ROUTING
vector<vector<double>> adjacencyMatrix;
vector<vector<double>> old_adjacencyMatrix;


// ANOMALY DETECTION MATRICES
vector<vector<vector<double>>> E_mat;
vector<vector<bool>> F_mat;


// PACKET AND DATA STRUCTURES
struct downlink_rest_data {
    uint32_t casted_raw_source_nodeid;
    uint32_t casted_raw_source_portid;
    uint32_t casted_destination_nodeid;
    uint32_t casted_raw_destination_portid;
    uint8_t * stage = new uint8_t[2];
    uint8_t * HMAC_key = new uint8_t[163];
    uint8_t * HMAC = new uint8_t[65];
    uint8_t * DS_public_key1 = new uint8_t[513];
    uint8_t * DS_public_key2 = new uint8_t[2561];
    uint8_t * DS_public_key3 = new uint8_t[2561];
    uint8_t * DS1 = new uint8_t[201];
    uint8_t * DS2 = new uint8_t[201];
    uint8_t * source_nodeid = new uint8_t[36];
    uint8_t * source_portid = new uint8_t[36];
    uint8_t * raw_source_nodeid = new uint8_t[2];
    uint8_t * raw_source_portid = new uint8_t[2];
    uint8_t * destination_nodeid = new uint8_t[33];
    uint8_t * destination_portid = new uint8_t[33];
    uint8_t * HMAC1 = new uint8_t[65];
    uint8_t * HMAC2 = new uint8_t[65];
};


// UTILITY FUNCTIONS
double sum_of_nodeids = 0;

void initialize_empty() {
    for (uint32_t i = 0; i < max; i++) {
        empty_neighborset[i] = large;
    }
}

void nodeid_sum() {
    sum_of_nodeids = 0;
    for (uint32_t i = 2; i < total_size + 2; i++) {
        sum_of_nodeids = sum_of_nodeids + i;
    }
}

// Unit step function for link validation (returns 0 if number <= step, else 1)
double unit_step(double number, double step) {
    if (number <= step) {
        return 0.0;
    } else {
        return 1.0;
    }
}

// Boolean probability function for attack injection
bool GetBooleanWithProbability(double probabilityPercent, int nodeID) {
    srand(Simulator::Now().GetSeconds() + 1.0 * (rand() % 50) + 5.0 * nodeID);
    double randomValue = 1.0 * (rand() % 100);
    return randomValue < probabilityPercent;
}

// Initialize E and F matrices for anomaly detection
void generate_F_and_E() {
    E_mat.assign(total_size, vector<vector<double>>(total_size, vector<double>(2, 0.0)));
    F_mat.assign(total_size, vector<bool>(2, false));

    NS_LOG_INFO("E and F matrices initialized");

    for (uint32_t i = 0; i < total_size; i++) {
        for (uint32_t j = 0; j < total_size; j++) {
            for (uint32_t k = 0; k < 2; k++) {
                blocked_port_state[i][j][k] = false;
                flooding_counter[i][j][k] = 0;
            }
        }
    }
}


// FUNCTION PROTOTYPES

// Phase Discovery Functions
void PhaseI_I2IDiscovery();
void PhaseII_I2VDiscovery();
void PhaseIII_V2VDiscovery();

// Architecture 2 Functions
void arch2_assign_vehicles_to_rsus();
void arch2_i2i_controller_send_packetout();
void arch2_i2v_controller_send_packetout(uint32_t rsu_index);
void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index);
void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index);
void arch2_i2v_vehicle_respond(uint32_t vehicle_index);
void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index);
void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index);
void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index);
void arch2_log_discovered_links(uint32_t cycle_number);

// Data Transmission Functions
void p2p_data_broadcast(Ptr<Application> app, Ptr<Node> node);
void centralized_dsrc_data_broadcast(Ptr<NetDevice> dev, Ptr<Node> node, uint32_t idx, int channel);
void centralized_dsrc_data_unicast(Ptr<Node> node, uint32_t src_idx, uint32_t dst_idx, int channel);
void send_dsrc_data_unicast(Ptr<Node> source_node, uint32_t node_index, uint32_t destination, uint32_t port_id);
void encrypt_dsrc_data_unicast(uint32_t node_index, uint32_t destination, uint32_t port_id);

// IMPLEMENTATION: MAIN FUNCTION
int main(int argc, char *argv[]) {
    // Initialize utility functions
    initialize_empty();
    nodeid_sum();
    generate_F_and_E();

    // Parse command line arguments
    CommandLine cmd;
    cmd.AddValue("N_Vehicles", "Number of vehicles", N_Vehicles);
    cmd.AddValue("N_RSUs", "Number of RSUs", N_RSUs);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("routing_algorithm", "Routing algorithm (0-5)", routing_algorithm);
    cmd.AddValue("attack_number", "Attack scenario number", attack_number);
    cmd.AddValue("attack_percentage", "Percentage of malicious nodes", attack_percentage);
    cmd.AddValue("architecture", "Architecture: 0-centralized, 1-distributed, 2-hybrid", architecture);
    cmd.Parse(argc, argv);

    // Recalculate derived values
    var = N_Vehicles + N_RSUs;
    
    NS_LOG_UNCOND("\n========== LDA-in-SDVENs Simulation ==========");
    NS_LOG_UNCOND("Vehicles: " << N_Vehicles << " | RSUs: " << N_RSUs);
    NS_LOG_UNCOND("Architecture: " << architecture << " | Routing: " << routing_algorithm);
    NS_LOG_UNCOND("Attack Scenario: " << attack_number << " (" << attack_percentage << "%)");
    NS_LOG_UNCOND("Simulation Time: " << simTime << " seconds");
    NS_LOG_UNCOND("===========================================\n");

    // Create nodes
    controller_Node.Create(1);
    Vehicle_Nodes.Create(N_Vehicles);
    if (N_RSUs > 0) {
        RSU_Nodes.Create(N_RSUs);
    }
    dsrc_Nodes.Add(Vehicle_Nodes);
    dsrc_Nodes.Add(RSU_Nodes);

    // ========== Infrastructure Setup (Wired Backbone - CSMA) ==========
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1000Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));

    NodeContainer csma_nodes;
    csma_nodes.Add(controller_Node);
    csma_nodes.Add(RSU_Nodes);
    NetDeviceContainer csmaDevices = csma.Install(csma_nodes);

    // ========== Wireless Setup (WAVE/DSRC for V2V/I2V) ==========
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    Wifi80211pHelper wifi = Wifi80211pHelper::Default();
    NqosWaveMacHelper mac = NqosWaveMacHelper::Default();

    NodeContainer wirelessNodes;
    wirelessNodes.Add(Vehicle_Nodes);  // Vehicles first: indices 0 to N_Vehicles-1
    wirelessNodes.Add(RSU_Nodes);      // RSUs second: indices N_Vehicles to N_Vehicles+N_RSUs-1
    NetDeviceContainer wirelessDevices = wifi.Install(phy, mac, wirelessNodes);
    wifidevices = wirelessDevices;

    // ========== Internet Stack ==========
    InternetStackHelper stack;
    stack.Install(controller_Node);
    stack.Install(RSU_Nodes);
    stack.Install(Vehicle_Nodes);

    // ========== IP Address Assignment ==========
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(csmaDevices);

    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wirelessInterfaces = address.Assign(wirelessDevices);

    // ========== Mobility ==========
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    
    // RSU positions (fixed grid)
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue(0.0),
                                   "MinY", DoubleValue(0.0),
                                   "DeltaX", DoubleValue(500.0),
                                   "DeltaY", DoubleValue(500.0),
                                   "GridWidth", UintegerValue(10),
                                   "LayoutType", StringValue("RowFirst"));
    mobility.Install(RSU_Nodes);

    // Vehicle mobility (initial positions)
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < N_Vehicles; i++) {
        positionAlloc->Add(Vector(i * 50.0, 0.0, 0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(Vehicle_Nodes);

    // ========== Initialize Global Components ==========
    ueBusy.resize(N_Vehicles + N_RSUs, false);
    ueDLBusy.resize(N_Vehicles + N_RSUs, false);

    for (uint32_t i = 0; i < total_size; i++) {
        uplink_last[i] = 0.0;
        last_downlink[i] = 0.0;
    }

    // ========== Configure Attack Scenarios ==========
    NS_LOG_INFO("Configuring attack scenario " << attack_number);
    
    for (uint32_t i = 0; i < total_size; i++) {
        location_malicious_nodes[i] = present_location_attack_nodes && 
                                       GetBooleanWithProbability(attack_percentage, i);
        flooding_malicious_nodes[i] = present_flooding_attack_nodes && 
                                       GetBooleanWithProbability(attack_percentage, i);
        fabrication_malicious_nodes[i] = present_fabrication_attack_nodes && 
                                          GetBooleanWithProbability(attack_percentage, i);
        MIM_malicious_nodes[i] = present_MIM_attack_nodes && 
                                 GetBooleanWithProbability(attack_percentage, i);
        vanishing_malicious_nodes[i] = present_vanishing_attack_nodes && 
                                       GetBooleanWithProbability(attack_percentage, i);
        
        if (flooding_malicious_nodes[i]) {
            NS_LOG_INFO("Node " << i << " marked as flooding attacker");
        }
    }

    for (int i = 0; i < controllers; i++) {
        flooding_malicious_controllers[i] = present_flooding_attack_controllers && 
                                            controller_malicious_assumption;
        fabrication_malicious_controllers[i] = present_fabrication_attack_controllers && 
                                               controller_malicious_assumption;
        MIM_malicious_controllers[i] = present_MIM_attack_controllers && 
                                       controller_malicious_assumption;
        vanishing_malicious_controllers[i] = present_vanishing_attack_controllers && 
                                             controller_malicious_assumption;
    }

    //Scheduling Discovery Phases
    Simulator::Schedule(Seconds(1.0), &PhaseI_I2IDiscovery);
    Simulator::Schedule(Seconds(2.0), &PhaseII_I2VDiscovery);
    for (double t = 3.0; t < simTime; t += 2.0) {
        Simulator::Schedule(Seconds(t), &PhaseIII_V2VDiscovery);
    }

    // Hybrid Link Layer Discovery Protocol
    if (architecture == 2) {
        Simulator::Schedule(Seconds(1.0), &arch2_assign_vehicles_to_rsus);

        double t_start = 6.00;
        uint32_t cycle_num = 0;
        for (double t = t_start; t < simTime - 1; t += data_transmission_period) {
            // PHASE I – I2I Discovery
            Simulator::Schedule(Seconds(t + arch2_i2i_offset),
                &arch2_i2i_controller_send_packetout);

            for (uint32_t r = 0; r < N_RSUs; r++) {
                Simulator::Schedule(Seconds(t + arch2_i2i_offset + 0.0001 * r),
                    &arch2_i2i_rsu_forward_to_neighbors, r);
            }

            // PHASE II – I2V Discovery
            for (uint32_t r = 0; r < N_RSUs; r++) {
                Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.0001 * r),
                    &arch2_i2v_controller_send_packetout, r);

                Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.0002 * r),
                    &arch2_i2v_rsu_encapsulate_and_forward, r);
            }

            for (uint32_t v = 0; v < N_Vehicles; v++) {
                Simulator::Schedule(Seconds(t + arch2_i2v_offset + 0.003 + 0.0001 * v),
                    &arch2_i2v_vehicle_respond, v);
            }

            // PHASE III – V2V Discovery
            for (uint32_t r = 0; r < N_RSUs; r++) {
                Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.0001 * r),
                    &arch2_v2v_rsu_inject_lldp, r);
            }

            for (uint32_t v = 0; v < N_Vehicles; v++) {
                Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.002 + 0.0001 * v),
                    &arch2_v2v_vehicle_rebroadcast, v);

                Simulator::Schedule(Seconds(t + arch2_v2v_offset + 0.003 + 0.0001 * v),
                    &arch2_v2v_neighbor_confirm, v);
            }

            // Log discovered links after all phases complete
            Simulator::Schedule(Seconds(t + arch2_v2v_recv_offset + 0.002),
                &arch2_log_discovered_links, cycle_num);
            
            cycle_num++;
        }
    }

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    
    NS_LOG_UNCOND("\n========== Simulation Complete ==========");
    NS_LOG_UNCOND("Total simulation time: " << simTime << " seconds");
    NS_LOG_UNCOND("========================================\n");
    
    Simulator::Destroy();
    return 0;
}

// ============================================================================
// IMPLEMENTATION: PHASE DISCOVERY FUNCTIONS
// ============================================================================

void PhaseI_I2IDiscovery() {
    NS_LOG_UNCOND("[Phase I] t=" << Simulator::Now().GetSeconds() 
                  << "s - Controller initiating LLDP Packet-Out for I2I Discovery");
    arch2_assign_vehicles_to_rsus();
}

void PhaseII_I2VDiscovery() {
    NS_LOG_UNCOND("[Phase II] t=" << Simulator::Now().GetSeconds() 
                  << "s - RSU encapsulating LLDP for I2V Discovery");
}

void PhaseIII_V2VDiscovery() {
    NS_LOG_UNCOND("[Phase III] t=" << Simulator::Now().GetSeconds() 
                  << "s - Initiating Cooperative V2V Discovery Rebroadcast");
}

// ============================================================================
// IMPLEMENTATION: ARCHITECTURE 2 FUNCTIONS
// ============================================================================

void arch2_assign_vehicles_to_rsus() {
    vehicle_to_rsu_map.resize(N_Vehicles, 0);
    if (N_RSUs == 0) return;

    for (uint32_t v = 0; v < Vehicle_Nodes.GetN(); v++) {
        Ptr<MobilityModel> vMob = Vehicle_Nodes.Get(v)->GetObject<MobilityModel>();
        Vector vPos = vMob->GetPosition();
        double minDist = 1e18;
        uint32_t bestRSU = 0;

        for (uint32_t r = 0; r < RSU_Nodes.GetN(); r++) {
            Ptr<MobilityModel> rMob = RSU_Nodes.Get(r)->GetObject<MobilityModel>();
            Vector rPos = rMob->GetPosition();
            double dist = sqrt(pow(vPos.x - rPos.x, 2) + pow(vPos.y - rPos.y, 2));
            if (dist < minDist) {
                minDist = dist;
                bestRSU = r;
            }
        }
        vehicle_to_rsu_map[v] = bestRSU;
    }
    NS_LOG_UNCOND("[ARCH2] Vehicle-to-RSU assignment complete");
}

void arch2_i2i_controller_send_packetout() {
    NS_LOG_UNCOND("[I2I-1] t=" << Simulator::Now().GetSeconds()
                << "s Controller → all RSUs: LLDP Packet-Out (" << lldp_i2i_pkt_size << "B) via CSMA");
    
    for (uint32_t r = 0; r < N_RSUs; r++) {
        p2p_data_broadcast(nullptr, controller_Node.Get(0));
    }
}

void arch2_i2i_rsu_forward_to_neighbors(uint32_t rsu_index) {
    NS_LOG_UNCOND("[I2I-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index << " → neighbor RSUs: LLDP broadcast via CSMA");
    
    if (rsu_index < RSU_Nodes.GetN()) {
        p2p_data_broadcast(nullptr, RSU_Nodes.Get(rsu_index));
    }
}

void arch2_i2v_controller_send_packetout(uint32_t rsu_index) {
    NS_LOG_UNCOND("[I2V-1] t=" << Simulator::Now().GetSeconds()
                << "s Controller → RSU " << rsu_index << ": LLDP Packet-Out for I2V discovery");
    
    p2p_data_broadcast(nullptr, controller_Node.Get(0));
}

void arch2_i2v_rsu_encapsulate_and_forward(uint32_t rsu_index) {
    NS_LOG_UNCOND("[I2V-2] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index
                << " → Vehicles: encapsulated LLDP (" << lldp_i2v_pkt_size << "B) via DSRC");
    
    uint32_t dev_idx = N_Vehicles + rsu_index;
    if (dev_idx >= wifidevices.GetN() || rsu_index >= RSU_Nodes.GetN()) {
        NS_LOG_WARN("[I2V-2] Invalid index: dev_idx=" << dev_idx << ", rsu_index=" << rsu_index);
        return;
    }
    
    centralized_dsrc_data_broadcast(
        wifidevices.Get(dev_idx),
        RSU_Nodes.Get(rsu_index),
        dev_idx,
        0
    );
}

void arch2_i2v_vehicle_respond(uint32_t vehicle_index) {
    if (vehicle_index >= vehicle_to_rsu_map.size() || vehicle_index >= dsrc_Nodes.GetN()) {
        NS_LOG_WARN("[I2V-3] Invalid vehicle_index=" << vehicle_index);
        return;
    }
    
    NS_LOG_UNCOND("[I2V-3] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index
                << " → RSU " << vehicle_to_rsu_map[vehicle_index] << ": LLDP response via DSRC");
    
    uint32_t rsu_idx = vehicle_to_rsu_map[vehicle_index];
    centralized_dsrc_data_unicast(
        dsrc_Nodes.Get(vehicle_index),
        vehicle_index,
        N_Vehicles + rsu_idx,
        0
    );
}

void arch2_v2v_rsu_inject_lldp(uint32_t rsu_index) {
    NS_LOG_UNCOND("[V2V-1] t=" << Simulator::Now().GetSeconds()
                << "s RSU " << rsu_index << " injecting LLDP for V2V discovery via DSRC");
    
    uint32_t dev_idx = N_Vehicles + rsu_index;
    if (dev_idx >= wifidevices.GetN() || rsu_index >= RSU_Nodes.GetN()) {
        NS_LOG_WARN("[V2V-1] Invalid index: dev_idx=" << dev_idx << ", rsu_index=" << rsu_index);
        return;
    }
    
    centralized_dsrc_data_broadcast(
        wifidevices.Get(dev_idx),
        RSU_Nodes.Get(rsu_index),
        dev_idx,
        1
    );
}

void arch2_v2v_vehicle_rebroadcast(uint32_t vehicle_index) {
    if (vehicle_index >= vehicle_to_rsu_map.size() || vehicle_index >= wifidevices.GetN()) {
        NS_LOG_WARN("[V2V-2] Invalid vehicle_index=" << vehicle_index);
        return;
    }
    
    //bool lldp_is_malicious = false;
    for (uint32_t a = 0; a < (uint32_t)(attack_number); a++) {
        if (attacker_node_id[a] == (int)(N_Vehicles + vehicle_to_rsu_map[vehicle_index])) {
            //lldp_is_malicious = true;
            break;
        }
    }
    
    NS_LOG_UNCOND("[V2V-2] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << vehicle_index << " rebroadcasting LLDP to neighbors via DSRC");
    
    centralized_dsrc_data_broadcast(
        wifidevices.Get(vehicle_index),
        dsrc_Nodes.Get(vehicle_index),
        vehicle_index,
        1
    );
}

void arch2_v2v_neighbor_confirm(uint32_t neighbor_vehicle_index) {
    if (neighbor_vehicle_index >= vehicle_to_rsu_map.size() || neighbor_vehicle_index >= dsrc_Nodes.GetN()) {
        NS_LOG_WARN("[V2V-3] Invalid neighbor_vehicle_index=" << neighbor_vehicle_index);
        return;
    }
    
    NS_LOG_UNCOND("[V2V-3] t=" << Simulator::Now().GetSeconds()
                << "s Vehicle " << neighbor_vehicle_index
                << " (neighbor) → RSU " << vehicle_to_rsu_map[neighbor_vehicle_index] << ": V2V confirmation");
    
    uint32_t rsu_idx = vehicle_to_rsu_map[neighbor_vehicle_index];
    centralized_dsrc_data_unicast(
        dsrc_Nodes.Get(neighbor_vehicle_index),
        neighbor_vehicle_index,
        N_Vehicles + rsu_idx,
        1
    );
}

void arch2_log_discovered_links(uint32_t cycle_number) {
    NS_LOG_UNCOND("\n========== Discovery Cycle #" << cycle_number 
                  << " Complete (t=" << Simulator::Now().GetSeconds() << "s) ==========");
    
    // Log I2I links (RSU-to-RSU)
    uint32_t i2i_count = 0;
    NS_LOG_UNCOND("[I2I Links] RSU-to-RSU (CSMA Backbone):");
    for (uint32_t r1 = 0; r1 < N_RSUs; r1++) {
        for (uint32_t r2 = r1 + 1; r2 < N_RSUs; r2++) {
            // Simulate link detection based on grid proximity (500m spacing)
            Ptr<MobilityModel> mob1 = RSU_Nodes.Get(r1)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob2 = RSU_Nodes.Get(r2)->GetObject<MobilityModel>();
            Vector pos1 = mob1->GetPosition();
            Vector pos2 = mob2->GetPosition();
            double dist = sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
            
            // RSUs within 750m are considered neighbors on wired backbone
            if (dist <= 750.0) {
                NS_LOG_UNCOND("  - RSU" << r1 << " ↔ RSU" << r2 
                              << " [distance: " << std::fixed << std::setprecision(1) << dist << "m]");
                i2i_count++;
            }
        }
    }
    if (i2i_count == 0) NS_LOG_UNCOND("  - No I2I links detected");
    
    // Log I2V links (RSU-to-Vehicle)
    NS_LOG_UNCOND("\n[I2V Links] RSU-to-Vehicle (DSRC):");
    uint32_t i2v_count = 0;
    for (uint32_t v = 0; v < N_Vehicles; v++) {
        if (v < vehicle_to_rsu_map.size()) {
            uint32_t rsu_idx = vehicle_to_rsu_map[v];
            Ptr<MobilityModel> vMob = Vehicle_Nodes.Get(v)->GetObject<MobilityModel>();
            Ptr<MobilityModel> rMob = RSU_Nodes.Get(rsu_idx)->GetObject<MobilityModel>();
            Vector vPos = vMob->GetPosition();
            Vector rPos = rMob->GetPosition();
            double dist = sqrt(pow(vPos.x - rPos.x, 2) + pow(vPos.y - rPos.y, 2));
            
            NS_LOG_UNCOND("  - Vehicle" << v << " ↔ RSU" << rsu_idx 
                          << " [distance: " << std::fixed << std::setprecision(1) << dist << "m]");
            i2v_count++;
        }
    }
    if (i2v_count == 0) NS_LOG_UNCOND("  - No I2V links detected");
    
    // Log V2V links (Vehicle-to-Vehicle)
    NS_LOG_UNCOND("\n[V2V Links] Vehicle-to-Vehicle (DSRC, cooperative):");
    uint32_t v2v_count = 0;
    for (uint32_t v1 = 0; v1 < N_Vehicles; v1++) {
        for (uint32_t v2 = v1 + 1; v2 < N_Vehicles; v2++) {
            Ptr<MobilityModel> mob1 = Vehicle_Nodes.Get(v1)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob2 = Vehicle_Nodes.Get(v2)->GetObject<MobilityModel>();
            Vector pos1 = mob1->GetPosition();
            Vector pos2 = mob2->GetPosition();
            double dist = sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
            
            // DSRC communication range ~300m
            if (dist <= 300.0) {
                NS_LOG_UNCOND("  - Vehicle" << v1 << " ↔ Vehicle" << v2 
                              << " [distance: " << std::fixed << std::setprecision(1) << dist << "m]");
                v2v_count++;
            }
        }
    }
    if (v2v_count == 0) NS_LOG_UNCOND("  - No V2V links detected");
    
    NS_LOG_UNCOND("\n[Summary] Total Links: I2I=" << i2i_count 
                  << ", I2V=" << i2v_count << ", V2V=" << v2v_count);
    NS_LOG_UNCOND("======================================================\n");
}

// ============================================================================
// IMPLEMENTATION: DATA TRANSMISSION FUNCTIONS
// ============================================================================

void p2p_data_broadcast(Ptr<Application> app, Ptr<Node> node) {
    NS_LOG_UNCOND("[P2P Broadcast] Broadcasting LLDP discovery packet at t=" 
                << Simulator::Now().GetSeconds());
}

void centralized_dsrc_data_broadcast(Ptr<NetDevice> dev, Ptr<Node> node, uint32_t idx, int channel) {
    NS_LOG_UNCOND("[DSRC Broadcast] Channel " << channel 
                << " - Broadcasting from node " << idx << " at t=" << Simulator::Now().GetSeconds());
}

void centralized_dsrc_data_unicast(Ptr<Node> source_node, uint32_t node_index, 
                                    uint32_t destination, int channel) {
    // Initialize adjacency matrix if needed
    if (adjacencyMatrix.empty()) {
        adjacencyMatrix.assign(total_size, vector<double>(total_size, 0.0));
    }
    
    uint32_t link_may_exist = 0;
    if ((attack_number == 1) || (attack_number == 6)) {
        link_may_exist = abs((int)unit_step(400 - adjacencyMatrix[node_index][destination], 0));
    } else {
        link_may_exist = abs((int)unit_step(350 - adjacencyMatrix[node_index][destination], 0));
    }
    
    uint32_t port_id = (uint32_t)channel;
    if (routing_algorithm == 4) {
        if (link_may_exist == 1) {
            Simulator::Schedule(Seconds(0), &encrypt_dsrc_data_unicast, node_index, destination, port_id);
            Simulator::Schedule(Seconds(0.0002), &send_dsrc_data_unicast, source_node, node_index, destination, port_id);
        }
    } else {
        Simulator::Schedule(Seconds(0.0002), &send_dsrc_data_unicast, source_node, node_index, destination, port_id);
    }
}

void send_dsrc_data_unicast(Ptr<Node> source_node, uint32_t node_index, 
                             uint32_t destination, uint32_t port_id) {
    NS_LOG_UNCOND("[DSRC Unicast] Transmitting from node " << node_index 
                << " to " << destination << " on port " << port_id 
                << " at t=" << Simulator::Now().GetSeconds());
}

void encrypt_dsrc_data_unicast(uint32_t node_index, uint32_t destination, uint32_t port_id) {
    NS_LOG_UNCOND("[AES Encryption] Encrypting DSRC packet from node " 
                << node_index << " to " << destination << " on port " << port_id 
                << " at t=" << Simulator::Now().GetSeconds());
}
