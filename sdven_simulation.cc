#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SDVEN Simulator"); //print a log to indicate simulation start

#define MAX_NODES 1000   // example value

uint32_t max = MAX_NODES;
uint32_t total_size = 0;
uint32_t large = 50000;

double routing_frequency = 0.0;
double optimization_frequency = 1.0;
double optimization_period = 0.0;
double data_transmission_period = 0.0;

uint32_t N_eNodeBs = 0;
uint32_t var = 0;

bool routing_test = false;
uint32_t paper = 0;

uint32_t empty_neighborset[max]; //create an array to store empty neighbor set values for all nodes

void initialize_empty() //function to initialize the empty neighbor set values for all nodes
{
	for (uint32_t i =0; i < max; i++)
	{
		empty_neighborset[i] = large;
	}
}

double sum_of_nodeids = 0; //variable to store the sum of node ids for all nodes in the network, used for optimization calculations

void nodeid_sum() //function to calculate the sum of node ids for all nodes in the network, used for optimization calculations
{
	sum_of_nodeids = 0;
	for (uint32_t i=2;i<total_size+2;i++)
	{
		sum_of_nodeids = sum_of_nodeids + i;
	}
}

uint32_t N_RSUs = 0; //number of RSUs in the network, can be set by the user through command line arguments
uint32_t N_Vehicles = 0; //number of vehicles in the network, can be set by the user through command line arguments

double data_transmission_frequency = 0.33; //frequency at which data is transmitted from vehicles to RSUs, can be set by the user through command line arguments, default is 0.33 Hz (one transmission every 3 seconds)
double simTime = 13.7; //total simulation time in seconds, can be set by the user through command line arguments, default is 13.7 seconds

int architecture = 0; // network architecture type, can be set by the user through command line arguments, default is 0 (Centralized), 1 (Distributed), 2 (Edge)
int lambda = 30; // lambda parameter for the exponential distribution used in the optimization algorithm, can be set by the user through command line arguments, default is 30

int experiment_number = 0; //0 - individual attack, 1 - combined attack
int attack_number = 5; //1 - Attack 1, etc. 2-Attack 2, 3-Attack 3, 4-Attack 4, 5-Attack 5, 6-Combined attack

uint32_t qf = 1; //quantile factor for the optimization algorithm, can be set by the user through command line arguments, default is 1 (no quantile factor applied)
int attack_percentage = 40; //percentage of nodes that are compromised in the attack scenario, can be set by the user through command line arguments, default is 40%


int main (int argc, char *argv[])
{
  NS_LOG_UNCOND ("Starting SDVEN Simulator..."); //print a log to indicate simulation start

  initialize_empty(); //initialize empty neighbor set
  nodeid_sum(); //calculate sum of node ids

  CommandLine cmd;
  cmd.AddValue ("N_RSUs", "N_RSUs", N_RSUs); //add command line argument for number of RSUs
  cmd.AddValue ("N_Vehicles", "N_Vehicles", N_Vehicles); //add command line argument for number of vehicles
  cmd.AddValue ("data_transmission_frequency", "data_transmission_frequency", data_transmission_frequency); //add command line argument for data transmission frequency
  cmd.AddValue ("simTime", "simTime", simTime); //add command line argument for simulation time
  cmd.AddValue ("architecture", "architecture", architecture); //add command line argument for network architecture type
  //cmd.AddValue ("maxspeed", "maxspeed", maxspeed);
  cmd.AddValue ("lambda", "lambda", lambda); //add command line argument for lambda parameter in optimization algorithm
  cmd.AddValue ("attack_number", "attack_number", attack_number); //add command line argument for attack scenario number
  cmd.AddValue ("experiment_number", "experiment_number", experiment_number); //add command line argument for experiment number (individual attack vs combined attack)
  cmd.AddValue ("qf", "qf", qf); //add command line argument for quantile factor in optimization algorithm
  cmd.AddValue ("attack_percentage", "attack_percentage", attack_percentage); //add command line argument for percentage of compromised nodes in attack scenario
  cmd.Parse (argc, argv);	//parse command line arguments

  routing_frequency = data_transmission_frequency; //set routing frequency equal to data transmission frequency for simplicity, can be modified if needed
  N_eNodeBs = 1 + N_Vehicles/320; //calculate number of eNodeBs needed based on number of vehicles, assuming each eNodeB can support up to 320 vehicles, can be modified if needed
  var = N_Vehicles+N_RSUs; //total number of nodes in the network (vehicles + RSUs)
  large=50000;
  optimization_period = 1.0/optimization_frequency;
  data_transmission_period = 1.0/data_transmission_frequency; //datatransmission period. (default is 3 seconds, can be modified by user through command line arguments)
  //APB apb(memblock_key);
  //std::cout << a << "+" << b << "=" << apb.Func(a, b) << std::endl;
  //std::cout << a+2 << "+" << b+2 << "=" << apb.Func(a+2, b+2) << std::endl;
    
  LogComponentEnable ("vanet", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);
    
  clear_RQY();
    
  for (int i = 0; i<total_size+2; i++)
  {
    clear_neighbordata(neighbordata_inst+i);
    clear_controllerdata(con_data_inst+i);
    clear_data_at_nodes(data_at_nodes_inst+i);
    clear_routing_data_at_nodes(routing_data_at_nodes_inst+i-2);
    clear_data_at_manager(data_at_manager_inst+i);
  }
  clear_delta_at_controller(delta_at_controller_inst);
  clear_solution();
  initialize_all_routing_tables();
  
  controller_Node.Create(1);
  management_Node.Create(1); 
  if (routing_test == false)
  {
  	  if(N_Vehicles > 0)
  	{ 
  		Vehicle_Nodes.Create(N_Vehicles); 
  	}
  }
  
  else
  {
  
  	    Vehicle_Nodes.Create(N_Vehicles);
    	double x = 250;
	    MobilityHelper custom_mobility;
	    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	    positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // Custom position for Node 0
	    positionAlloc->Add(Vector(0.0, -x*1, 0.0)); // Custom position for Node 1
	    positionAlloc->Add(Vector(0.0, -x*2, 0.0)); // Custom position for Node 2 
	    /*
	    positionAlloc->Add(Vector(0.0, -x*3, 0.0)); // Custom position for Node 3
	    positionAlloc->Add(Vector(0.0, -x*4, 0.0)); // Custom position for Node 4
	    positionAlloc->Add(Vector(x, -x*4, 0.0)); // Custom position for Node 5 
	    positionAlloc->Add(Vector(2*x, -x*4, 0.0)); // Custom position for Node 6
	    positionAlloc->Add(Vector(3*x, -x*4, 0.0)); // Custom position for Node 7
	    positionAlloc->Add(Vector(3*x, -x*3, 0.0)); // Custom position for Node 8
	    positionAlloc->Add(Vector(3*x, -x*2, 0.0)); // Custom position for Node 9
	    positionAlloc->Add(Vector(3*x, -x, 0.0)); // Custom position for Node 10
	    positionAlloc->Add(Vector(3*x, 0.0, 0.0)); // Custom position for Node 11
	    positionAlloc->Add(Vector(x, 0.0, 0.0)); // Custom position for Node 12
	    positionAlloc->Add(Vector(2*x, 0.0, 0.0)); // Custom position for Node 13
	    positionAlloc->Add(Vector(0.0, x, 0.0)); // Custom position for Node 14
	    positionAlloc->Add(Vector(0.0, x*2, 0.0)); // Custom position for Node 15
	    positionAlloc->Add(Vector(x, x*2, 0.0)); // Custom position for Node 16
	    positionAlloc->Add(Vector(x*2, x*2, 0.0)); // Custom position for Node 17
	    positionAlloc->Add(Vector(x*3, x*2, 0.0)); // Custom position for Node 18
	    positionAlloc->Add(Vector(x*3, x, 0.0)); // Custom position for Node 19
	    positionAlloc->Add(Vector(x, x, 0.0)); // Custom position for Node 20
	    positionAlloc->Add(Vector(2*x, x, 0.0)); // Custom position for Node 21
	    */
	    custom_mobility.SetPositionAllocator(positionAlloc);
	    custom_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
	    custom_mobility.Install(Vehicle_Nodes);
	    //custom_mobility.Install(RSU_Nodes);

	  // Set custom velocity and acceleration for each node
	  
	    for (uint32_t i = 0; i < Vehicle_Nodes.GetN(); i++) 
	    {
	    
	    	Ptr<ConstantVelocityMobilityModel> cvmm = DynamicCast <ConstantVelocityMobilityModel> (Vehicle_Nodes.Get(i)->GetObject<MobilityModel>());
	    	if (i == 0)
	    	{
	    		cvmm->SetVelocity(Vector(0.0, 0.0, 0.0));
	    	}
	    	else if (i == 1)
	    	{
	    		cvmm->SetVelocity(Vector(0, -2.5, 0.0));
	    	}
	   
	    	else if (i == 2)
	    	{
	    		cvmm->SetVelocity(Vector(0, 0, 0.0));
	    	}
	    		
	    	//cvmm->SetAcceleration(Vector(0.0, 0.0, 0.0)); // Custom acceleration for each node

	    }
	  
  }
  

 
  //Install and configure the RSUs  
  if(N_Vehicles > 0)
  {
  	RSU_Nodes.Create (N_Vehicles);  
  }
  
  //configuring the CSMA interface    
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("1000Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (10)));
  
  NodeContainer csma_nodes;
  Ipv4AddressHelper address;
  Ipv4InterfaceContainer csmaInterfaces;
  NetDeviceContainer csmaDevices;
  InternetStackHelper stack;
  
  if (architecture != 1)
  {
	  csma_nodes.Add(RSU_Nodes);
	  csma_nodes.Add(controller_Node);
	  csma_nodes.Add(management_Node);  
	  csmaDevices = csma.Install (csma_nodes);
  	  address.SetBase ("10.1.1.0", "255.255.255.0");
  	  stack.Install (csma_nodes);
  	  csmaInterfaces = address.Assign (csmaDevices);
  } 
  
  AodvHelper aodv;
  InternetStackHelper stack_AODV;
  stack_AODV.SetRoutingHelper(aodv);
  if (architecture == 1)
  {
  	if (paper == 1)
  	{
  		stack_AODV.Install(RSU_Nodes);
  	}
  	if (paper == 0)
  	{
  		stack.Install(RSU_Nodes);
  	}
  }

  // Logic for Edge Architecture
  if (architecture == 2) // Let's define 2 as 'Edge Mode'
  {
      for (uint32_t i = 0; i < N_RSUs; i++)
      {
          // Each RSU is grouped with a local 'Edge Container'
          NodeContainer edgeCluster;
          edgeCluster.Add(RSU_Nodes.Get(i));
          
          // Optionally add a local Micro-Controller/Server per RSU
          // edgeCluster.Add(edgeServerNodes.Get(i)); 

          csmaDevices = csma.Install(edgeCluster);
          
          // Assign unique subnets to each RSU 'Edge Zone' 
          // to prevent global broadcast storms
          std::ostringstream subnet;
          subnet << "10.1." << i << ".0"; 
          address.SetBase (subnet.str().c_str(), "255.255.255.0");
          address.Assign(csmaDevices);
      }
  }

  //configuring the point to point interfaces
  if (N_RSUs > 0)
  {
	  uint32_t length = (N_RSUs-1)/20;
	  PointToPointHelper p2p_horizontal[N_RSUs-length];
	  NetDeviceContainer p2pdevices_horizontal[N_RSUs-length];
	  Ipv4InterfaceContainer p2p_horizontal_interfaces[N_RSUs-length];
	  
	  
	  uint32_t width;
	  if (N_RSUs >20)
	  {
	  	width = N_RSUs - 20;
	  }
	  else
	  {
	  	width = 0;
	  }
	  PointToPointHelper p2p_vertical[width];
	  NetDeviceContainer p2pdevices_vertical[width];
	  Ipv4InterfaceContainer p2p_vertical_interfaces[width];
	  
	   
	   uint32_t z = 0;
	   for (uint32_t i=0; i<N_RSUs; i++)
	   {
		  uint32_t x = (i+1)%20;
		  if ((x != 0) and (i < (N_RSUs-1)))
		  {
		  	p2p_horizontal[z].SetDeviceAttribute ("DataRate", StringValue ("1000Mbps"));
		  	p2p_horizontal[z].SetChannelAttribute ("Delay", TimeValue (MicroSeconds (10)));
		  	p2pdevices_horizontal[z] = p2p_horizontal[z].Install (RSU_Nodes.Get(i), RSU_Nodes.Get(i+1));
		 	string part1 = "20.1.";
		  	string st = to_string(z);
		  	string part3 = ".0";
		  	string baseaddress = part1 + st + part3;
		  	char const * baseaddress_converted = baseaddress.c_str();
		  	Ipv4AddressHelper address;
		 	address.SetBase (Ipv4Address(baseaddress_converted), "255.255.255.0");
		 	p2p_horizontal_interfaces[z] = address.Assign (p2pdevices_horizontal[z]);
		 	z++;
		 }
		 
		 if (i < width)
		 {
		 	p2p_vertical[i].SetDeviceAttribute ("DataRate", StringValue ("1000Mbps"));
		  	p2p_vertical[i].SetChannelAttribute ("Delay", TimeValue (MicroSeconds (10)));
		  	p2pdevices_vertical[i] = p2p_vertical[i].Install (RSU_Nodes.Get(i), RSU_Nodes.Get(i+20));
		 	string part1 = "30.1.";
		  	string st = to_string(i);
		  	string part3 = ".0";
		  	string baseaddress = part1 + st + part3;
		  	char const * baseaddress_converted = baseaddress.c_str();
		  	Ipv4AddressHelper address;
		 	address.SetBase (Ipv4Address(baseaddress_converted), "255.255.255.0");
		 	p2p_vertical_interfaces[i] = address.Assign (p2pdevices_vertical[i]);
		 }	  
	   }
	   

  }

  //installing udp applications in RSUs

  for (uint32_t u=0; u<Vehicle_Nodes.GetN(); u++)
  {
	Ptr <SimpleUdpApplication> udp_app = Create <SimpleUdpApplication> ();
	RSU_Nodes.Get(u)->AddApplication(udp_app);
	RSU_apps.Add(udp_app);
  }
  RSU_apps.Start(Seconds(0.00));
  RSU_apps.Stop(Seconds(simTime));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  Config::SetDefault("ns3::Ipv4GlobalRouting::RespondToInterfaceEvents", BooleanValue(true));
  NodeContainer enbnodes;
  NodeContainer remotehostcontainer;
  Ptr<Node> pgw;
  Ptr<PointToPointEpcHelper> epchelper;
  Ptr<Node> remotehost;
  Ptr<LteHelper> ltehelper;
  InternetStackHelper internet;


  

  //run simulation
  Simulator::Stop (Seconds (25.0));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_LOG_UNCOND ("SDVEN Simulation Finished");
  return 0;
}
