int main(int argc, char *argv[])
{        
    initialize_empty();
    nodeid_sum();   
    
    CommandLine cmd;
    cmd.AddValue ("N_RSUs", "N_RSUs", N_RSUs);
    cmd.AddValue ("N_Vehicles", "N_Vehicles", N_Vehicles);
    cmd.AddValue ("data_transmission_frequency", "data_transmission_frequency", data_transmission_frequency);
    cmd.AddValue ("link_lifetime_threshold", "link_lifetime_threshold", link_lifetime_threshold);
    cmd.AddValue ("simTime", "simTime", simTime);
    cmd.AddValue ("mobility_scenario", "mobility_scenario", mobility_scenario);
    cmd.AddValue ("architecture", "architecture", architecture);
    cmd.AddValue ("maxspeed", "maxspeed", maxspeed);
    cmd.AddValue ("lambda", "lambda", lambda);
    cmd.AddValue ("attack_number", "attack_number", attack_number);
    cmd.AddValue ("experiment_number", "experiment_number", experiment_number);
    cmd.AddValue ("routing_test", "routing_test", routing_test);
    cmd.AddValue ("routing_algorithm", "routing_algorithm", routing_algorithm);
    cmd.AddValue ("qf", "qf", qf);
    cmd.AddValue ("attack_percentage", "attack_percentage", attack_percentage);
    cmd.Parse (argc, argv);	
    
    if (routing_test == true)
    {
     	//N_Vehicles = 22;
     	N_Vehicles = 3;
     	N_RSUs = 0;
     	//flows = 1;
    }
    
    ueBusy.resize(total_size, false);
    ueDLBusy.resize(total_size, false);
	
    
    routing_frequency = data_transmission_frequency;
    N_eNodeBs = 1 + N_Vehicles/320;
    var = N_Vehicles+N_RSUs;
    large=50000;
    optimization_period = 1.0/optimization_frequency;
    data_transmission_period = 1.0/data_transmission_frequency;
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

  if (N_Vehicles > 0)
  {
  	if (architecture != 1)
  	{
		  ltehelper = CreateObject<LteHelper> ();
		  ltehelper->SetAttribute("FadingModel",StringValue("ns3::TraceFadingLossModel"));
		  std::ifstream TraceFile;
		  TraceFile.open("/home/nilmantha/ns-allinone-3.35/ns-3.35/src/lte/model/fading-traces/fading_trace_EVA_60kmph.fad", std::ifstream::in);
		  if(TraceFile.good())
		  {
		  	ltehelper->SetFadingModelAttribute("TraceFilename", StringValue("/home/nilmantha/ns-allinone-3.35/ns-3.35/src/lte/model/fading-traces/fading_trace_EVA_60kmph.fad"));
		  }
		  
		  ltehelper->SetFadingModelAttribute("TraceLength",TimeValue(Seconds(10.0)));
		  ltehelper->SetFadingModelAttribute("SamplesNum",UintegerValue(10000));
		  ltehelper->SetFadingModelAttribute("WindowSize",TimeValue(Seconds(0.5)));
		  ltehelper->SetFadingModelAttribute("RbNum",UintegerValue(100));
		  ltehelper->SetEnbDeviceAttribute("DlEarfcn",UintegerValue(100));
		  ltehelper->SetEnbDeviceAttribute("UlEarfcn",UintegerValue(18100));
		  ltehelper->SetSchedulerType("ns3::RrFfMacScheduler");
		  //ltehelper->SetSpectrumChannelAttribute ("Bandwidth", UintegerValue (10000000));
  		  
		  
		  //epc-evolved packet core for LTE
		  epchelper = CreateObject<PointToPointEpcHelper> ();
		  ltehelper->SetEpcHelper(epchelper);
		  pgw = epchelper->GetPgwNode();
		  
		  //creating remote-host
		  remotehostcontainer.Create(1);
		  remotehost = remotehostcontainer.Get(0);
		  internet.Install(remotehostcontainer);
		  
		  Config::SetDefault("ns3::LteHelper::PathlossModel",StringValue("ns3::FriisPropagationLossModel"));
		  //Config::SetDefault("ns3::LteHelper::PathlossModel",StringValue("ns3::Cost231PropagationLossModel"));
		  Config::SetDefault("ns3::LteHelper::UseIdealRrc",BooleanValue(true));
		  Config::SetDefault("ns3::LteHelper::UsePdschForCqiGeneration",BooleanValue(true));
		  Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled",BooleanValue(true));
		  Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled",BooleanValue(true));
		  
		  ltehelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
          ltehelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));
          // Ensure HARQ is enabled
		  //Config::SetDefault ("ns3::LteSpectrumPhy::HarqEnabled", BooleanValue (true));

		  // Use AM bearer (with retransmission)
		   Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping", StringValue ("RlcAmAlways"));

			// Don’t use ideal RRC with EPC
			Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

		  Config::SetDefault ("ns3::LteRlcAm::PollRetransmitTimer", TimeValue (MilliSeconds (5)));
		  Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (MilliSeconds (4)));
		  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (3)));
		  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (999999));
		  Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999));

		  LogComponentEnable("LteRlc", LOG_LEVEL_INFO);

		  Config::SetDefault("ns3::LteUePhy::TxPower",DoubleValue(33));//maximum ue transmit power of 33 dBm
		  Config::SetDefault("ns3::LteUePhy::EnableUplinkPowerControl",BooleanValue(true));
		  Config::SetDefault("ns3::LteUePowerControl::ClosedLoop",BooleanValue(true));
		  Config::SetDefault("ns3::LteUePowerControl::AccumulationEnabled",BooleanValue(false));
		  //Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping", StringValue ("RlcUmAlways"));
		  //Config::Set ("/NodeList/*/DeviceList/*/LteEnbNetDevice/LteEnbPhy/LteSpectrumPhy/HarqEnabled",BooleanValue (false));

		  
		  /*
		  if ((N_Vehicles+5) < 2)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(2));
		  }
		  else if ((N_Vehicles+5) < 5)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(5));
		  }
		  else if ((N_Vehicles+5) < 10)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(10));
		  }
		    else if ((N_Vehicles+8) < 20)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(20));
		  }
		    else if ((N_Vehicles+20) < 40)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(40));
		  }
		    else if ((N_Vehicles+25) < 80)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(80));
		  }
		    else if ((N_Vehicles+30) < 160)
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(160));
		  }
		  else
		  {
		  	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(320));
		  }
		  */
		  Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",UintegerValue(320));
		  
		  Config::SetDefault("ns3::LteAmc::AmcModel", EnumValue(LteAmc::PiroEW2010));
		  Config::SetDefault("ns3::LteAmc::Ber", DoubleValue(0.00005));
		  enbnodes.Create(N_eNodeBs); 
  	}
  	dsrc_Nodes.Add(Vehicle_Nodes);
  }
  if (N_RSUs > 0)
  {
  	dsrc_Nodes.Add(RSU_Nodes);
  }
  
  if (routing_test == false)
  {
  //vehicular nodes mobility trace files
  string trace_file; 
  if (mobility_scenario == 0) //urban mobility
  {
  	switch(maxspeed)
  	{
  		case (0):
  			trace_file = "/home/nilmantha/mobility/mobility_urban_0.tcl";
  			break;
  		case (10):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_10.tcl";
	  		break;
	  	case (20):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_20.tcl";
	  		break;
	  	case (30):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_30.tcl";
	  		break;
	  	case (40):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_40.tcl";
	  		break;
	  	case (50):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_50.tcl";
	  		break;
	  	case (60):
	  		trace_file = "/home/nilmantha/mobility/mobility_urban_60.tcl";
	  		break;
	  	default:
	  		break;
	 }
   }
   
   if (mobility_scenario == 1) //non-urban mobility
   {
   	switch(maxspeed)
   	{
   		case (0):
   			trace_file = "/home/nilmantha/mobility/mobility_rural_0.tcl";
   	  		break;
   		case (10):
   	  		trace_file = "/home/nilmantha/mobility/mobility_rural_10.tcl";
   	  		break;
   	  	case (20):
	  		trace_file = "/home/nilmantha/mobility/mobility_rural_20.tcl";
	  		break;
	  	case (30):
	   		trace_file = "/home/nilmantha/mobility/mobility_rural_30.tcl";
	   		break;
	   	case (40):
	  		trace_file = "/home/nilmantha/mobility/mobility_rural_40.tcl";
	  		break;
	  	case (50):
	  		trace_file = "/home/nilmantha/mobility/mobility_rural_50.tcl";
	  		break;
	  	case (60):
	  		trace_file = "/home/nilmantha/mobility/mobility_rural_60.tcl";
	  		break;
   	  	case (70):
   	  		trace_file = "/home/nilmantha/mobility/mobility_rural_70.tcl";
   	  		break;
   	  	case (80):
   	  		trace_file = "/home/nilmantha/mobility/mobility_rural_80.tcl";
   	  		break;
   	  	case (90):
   	  		trace_file = "/home/nilmantha/mobility/mobility_rural_90.tcl";
   	  		break;
   	  	case (100):
   	  		trace_file = "/home/nilmantha/mobility/mobility_rural_100.tcl";
   	  		break;
   	  	default:
   	  		break;
	 }
   }
   
   if (mobility_scenario == 2)//highway
   {
   	  switch(maxspeed)
   	  {
   	  	case (0):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_0.tcl";
   	  		break;	
   	  	case (10):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_10.tcl";
   	  		break;
   	  	case (30):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_30.tcl";
   	  		break;
   	  	case (50):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_50.tcl";
   	  		break;
   	  	case (70):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_70.tcl";
   	  		break;
   	  	case (90):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_90.tcl";
   	  		break;
   	  	case (110):
   	  		trace_file = "/home/nilmantha/mobility/mobility_autobahn_110.tcl";
   	  		break;
	 	case (130):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_130.tcl";
	 		break;
	 	case (150):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_150.tcl";
	 		break;
	 	case (170):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_170.tcl";
	 		break;
	 	case (190):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_190.tcl";
	 		break;
	 	case (210):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_210.tcl";
	 		break;
	 	case (230):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_230.tcl";
	 		break;
	 	case (250):
	 		trace_file = "/home/nilmantha/mobility/mobility_autobahn_250.tcl";
	 		break;
	 	default:
	 		break;
	  }
   }


  
  //Ns2MobilityHelper vehicle_mobility  = Ns2MobilityHelper (trace_file);
  
 
  
  MobilityHelper vehicle_mobility2;
  vehicle_mobility2.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  if (N_Vehicles > 0)
  {
  	
  	if (experiment_number != 5)
  	{
  		//vehicle_mobility.Install(Vehicle_Nodes.Begin(),Vehicle_Nodes.End());
  	}
  	else if (experiment_number == 5)
  	{
  		vehicle_mobility2.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (0.0),"MinY", DoubleValue (0.0),"DeltaX", DoubleValue (260.0),"DeltaY", DoubleValue (1000),"GridWidth", UintegerValue (2),"LayoutType", StringValue ("RowFirst"));
  		vehicle_mobility2.Install(Vehicle_Nodes);
  		vehicle_mobility2.Install(RSU_Nodes);
  		
  		for (uint32_t i=0; i<Vehicle_Nodes.GetN(); i++)
	  	{
	  		Ptr<ConstantVelocityMobilityModel> mdl = DynamicCast <ConstantVelocityMobilityModel> (Vehicle_Nodes.Get(i)->GetObject<MobilityModel>());
	  		if (i%2 == 1)
	  		{
	  			mdl->SetVelocity(Vector(1, 0, 0));
	  		}
	  	}
  	}
  }
  }
  
 
  
  int x = ((N_RSUs)/12) + 1;
  double delta_x;
  double delta_y; 
  int lte_base_posx;
  int lte_base_posy;
  int man_base_posx;
  int man_base_posy;
  int con_base_posx;
  int con_base_posy;
  MobilityHelper RSU_mobility;
  RSU_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  vehicle_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  if (mobility_scenario == 0)//urban mobility
  {
  	man_base_posx = 1700;
  	man_base_posy = 1700;
  	con_base_posx = 1600;
  	con_base_posy = 1600;
  	lte_base_posx = 1500;
  	lte_base_posy = 1500;
  	
  	//delta_y = 800/x;
  	delta_y = 400;
  	//delta_x = 1600/5;
  	delta_x = 400;
	  	if (N_RSUs < 13)
	  	{		
	  		RSU_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (750.0),"MinY", DoubleValue (1200.0),"DeltaX", DoubleValue (delta_x),"DeltaY", DoubleValue (delta_y),"GridWidth", UintegerValue (7),"LayoutType", StringValue ("RowFirst"));
	  		vehicle_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (650.0),"MinY", DoubleValue (1000.0), "DeltaX", DoubleValue (delta_x/2),"DeltaY", DoubleValue (delta_y/2),"GridWidth", UintegerValue (5),"LayoutType", StringValue ("RowFirst"));
	  		
	  	}
	  	else
	  	{
	  		RSU_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (750.0),"MinY", DoubleValue (900.0),"DeltaX", DoubleValue (delta_x),"DeltaY", DoubleValue (delta_y),"GridWidth", UintegerValue (7),"LayoutType", StringValue ("RowFirst"));
	  		vehicle_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (650.0),"MinY", DoubleValue (1000.0), "DeltaX", DoubleValue (delta_x/2),"DeltaY", DoubleValue (delta_y),"GridWidth", UintegerValue (14),"LayoutType", StringValue ("RowFirst"));
	  	}
  }
  if(routing_test == false)
  {
  	vehicle_mobility.Install(Vehicle_Nodes);
  	vehicle_mobility.Install(RSU_Nodes);
  }
  
 
  
   if (mobility_scenario == 1)//non-urban mobility
  {

  	man_base_posx = 4600;
  	man_base_posy = 4600;
  	con_base_posx = 4700;
  	con_base_posy = 4700;
  	lte_base_posx = 4500;
  	lte_base_posy = 4500;
  	delta_x = 7000/9;
  	delta_y = 6500/(2*x);
  	RSU_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (1000.0),"MinY", DoubleValue (2000.0),"DeltaX", DoubleValue (delta_x),"DeltaY", DoubleValue (delta_y),"GridWidth", UintegerValue (10),"LayoutType", StringValue ("RowFirst"));  	
  }
  
  if (mobility_scenario == 2)//autobahn mobility
  {
  	man_base_posx = 2200;
  	man_base_posy = 3700;
  	con_base_posx = 2100;
  	con_base_posy = 3600;
  	lte_base_posx = 2000;
  	lte_base_posy = 3500;
  	delta_x = 4000/9;
  	delta_y = 7500/(2*x);
  	RSU_mobility.SetPositionAllocator ("ns3::GridPositionAllocator","MinX", DoubleValue (0),"MinY", DoubleValue (500),"DeltaX", DoubleValue (delta_x),"DeltaY", DoubleValue (delta_y),"GridWidth", UintegerValue (10),"LayoutType", StringValue ("RowFirst"));  	
  }
  
  if (routing_test == true)//routing_test
  {
  	man_base_posx = 500;
  	man_base_posy = 0;
  	con_base_posx = 550;
  	con_base_posy = 0;
  	lte_base_posx = 525;
  	lte_base_posy = 0;
  }
  
  if (N_RSUs > 0)
  {
  	RSU_mobility.Install(RSU_Nodes);
  }
  
  Ptr <Node> nd;
  NodeContainer other_stationary_LTE_nodes;
  if (N_Vehicles > 0)
  {
  	if (architecture != 1)
  	{
	  nd = ns3::NodeList::GetNode(N_Vehicles+N_RSUs+4);
	  other_stationary_LTE_nodes.Add(enbnodes);
	  other_stationary_LTE_nodes.Add(remotehostcontainer);
	  other_stationary_LTE_nodes.Add(pgw);
	  other_stationary_LTE_nodes.Add(epchelper->GetSgwNode());
	  other_stationary_LTE_nodes.Add(nd);
	}
  }
  
  if (architecture != 1)
  {
  
	  MobilityHelper other_stationary_mobility;
	  other_stationary_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
	  other_stationary_mobility.Install(controller_Node);
	  other_stationary_mobility.Install(management_Node);
	  if (N_Vehicles > 0)
	  {
	  	other_stationary_mobility.Install(other_stationary_LTE_nodes);
	  }
	  
	  //srand(time(0));
	  //int lte_base_posx = rand()%3000;
	  //int lte_base_posy = rand()%3000;
	  
	  if (N_Vehicles > 0)
	  {
		  for (uint32_t i=0; i<other_stationary_LTE_nodes.GetN(); i++)
		  {
		  	Ptr<ConstantVelocityMobilityModel> mdl = DynamicCast <ConstantVelocityMobilityModel> (other_stationary_LTE_nodes.Get(i)->GetObject<MobilityModel>());
		  	mdl->SetPosition(Vector(lte_base_posx+10*i, lte_base_posy+10*i, 0));
		  	mdl->SetVelocity(Vector(0, 0, 0));//other stationary LTE nodes
		  }
	  }
  }
  
  if (N_RSUs > 0)
  {
	  for (uint32_t i=0; i<RSU_Nodes.GetN(); i++)
	  {
	  	Ptr<ConstantVelocityMobilityModel> mdl = DynamicCast <ConstantVelocityMobilityModel> (RSU_Nodes.Get(i)->GetObject<MobilityModel>());
	  	mdl->SetVelocity(Vector(0, 0, 0));//RSUs nodes are stationary. Their position is already defined
	  }
  }
  
  //setting the position of controller node 
  //int con_base_posx = rand()%3000;
  //int con_base_posy = rand()%3000;

   if (architecture != 1)
   {
	   Ptr<ConstantVelocityMobilityModel> mdl_controller = DynamicCast <ConstantVelocityMobilityModel> (controller_Node.Get(0)->GetObject<MobilityModel>());
	   mdl_controller->SetPosition(Vector(con_base_posx, con_base_posy, 0));
	   mdl_controller->SetVelocity(Vector(0, 0, 0));//centralized controller placement
	   
	  //setting the position of management node
	  //int man_base_posx = rand()%3000;
	  //int man_base_posy = ;

	   Ptr<ConstantVelocityMobilityModel> mdl_management = DynamicCast <ConstantVelocityMobilityModel> (management_Node.Get(0)->GetObject<MobilityModel>());
	   mdl_management->SetPosition(Vector(man_base_posx, man_base_posy, 0));
	   mdl_management->SetVelocity(Vector(0, 0, 0));//centralized management server placement
   }
  
  Ipv4StaticRoutingHelper ipv4routinghelper_con;
  if (N_Vehicles > 0)
  {
  	if (architecture != 1)
  	{
		  //point to point connection for pgw and remotehost 
		  PointToPointHelper p2ph;
		  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1000Mb/s")));
		  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
		  p2ph.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
		  NetDeviceContainer internetdevices = p2ph.Install(pgw,remotehost);
		  Ipv4AddressHelper ipv4h;
		  ipv4h.SetBase("1.0.0.0","255.0.0.0");
		  Ipv4InterfaceContainer internetipfaces = ipv4h.Assign(internetdevices);
		  //Ipv4Address remoteHostAddr = internetipfaces.GetAddress (1);
		  
		  //point to point connction for controller
		  PointToPointHelper p2p_controllers;
		  p2p_controllers.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1000Mb/s")));
		  p2p_controllers.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
		  NetDeviceContainer p2pcontroller_devices = p2p_controllers.Install(pgw,controller_Node.Get(0));
		  Ipv4AddressHelper ipv4helper;
		  ipv4helper.SetBase("40.1.1.0","255.255.255.0");
		  Ipv4InterfaceContainer controller_interfaces = ipv4helper.Assign(p2pcontroller_devices);
		  
		  //point to point connection for management server
		  PointToPointHelper p2p_management;
		  p2p_management.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1000Mb/s")));
		  p2p_management.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));
		  NetDeviceContainer p2pmanagement_devices = p2p_management.Install(pgw,management_Node.Get(0));
		  Ipv4AddressHelper ipv4helper2;
		  ipv4helper2.SetBase("80.1.1.0","255.255.255.0");
		  Ipv4InterfaceContainer management_interfaces = ipv4helper2.Assign(p2pmanagement_devices);
		  
		  
		  Ipv4StaticRoutingHelper ipv4routinghelper;
		  Ptr<Ipv4StaticRouting> remotehoststaticrouting = ipv4routinghelper.GetStaticRouting(remotehost->GetObject<Ipv4>());
		  remotehoststaticrouting->AddNetworkRouteTo (Ipv4Address("7.0.0.0"),Ipv4Mask("255.0.0.0"),1);

		  Ptr<Ipv4StaticRouting> controller_staticrouting = ipv4routinghelper_con.GetStaticRouting(controller_Node.Get(0)->GetObject<Ipv4>());
		  controller_staticrouting->AddNetworkRouteTo (Ipv4Address("7.0.0.0"),Ipv4Mask("255.0.0.0"),2);

		  Ipv4StaticRoutingHelper ipv4routinghelper_man;
		  Ptr<Ipv4StaticRouting> management_staticrouting = ipv4routinghelper_man.GetStaticRouting(management_Node.Get(0)->GetObject<Ipv4>());
		  management_staticrouting->AddNetworkRouteTo (Ipv4Address("7.0.0.0"),Ipv4Mask("255.0.0.0"),2);
	}
}

  //broadcast data in RSU nodes
  /*
  for (uint32_t t=0 ; t<simTime-1; t++)
  {
	  for (uint32_t u=0; u<RSU_Nodes.GetN(); u++)
	  {
	  	Ptr <Node> nu = DynamicCast <Node> (RSU_Nodes.Get(u));	
	  	Ptr <SimpleUdpApplication> udp_app = DynamicCast <SimpleUdpApplication> (RSU_apps.Get(u));
		Simulator::Schedule(Seconds(t+1+0.0001*u),p2p_data_broadcast, udp_app, nu);
   	  }
  }
  */
  
  //broadcast metadata in RSU nodes
  /*
  for (uint32_t t=0 ; t<simTime-1; t++)
  {
	  for (uint32_t u=0; u<RSU_Nodes.GetN(); u++)
	  {
	  	Ptr <Node> nu = DynamicCast <Node> (RSU_Nodes.Get(u));	
	  	Ptr <SimpleUdpApplication> udp_app = DynamicCast <SimpleUdpApplication> (RSU_apps.Get(u));
		Simulator::Schedule(Seconds(t+1+0.0001*u),p2p_metadata_broadcast, udp_app, nu);
   	  }
  }
  */
 
  //YansWifiHelper to create WifiNetDevice
  
  switch(qf)
  {
  	//Background
  	case (0):
 
		flow_packet_size = 750;
		CW_min = 15;
		CW_max = 1023;
		SIFS = 12;
		T_slot = 20.0;
		AIFSN = 9;
		B_max = log2(1+(CW_max/CW_min));
		latency_max = 0.100;
		loss_max = 0.001;
		AIFS = SIFS + (AIFSN*T_slot);
  		break;
  	//Best effort	
  	case (1):
  		flow_packet_size = 750;
		CW_min = 15;
		CW_max = 127;
		SIFS = 12;
		T_slot = 20.0;
		AIFSN = 6;
		B_max = log2(1+(CW_max/CW_min));
		latency_max = 0.050;
		loss_max = 0.001;
		AIFS = SIFS + (AIFSN*T_slot);
  		break;
  	//Video
  	case (2):
  		flow_packet_size = 750;
		CW_min = 7;
		CW_max = 31;
		SIFS = 12;
		T_slot = 20.0;
		AIFSN = 3;
		B_max = log2(1+(CW_max/CW_min));
		latency_max = 0.025;
		loss_max = 0.005;
		AIFS = SIFS + (AIFSN*T_slot);
  		break;
  	//Audio
 	case (3):
  		flow_packet_size = 750;
		CW_min = 3;
		CW_max = 7;
		SIFS = 12;
		T_slot = 20.0;
		AIFSN = 2;
		B_max = log2(1+(CW_max/CW_min));
		latency_max = 0.0125;
		loss_max = 0.005;
		AIFS = SIFS + (AIFSN*T_slot);
  		break;
 
 	default:
 		break;
  }
  
  YansWifiChannelHelper channel;
  YansWifiChannelHelper channel_172;
  YansWifiChannelHelper channel_174;
  YansWifiChannelHelper channel_176;
  YansWifiChannelHelper channel_180;
  YansWifiChannelHelper channel_182;
  YansWifiChannelHelper channel_184;
  
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_172.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_174.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_176.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_180.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_182.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed
  channel_184.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");//set propagation delay model as constant speed

  
  if(mobility_scenario == 0)
  {
  	channel.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_172.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_174.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_176.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_180.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_182.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	channel_184.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For urban -v2v
  	//channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  	//channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  }
  if ((mobility_scenario==1) or (mobility_scenario==2))
  {
  	//channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  	channel.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_172.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_174.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_176.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_180.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_182.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	channel_184.AddPropagationLoss("ns3::Cost231PropagationLossModel");//For sub-urban and highway
  	
  	//channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  }
  
  //Physical layer helper for wave
  YansWifiPhyHelper Phy;
  YansWifiPhyHelper Phy_172;
  YansWifiPhyHelper Phy_174;
  YansWifiPhyHelper Phy_176;
  YansWifiPhyHelper Phy_180;
  YansWifiPhyHelper Phy_182;
  YansWifiPhyHelper Phy_184;
  
  Phy.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_172.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_172.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_172.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_174.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_174.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_174.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_176.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_176.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_176.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_180.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_180.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_180.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_182.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_182.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_182.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  Phy_184.SetErrorRateModel("ns3::NistErrorRateModel");
  Phy_184.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  Phy_184.Set("TxPowerLevels", UintegerValue(2));//number of transmission power levels
  if (mobility_scenario == 0)
  {
  	Phy.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_172.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_172.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_174.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_174.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_176.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_176.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_180.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_180.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_182.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_182.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_184.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_184.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  }
  if (mobility_scenario == 1)
  {
  	Phy.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = non-urban
  	Phy_172.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_172.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_174.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_174.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_176.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_176.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_180.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_180.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_182.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_182.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_184.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_184.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  }
  if (mobility_scenario == 2)
  {
  	Phy.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 44 dBm = highway
  	Phy_172.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_172.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_174.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_174.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_176.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_176.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_180.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_180.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_182.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_182.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  	Phy_184.Set ("TxPowerStart", DoubleValue (41));//TxPowerStart is the minimum power
  	Phy_184.Set ("TxPowerEnd", DoubleValue (41));//TxPowerEnd is the maximum power. 41 dBm = urban
  }
  Phy.Set ("Frequency", UintegerValue(5890));//center frequency
  Phy.Set ("ChannelNumber", UintegerValue(178));//channel number
  Phy_172.Set ("Frequency", UintegerValue(5860));//center frequency
  Phy_172.Set ("ChannelNumber", UintegerValue(172));//channel number
  Phy_174.Set ("Frequency", UintegerValue(5870));//center frequency
  Phy_174.Set ("ChannelNumber", UintegerValue(174));//channel number
  Phy_176.Set ("Frequency", UintegerValue(5880));//center frequency
  Phy_176.Set ("ChannelNumber", UintegerValue(176));//channel number
  Phy_180.Set ("Frequency", UintegerValue(5900));//center frequency
  Phy_180.Set ("ChannelNumber", UintegerValue(180));//channel number
  Phy_182.Set ("Frequency", UintegerValue(5910));//center frequency
  Phy_182.Set ("ChannelNumber", UintegerValue(182));//channel number
  Phy_184.Set ("Frequency", UintegerValue(5920));//center frequency
  Phy_184.Set ("ChannelNumber", UintegerValue(184));//channel number
  
  Phy.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy.Set ("Primary20MHzIndex", UintegerValue(3));//0 for least
  Phy.Set ("RxSensitivity", DoubleValue(-105));//
  Phy.Set ("TxGain", DoubleValue(0));//
  Phy.Set ("RxGain", DoubleValue(0));//
  Phy.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy.Set ("Antennas", UintegerValue(1));//
  Phy.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
 
  Phy_172.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_172.Set ("Primary20MHzIndex", UintegerValue(0));//0 for least
  Phy_172.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_172.Set ("TxGain", DoubleValue(0));//
  Phy_172.Set ("RxGain", DoubleValue(0));//
  Phy_172.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_172.Set ("Antennas", UintegerValue(1));//
  Phy_172.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_172.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_172.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
  
  Phy_174.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_174.Set ("Primary20MHzIndex", UintegerValue(1));//0 for least
  Phy_174.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_174.Set ("TxGain", DoubleValue(0));//
  Phy_174.Set ("RxGain", DoubleValue(0));//
  Phy_174.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_174.Set ("Antennas", UintegerValue(1));//
  Phy_174.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_174.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_174.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
 
  Phy_176.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_176.Set ("Primary20MHzIndex", UintegerValue(2));//0 for least
  Phy_176.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_176.Set ("TxGain", DoubleValue(0));//
  Phy_176.Set ("RxGain", DoubleValue(0));//
  Phy_176.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_176.Set ("Antennas", UintegerValue(1));//
  Phy_176.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_176.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_176.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
  
  Phy_180.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_180.Set ("Primary20MHzIndex", UintegerValue(4));//0 for least
  Phy_180.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_180.Set ("TxGain", DoubleValue(0));//
  Phy_180.Set ("RxGain", DoubleValue(0));//
  Phy_180.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_180.Set ("Antennas", UintegerValue(1));//
  Phy_180.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_180.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_180.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
  
  Phy_182.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_182.Set ("Primary20MHzIndex", UintegerValue(5));//0 for least
  Phy_182.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_182.Set ("TxGain", DoubleValue(0));//
  Phy_182.Set ("RxGain", DoubleValue(0));//
  Phy_182.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_182.Set ("Antennas", UintegerValue(1));//
  Phy_182.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_182.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_182.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
  
  Phy_184.Set ("ChannelWidth", UintegerValue(10));//channel width
  //Phy_184.Set ("Primary20MHzIndex", UintegerValue(6));//0 for least
  Phy_184.Set ("RxSensitivity", DoubleValue(-105));//
  Phy_184.Set ("TxGain", DoubleValue(0));//
  Phy_184.Set ("RxGain", DoubleValue(0));//
  Phy_184.Set ("RxNoiseFigure", DoubleValue(0));//
  Phy_184.Set ("Antennas", UintegerValue(1));//
  Phy_184.Set ("PowerDensityLimit", DoubleValue(100));// 
  Phy_184.Set ("Slot", ns3::TimeValue(ns3::MicroSeconds(T_slot)));//set slot time
  Phy_184.Set ("Sifs", ns3::TimeValue(ns3::MicroSeconds(SIFS)));//set SIFS
  
  
  //Phy.Set ("ChannelSettings", StringValue ("{176, 10, BAND_5GHZ, 0}"));
  //Config::SetDefault ("ns3::WifiPhy::ChannelSettings", StringValue ("{176, 10, BAND_5GHZ, 0}"));
  Phy.SetChannel (channel.Create ());
  Phy_172.SetChannel (channel_172.Create ());
  Phy_174.SetChannel (channel_174.Create ());
  Phy_176.SetChannel (channel_176.Create ());
  Phy_180.SetChannel (channel_180.Create ());
  Phy_182.SetChannel (channel_182.Create ());
  Phy_184.SetChannel (channel_184.Create ());
  
  
  //Set the number of power levels.
  
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/TxPowerLevels", ns3::UintegerValue(7)); 

  
  //setting up the MAC layer
  
  Ssid ssid = Ssid ("ns-3-ssid");
  WifiMacHelper Mac;
  Mac.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211p);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));
						
  Ssid ssid_172 = Ssid ("ns-3-ssid-172");
  WifiMacHelper Mac_172;
  Mac_172.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_172),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_172;
  wifi_172.SetStandard (WIFI_STANDARD_80211p);
  wifi_172.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW5MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW5MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));
						
  Ssid ssid_174 = Ssid ("ns-3-ssid-174");
  WifiMacHelper Mac_174;
  Mac_174.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_174),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_174;
  wifi_174.SetStandard (WIFI_STANDARD_80211p);
  wifi_174.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));
 
  Ssid ssid_176 = Ssid ("ns-3-ssid-176");
  WifiMacHelper Mac_176;
  Mac_176.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_176),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_176;
  wifi_176.SetStandard (WIFI_STANDARD_80211p);
  wifi_176.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));

  Ssid ssid_180 = Ssid ("ns-3-ssid-180");
  WifiMacHelper Mac_180;
  Mac_180.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_180),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_180;
  wifi_180.SetStandard (WIFI_STANDARD_80211p);
  wifi_180.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));
  
  Ssid ssid_182 = Ssid ("ns-3-ssid-182");
  WifiMacHelper Mac_182;
  Mac_182.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_182),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_182;
  wifi_182.SetStandard (WIFI_STANDARD_80211p);
  wifi_182.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));

  Ssid ssid_184 = Ssid ("ns-3-ssid-184");
  WifiMacHelper Mac_184;
  Mac_184.SetType("ns3::AdhocWifiMac","Ssid", SsidValue (ssid_184),"QosSupported", BooleanValue(true));
  

  WifiHelper wifi_184;
  wifi_184.SetStandard (WIFI_STANDARD_80211p);
  wifi_184.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate12MbpsBW10MHz"),
  						"ControlMode",StringValue ("OfdmRate12MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("Invalid-WifiMode"),
						"MaxSsrc",UintegerValue(B_max),
						"MaxSlrc",UintegerValue(B_max),
						"RtsCtsThreshold",UintegerValue(1000));
						

  //MaxSsrc - maximum retransmission for packets lower than RTSCTS threshold.
  //MaxSlrc - maximum retransmissions for packets larger than RTSCTS threshold.
  
 
 
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/Queue/Mode",EnumValue(1000000*qf));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/Queue/MaxPackets",UintegerValue(50000));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/Queue/MaxBytes",UintegerValue(5000000));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/Queue/MaxDelay",TimeValue(MilliSeconds(500000)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/MinCw",UintegerValue(CW_min));//set minimum contention window
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/MaxCw",UintegerValue(CW_max));//set maximum contention window
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/Aifsn",UintegerValue(AIFSN));//set AIFSN
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::AdhocWifiMac/DcaTxop/TxopLimit",TimeValue(NanoSeconds(5000000)));
  Config::SetDefault ("ns3::WifiMacQueue::MaxSize", QueueSizeValue (QueueSize ("10000p")));
  Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue (Seconds (100)));
  //Config::SetDefault("ns3::QueueBase::MaxPackets", UintegerValue(1000));
  //Config::SetDefault("ns3::QueueBase::MaxBytes", UintegerValue(1 << 20)); // 1 MB

  
  //Mac.SetAifs(ns3::UintegerValue(AIFS));//set AIFS
 
  wifidevices = wifi.Install (Phy, Mac, dsrc_Nodes);
  wifidevices_172 = wifi_172.Install (Phy_172, Mac_172, dsrc_Nodes);
  wifidevices_174 = wifi_174.Install (Phy_174, Mac_174, dsrc_Nodes);
  wifidevices_176 = wifi_176.Install (Phy_176, Mac_176, dsrc_Nodes);
  wifidevices_180 = wifi_180.Install (Phy_180, Mac_180, dsrc_Nodes);
  wifidevices_182 = wifi_182.Install (Phy_182, Mac_182, dsrc_Nodes);
  wifidevices_184 = wifi_184.Install (Phy_184, Mac_184, dsrc_Nodes);
  
  NetDeviceContainer enbdevices;
  NetDeviceContainer uedevices;
  NodeContainer LTE_Nodes;
  LTE_Nodes.Add(controller_Node);
  LTE_Nodes.Add(management_Node);
  
  if (N_Vehicles >0)
  {
  	if (architecture != 1)
  	{
	  	  enbdevices = ltehelper->InstallEnbDevice(enbnodes);
		  uedevices = ltehelper->InstallUeDevice(Vehicle_Nodes);
		  
		  
		  
	  	  internet.Install(Vehicle_Nodes);
	  	  Ipv4InterfaceContainer ueIpinterface;
	  	  ueIpinterface = epchelper->AssignUeIpv4Address(uedevices);

	  	  //Assign default gateway of the UEs, attach ues to enodebs.
		  for(uint32_t i=0; i<uedevices.GetN();i++)
		  {
		  	Ptr <Node> uenode = Vehicle_Nodes.Get(i);
		  	Ptr <Ipv4StaticRouting> ueStaticRouting = ipv4routinghelper_con.GetStaticRouting(uenode->GetObject<Ipv4>());//get the ip
		  	ueStaticRouting->SetDefaultRoute (epchelper->GetUeDefaultGatewayAddress(),1);  	
		  	uint32_t x = N_Vehicles/N_eNodeBs;
		  	uint32_t index = i/x;
		  	ltehelper->Attach (uedevices.Get(i), enbdevices.Get(index));
		  }
		  //tft stands for traffic flow template
		  Ptr <EpcTft> tft = Create <EpcTft> ();
		  EpcTft::PacketFilter pf;
		  pf.localPortStart =1234;
		  pf.localPortEnd = 1234;
		  tft->Add(pf);
		  ltehelper->ActivateDedicatedEpsBearer(uedevices, EpsBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT), tft); 
	  	  //Install UDP applications in the  controller, management node, vehicular nodes,
	  	  LTE_Nodes.Add(Vehicle_Nodes);
		  
		  //uint16_t protocolip = 0x86DD;//ethertype for Ipv4 is set here.
		  
		  ltehelper->EnablePhyTraces();
		  ltehelper->EnableMacTraces();
		  ltehelper->EnableRlcTraces();
		  
		  //Phy.EnablePcap ("WaveTest", wifidevices);
	}
	if (architecture == 1)
	{
		if (paper == 1)
		{
			stack_AODV.Install(Vehicle_Nodes);
		}
		if (paper == 0)
		{
			stack.Install(Vehicle_Nodes);
		}
	}
}

  Ipv4AddressHelper address_dsrc;
  Ipv4AddressHelper address_dsrc_172;
  Ipv4AddressHelper address_dsrc_174;
  Ipv4AddressHelper address_dsrc_176;
  Ipv4AddressHelper address_dsrc_180;
  Ipv4AddressHelper address_dsrc_182;
  Ipv4AddressHelper address_dsrc_184;
 
  Ipv4InterfaceContainer dsrc_interfaces;
  Ipv4InterfaceContainer dsrc_interfaces_172;
  Ipv4InterfaceContainer dsrc_interfaces_174;
  Ipv4InterfaceContainer dsrc_interfaces_176;
  Ipv4InterfaceContainer dsrc_interfaces_180;
  Ipv4InterfaceContainer dsrc_interfaces_182;
  Ipv4InterfaceContainer dsrc_interfaces_184;
  address_dsrc.SetBase ("3.0.0.0", "255.0.0.0");
  dsrc_interfaces = address_dsrc.Assign (wifidevices);
  address_dsrc_172.SetBase ("4.0.0.0", "255.0.0.0");
  dsrc_interfaces_172 = address_dsrc_172.Assign (wifidevices_172);
  address_dsrc_174.SetBase ("5.0.0.0", "255.0.0.0");
  dsrc_interfaces_174 = address_dsrc_174.Assign (wifidevices_174);
  address_dsrc_176.SetBase ("6.0.0.0", "255.0.0.0");
  dsrc_interfaces_176 = address_dsrc_176.Assign (wifidevices_176);
  address_dsrc_180.SetBase ("11.0.0.0", "255.0.0.0");
  dsrc_interfaces_180 = address_dsrc_180.Assign (wifidevices_180);
  address_dsrc_182.SetBase ("8.0.0.0", "255.0.0.0");
  dsrc_interfaces_182 = address_dsrc_182.Assign (wifidevices_182);
  address_dsrc_184.SetBase ("9.0.0.0", "255.0.0.0");
  dsrc_interfaces_184 = address_dsrc_184.Assign (wifidevices_184);
 
 reset_LLDP_counters();
 Simulator::Schedule(Seconds(0.90), connect_to_uplink);
 Simulator::Schedule(Seconds(0.90), connect_to_uplink_RSU);
 Simulator::Schedule(Seconds(0.90), connect_to_downlink);
 Simulator::Schedule(Seconds(0.90), connect_to_downlink_RSU);
 
uint32_t ue0_index = 0;                   // Node index of UE0
uint32_t dest_index = 0;                  // Controller node index
uint32_t port_id   = 7777;                // Or whichever port you use for uplink
Ptr<Packet> testPkt = Create<Packet>(100); // 100-byte test packet


Simulator::Schedule(Seconds(0.93), &send_LTE_LLDP_packetin_uplink_alone,
                    ue0_index,
                    dest_index,
                    port_id,
                    testPkt);
                    
Simulator::Schedule(Seconds(0.95), &send_Ethernet_LLDP_packetin_uplink_alone,
                    ue0_index,
                    dest_index,
                    port_id,
                    testPkt);
                    
                   
Simulator::Schedule(Seconds(0.97), &Trysend_Ethernet_LLDP_packetout_downlink_alone,
                    ue0_index,
                    dest_index,
                    port_id,
                    testPkt);
                    
Simulator::Schedule(Seconds(1.0), &Trysend_LTE_LLDP_packetout_downlink_alone,
                    ue0_index,
                    dest_index,
                    port_id,
                    testPkt);
                    
                    
cout<<"Routing algorithm is "<<routing_algorithm<<"experiment number is "<<experiment_number<<"attack number is "<<attack_number<<"attack percentage is"<<attack_percentage<<endl;
 
 if (architecture != 1)
 {
	 for (uint32_t u=0; u<LTE_Nodes.GetN(); u++)
	 {
	  	Ptr <SimpleUdpApplication> udp_app = Create <SimpleUdpApplication> ();
		LTE_Nodes.Get(u)->AddApplication(udp_app);
		apps.Add(udp_app);
	 }
	 apps.Start(Seconds(0.00));
	 apps.Stop(Seconds(simTime)); 
 }
 
 if (architecture == 1)
 {
	 for (uint32_t u=0; u<Vehicle_Nodes.GetN(); u++)
	 {
	  	Ptr <SimpleUdpApplication> udp_app = Create <SimpleUdpApplication> ();
		Vehicle_Nodes.Get(u)->AddApplication(udp_app);
		apps.Add(udp_app);
	 }
	 apps.Start(Seconds(0.00));
	 apps.Stop(Seconds(simTime)); 
 }

 if (architecture == 0)//centralized architecture
 { 
		Simulator::Schedule(Seconds(0.0), update_mobility);
///*
	  //unicast its own data packets to management node from vehicles.
	    for (uint32_t i=0;i<total_size;i++)
		{
			uplink_last[i] = 0.0;
			last_downlink[i] = 0.0;
		}
        
	 
	  	//Set initial true location
	  	 for (uint32_t i=0; i<wifidevices.GetN() ; i++)
		 {     
				 Ptr <Node> node_copy = DynamicCast <Node> (Vehicle_Nodes.Get(i));
				 Ptr<ConstantVelocityMobilityModel> mdl = DynamicCast <ConstantVelocityMobilityModel> (node_copy->GetObject<MobilityModel>());
				 Vector posi = mdl->GetPosition();
				 Simulator::Schedule (Seconds(1.0), set_last_true_location_and_timestamp, i, posi);
		 }
//*/	  	
		for (double t=6.70; t<simTime-1; t=t+data_transmission_period)//All official data transmissions begin at t=0
		{	
			//Go over all the wifi devices
			//if (routing_algorithm != 5)
	  		//{   
	  		      
///*			  
				  //if(routing_algorithm == 4)
				  //{
					  for (uint32_t i=0; i<wifidevices.GetN() ; i++)
					  {     
						 Simulator::Schedule (Seconds (t+0.0002*i), centralized_dsrc_data_broadcast, wifidevices.Get (i), dsrc_Nodes.Get(i), i, 0);
						 Simulator::Schedule (Seconds(t-0.0001), setting_last_true_location_and_timestamp, i);
					  }
			      //}
				  Simulator::Schedule (Seconds (t), set_dsrc_initial_timestamp);
				  if(routing_algorithm == 4)
				  {
					Simulator::Schedule(Seconds(t+0.25), verify_location);
				  }
				  Simulator::Schedule(Seconds(t-0.002), update_mobility);
				  
//*/
			//}
			  
		}		
///*		
		
	  	for (double t=6.707 ; t<simTime-1; t=t+data_transmission_period)
	  	{
			  for (uint32_t u=0; u<Vehicle_Nodes.GetN(); u++)
			  {
					Ptr <SimpleUdpApplication> udp_app = DynamicCast <SimpleUdpApplication> (apps.Get(u+2));
					Simulator::Schedule(Seconds(t+0.000025*u),send_LTE_data_alone,udp_app,Vehicle_Nodes.Get(u),management_Node.Get(0), u);	
			  }
			  Simulator::Schedule (Seconds (t), set_lte_initial_timestamp);	  
	  	}
  
//*/	  	
	  	/*
	  	//unicast data from RSU nodes alone to management server
	  	if (N_RSUs > 0)
	  	{
			for (double t=6.707 ; t<simTime-1; t=t+data_transmission_period)
			{
				  //if (routing_algorithm != 5)
	  			  //{
					  for (uint32_t u=0; u<RSU_Nodes.GetN(); u++)
					  {
					  	Ptr <Node> nu = DynamicCast <Node> (RSU_Nodes.Get(u));	
					  	Ptr <SimpleUdpApplication> udp_app = DynamicCast <SimpleUdpApplication> (RSU_apps.Get(u));
						Simulator::Schedule(Seconds(t+0.000050*u),RSU_dataunicast_alone, udp_app, nu, management_Node.Get(0));
				   	  }
				   	  Simulator::Schedule (Seconds (t), set_ethernet_initial_timestamp);
				  //}
			   	  
			 }
		 }
		 */
		 
	  
	  //else
	  //{
///*
	  	if (experiment_number != 5)
	  	{			
		  	//DSRC flow instantiation
		  	
		  	double t0 = 6.000;
		  	cout<<t0<<endl;			
		       //unicast metadata from RSU nodes to management server - only in the first data cycle
			declare_attack_states();
			declare_attackers();
			assign_controllers();
			test_boolean();			
			
			if(routing_algorithm == 4)
			{
				initialize_blockchain();
				assign_basic_keys();
				//call_blockchain();
				initialize_server();
				
			}
			
			//Simulator::Schedule (Seconds (7.400), reset_packet_timestamps);
			//Simulator::Schedule(Seconds(7.400),generate_F_and_E);
			
			for(uint32_t i=0;i<2*flows;i++)
			{
				(demanding_flow_struct_controller_inst+i)->f_size = 2;
			}
			
			Simulator::Schedule(Seconds(0.0010),generate_B_matrix);
			Simulator::Schedule(Seconds(0.0010),generate_F_and_E);
//*/
///*			
			if (N_Vehicles > 0)
			{
	
			  	for (double t=t0+1.000 ; t<simTime-1; t=t+data_transmission_period)
			  	{	
			  		  Simulator::Schedule(Seconds(t),clear_delta_at_nodes, delta_at_nodes_inst);
			  		  
//*/			  		  
			  		  /*
					  for (uint32_t u=0; u<Vehicle_Nodes.GetN(); u++)
					  {
					  	Ptr <SimpleUdpApplication> udp_app = DynamicCast <SimpleUdpApplication> (apps.Get(u+2));
						Simulator::Schedule(Seconds(t+0.000025*u),send_LTE_metadata_uplink_alone,udp_app,Vehicle_Nodes.Get(u),management_Node.Get(0), u);
					  }
					  */
					  //calculate the routing solution
					  //unicast the solution back to nodes
					  
///*					  
				      Simulator::Schedule(Seconds(t), generate_F_and_E);
				      if(routing_algorithm == 4)
					  {	
						    Simulator::Schedule(Seconds(t-0.001), generate_adjacency_matrix);
							Simulator::Schedule(Seconds(t), initialize_bmatrix);
					  }
					  Simulator::Schedule(Seconds(t+0.034500),run_optimization_link_lifetime);
					  Simulator::Schedule(Seconds(t+0.034800),update_flows);
					  
					  Simulator::Schedule(Seconds(t+0.034900+(2*(flows+1)*0.000050)),filter_flows);
					  Simulator::Schedule(Seconds(t+0.034900+(2*(flows+1)*0.000050)),LDA_security, " is_controller=true create_security_manager_con=true netsize=2 node_id=0 generate_dig_rsa_key_pair=true generate_own_aes_key=true sign_data=true verify_signature=true initiate_session1=true initiate_session2=true create_hmac_global=true create_hmac_set1=true create_hmac_set2=true verify_key_expiry=true is_node=true pid=1 get_rsa_keys=true encrypt_rsa=true set_aes_key_for_pair=true encrypt_controller_data=true decrypt_controller_data=true generate_global_HMAC_secret_key=true get_digital_public_key=true get_session_HMAC_1=true get_session_HMAC_2=true create_global_HMAC_node=true decrypt_rsa=true get_aes_keys=true encrypt_aes_node_pair=true decrypt_aes_node_pair=true");
					  Simulator::Schedule(Seconds(t+0.034900), reset_LLDP_received_count);
///*					  
					  switch(routing_algorithm)
				   	  {
				   	  	//port-based
				   	  	case (0):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_port_based);
				   	  		break;
				   	  	//normal_LLDP
				   	  	case (1):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_normal_LLDP);
				   	  		break;
				   	  	//Pure-crypto
				   	  	case (2):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_pure_crypto);
				   	  		break;
				   	  	//Link guard.
				   	 	case (3):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_link_guard);
				   	  		break;
				   	  	//proposed
				   	  	case (4):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_proposed_LLDP);
				   	  		break;
				   	  	//HELLO packets
				   	  	case (5):
				   	  		Simulator::Schedule(Seconds(t+0.035900),run_HELLO);
				   	  		break;
					 	default:
					 		break;
					  }
					 
					  if(routing_algorithm != 5)
					  {
					  	Simulator::Schedule(Seconds(t+0.036000),transmit_delta_values);
					  }

					  //Simulator::Schedule(Seconds(t+0.099500),initialize_flow_counters);
					  //Simulator::Schedule(Seconds(t+0.100000),initiate_all_flows); 
					  Simulator::Schedule(Seconds(t+data_transmission_period-0.002-0.3),calculate_performance_evaluation_metricsLLDP);
					  Simulator::Schedule(Seconds(t+data_transmission_period-0.001-0.3),reset_LLDP_counters);
					  //Simulator::Schedule(Seconds(t+data_transmission_period-1.582),CallBWTRCBFromNS3, 0);
					  Simulator::Schedule (Seconds (t), reset_packet_timestamps);
					  Simulator::Schedule (Seconds (t), reset_confusion_matrix); 
					  Simulator::Schedule (Seconds (t), set_lte_initial_timestamp);
					  Simulator::Schedule (Seconds (t), set_ethernet_initial_timestamp);
					  Simulator::Schedule (Seconds (t+0.035900), set_LLDP_initial_timestamp);
//*/
			  	}
		  	}
//*/
	  	
///*
		        //DSRC nodes data unicast 
	    
	  		double t_check;
	  		if((routing_algorithm == 5))
	  		{
				t_check = 7.80;
			}
			else
			{
				t_check = 9.30;
			}
			
			
			for (double t=t_check; t<simTime-1; t=t+data_transmission_period)//All official data transmissions begin at t=0
			{	
				  //Go over all the wifi devices
				 
				 for (uint32_t i=0; i<total_size; i++)
				 {  
					if(routing_algorithm ==5)
					{ 
						for(uint32_t j=0; j<total_size;j++)
						{
							 int o = 31*i+7*j;
							 srand(o);
							 double rand_delay1 = 0.000001*(rand()%100);
							 int p = 27*i+11*j;
							 srand(p);
							 double rand_delay2 = 0.000001*(rand()%100);
							 double delta = 0.005; // base spacing
							 double delay1 = t + delta * (i * total_size + j) + rand_delay1;
							 double delay2 = t + delta * (i * total_size + j) + rand_delay2;
							Simulator::Schedule (Seconds (delay1), centralized_dsrc_data_unicast, dsrc_Nodes.Get(i), i, j, 0);
							Simulator::Schedule (Seconds (delay2), centralized_dsrc_data_unicast, dsrc_Nodes.Get(i), i, j, 1);
						}
					}
					if(routing_algorithm == 4)
					{
						std::string controller_str;
						std::string consortium_str;
						switch(node_controller_ID[i])
						{
							case (0):
								controller_str = "C0";
								break;
							case (1):
								controller_str = "C1";
								break;
							case (2):
								controller_str = "C2";
								break;
							case (3):
								controller_str = "C3";
								break;
							default:
								controller_str = "C0";
								break;
						
						}
						
						switch(assigned_consortium_ID[i])
						{
							case (0):
								consortium_str = "consortium0";
								break;
							case (1):
								consortium_str = "consortium1";
								break;
							case (2):
								consortium_str = "consortium2";
								break;
							case (3):
								consortium_str = "consortium3";
								break;
							default:
								consortium_str = "consortium0";
								break;
						
						}    
						Simulator::Schedule(Seconds(t+0.0050*(total_size+1)+0.0050*(total_size+1)+i*0.002+0.05), CallBWTRCBFromNS3, i+2, controller_str);
						Simulator::Schedule(Seconds(t+0.0050*(total_size+2)+0.0050*(total_size+2)+0.002*total_size + i*0.002+0.05), BCTES, i+2, 0, controller_str, consortium_str);
					}
				 }
				 //Simulator::Schedule (Seconds (t+0.0), centralized_dsrc_data_unicast, dsrc_Nodes.Get(0), 0, 1, 1);
				 //Simulator::Schedule(Seconds(t+0.0050), CallBWTRCBFromNS3, 0+2, "consortium1");
				 //Simulator::Schedule(Seconds(t+0.0150), BCTES, 0+2, 0, "C1", "consortium1");
				 
			}	
//*/	
	}
	
 
 }
 

 
  
 
  
  for (double t=0; t< simTime-1;t=t+1)
  {
  	Simulator::Schedule (Seconds (t), print_time);
  }
 
  Config::ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&Rx) );
  Config::ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/MacRx", MakeCallback (&MacRx) );
  Config::ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/MacTx", MakeCallback (&MacTx) );
  Config::ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::RegularWifiMac/DcaTxop/Queue/Enqueue",MakeCallback (&Enqueue));
  for (uint32_t i = 0; i < uedevices.GetN(); ++i)
	{
		 Ptr<LteUeNetDevice> ueDev = uedevices.Get(i)->GetObject<LteUeNetDevice>();
		 Ptr<LteUeMac> ueMac = ueDev->GetMac();
		 ueMac->TraceConnectWithoutContext("UlScheduling", MakeBoundCallback(&UeTxStartCallback, i));

	}
  //Config::ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/ns3::RegularWifiMac/DcaTxop/Queue/Dequeue",MakeCallback (&Dequeue)); 
  
  AnimationInterface anim("/home/nilmantha/ns-allinone-3.35/ns-3.35/routing.xml");  

  if (N_RSUs > 0)
  {
	  for (uint32_t i=0; i<RSU_Nodes.GetN() ; i++)
	  {
	  	anim.UpdateNodeColor(RSU_Nodes.Get(i),255,255,0);//RSUs in yellow color
	  	Ptr <Node> ni = DynamicCast <Node> (RSU_Nodes.Get(i));
	  	anim.UpdateNodeSize(ni->GetId(),20.0,20.0);
	  }
  }
  
  if (N_Vehicles > 0)
  {
	  for (uint32_t i=0; i<Vehicle_Nodes.GetN() ; i++)
	  {
	  	anim.UpdateNodeColor(Vehicle_Nodes.Get(i),0,255,0);//vehicle nodes are green color
	  	Ptr <Node> ni = DynamicCast <Node> (Vehicle_Nodes.Get(i));
	  	anim.UpdateNodeSize(ni->GetId(),20.0,20.0);
	  }
	   
	  if (architecture !=1)
	  {
		  for (uint32_t i=0; i<other_stationary_LTE_nodes.GetN() ; i++)
		  {
		  	anim.UpdateNodeColor(other_stationary_LTE_nodes.Get(i),0,0,255);//LTE stationary nodes are blue color
		  	Ptr <Node> ni = DynamicCast <Node> (other_stationary_LTE_nodes.Get(i));
		  	anim.UpdateNodeSize(ni->GetId(),20.0,20.0);
		  }
	  }
  }
  
    if (architecture != 1)
    {
	    anim.UpdateNodeColor(controller_Node.Get(0),255,0,255);//controller node is purple color.
	    Ptr <Node> node_controller = DynamicCast <Node> (controller_Node.Get(0));
	    anim.UpdateNodeSize(node_controller->GetId(),20.0,20.0);
	    
	    anim.UpdateNodeColor(management_Node.Get(0),255,0,0);//management node is red color.
	    Ptr <Node> node_management = DynamicCast <Node> (management_Node.Get(0));
	    anim.UpdateNodeSize(node_management->GetId(),20.0,20.0);
    }
 
  //AnimationInterface anim("/home/nilmantha/ns-allinone-3.35/ns-3.35/routing.xml"); 
  
  /*
  for (uint32_t i=0; i<Custom_Nodes.GetN() ; i++)
  {
  	anim.UpdateNodeColor(Custom_Nodes.Get(i),255,255,0);//RSUs in yellow color
  	Ptr <Node> ni = DynamicCast <Node> (Custom_Nodes.Get(i));
  	anim.UpdateNodeSize(ni->GetId(),20.0,20.0);
  }
  */
  
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  
 
  //apb.SetFinish();
  return 0;  
}