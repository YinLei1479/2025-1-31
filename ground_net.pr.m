MIL_3_Tfile_Hdr_ 145A 140A modeler 9 66A71D0C 675FF3C8 45 ray-laptop 28918 0 0 none none 0 0 none 2A66D9E9 3EC6 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                              ЋЭg      @   D   H      *­  <Ђ  <І  <Њ  <Ў  <К  <О  <Т  *Ё           	   begsim intrpt         
   џџџџ   
   doc file            	nd_module      endsim intrpt             џџџџ      failure intrpts            disabled      intrpt interval         дВI­%У}џџџџ      priority              џџџџ      recovery intrpts            disabled      subqueue                     count    џџџ   
   џџџџ   
      list   	џџџ   
          
      super priority             џџџџ          ?   /* my node address */   int	\my_address;       Objid	\my_id;       Objid	\my_node_id;       /* rcv pk type */   
int	\type;       /* 0 or 1 for the net topo */   int	\topo[24][24];       int	\node_num;       int	\topo_address[24][3];       int	\dest[24];       int	\dest_num;       int	\data_pk_num;       int	\data_type;       int	\dist[24];       int	\mark[24];       int	\path[24];       int	\A[24][24];       int	\add_topo[24][24];       int	\interactive_id;       int	\route[24][5];       int	\route_num;       	int	\num;       int	\slot_assignment[24][24];       #int	\slot_appoint_over[24][24][24];       int	\remote_control_node[5];       double	\remote_control_time[5];       int	\remote_control_num;       int	\final_route[6];       int	\data_req_node;       double	\data_req_time;       int	\data_req_prio;       int	\data_req_continue;          Packet* pkptr;   int i,j;      #include <math.h>       /* Constant Definitions */   #define m 5000       #define SRC_IN		(1)   #define SINK_OUT	(1)   #define TX_OUT		(2)   #define RX_IN		(2)   #define SEND_DATA_S_CODE		100   #define DATA_REQ_CODE			101       h#define SEND_DATA_S				((op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code() == SEND_DATA_S_CODE))   b#define DATA_REQ				((op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code() == DATA_REQ_CODE))       "/* Transition Condition Macros */    Z#define FROM_RX_PK			(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm () == RX_IN)   \#define FROM_SRC_PK 		(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm () == SRC_IN)           њ   static void Initial(void)   	{   	int i;   	FIN(Initial(void));   	for(i=0;i<24;i++)   		{   		dist[i] = m;   		path[i] = -1;   		mark[i] = 0;   		}   	FOUT;   	}       static int Min(void)   	{   	int l=0,i;   	FIN(Min(void));   	while (mark[l]!=0)   		l++;   	for(i=0;i<24;i++)   (		if (mark[i] == 0 && dist[l] > dist[i])   				l = i;   		FRET(l);   	}       'static void Update(int start, int last)   	{   	int number = 0, k, i;   "	FIN(Update(int start, int last));   	k = start;   	dist[k] = 0;   
	number++;   	while (number <= 24)   		{   		for(i=0;i<24;i++)   			{   			if(mark[i] == 0)   				{   $				if (dist[i] > dist[k] + A[k][i])   					{   !					dist[i] = dist[k] + A[k][i];   					path[i] = k;   					}   				}   			}   		k = Min();   		number++;   		mark[k] = 1;   		}   	FOUT;   	}   	   'static void Output(int start, int last)   	{   
	int k, i;   	int res[30], n = 0;   "	FIN(Output(int start, int last));   	res[n++] = last;   
	k = last;   !	printf("%d--->%d\n",start,last);   ,	for(i=0;i<24;i++) printf("  %d  ",path[i]);   	while (k != start)   		{   		res[n++] = path[k];   		k = path[k];   		}   	printf("\n path:");   	for (i = n - 1; i >= 0; i--)   		final_route[i]=res[i];   	   	FOUT;   	}	   		   static void send_data_req(void)   	{   	Packet* pkptr;   	FIN(send_data_req(void));   (	pkptr=op_pk_create_fmt("data_request");   (	op_pk_nfd_set(pkptr,"SEND",my_address);   "	op_pk_nfd_set(pkptr,"TYPE",0x11);   *	op_pk_nfd_set(pkptr,"REQ",data_req_node);   +	op_pk_nfd_set(pkptr,"Prio",data_req_prio);   3	op_pk_nfd_set(pkptr,"Continue",data_req_continue);   	op_pk_send(pkptr,TX_OUT);   	FOUT;   	}       ,static void Link_maintain_rcv(Packet *pkptr)   	{   	int z;   		int i,j;   	int a_topo[276];   	int num_elements=9;   	int int_array[9];    	unsigned char bit_sequence[35];   	   &	FIN(Link_maintain_rcv(Packet *pkptr))   	z=0;   0	op_pk_nfd_get(pkptr,"Net_Topo1",&int_array[0]);   0	op_pk_nfd_get(pkptr,"Net_Topo2",&int_array[1]);   0	op_pk_nfd_get(pkptr,"Net_Topo3",&int_array[2]);   0	op_pk_nfd_get(pkptr,"Net_Topo4",&int_array[3]);   0	op_pk_nfd_get(pkptr,"Net_Topo5",&int_array[4]);   0	op_pk_nfd_get(pkptr,"Net_Topo6",&int_array[5]);   0	op_pk_nfd_get(pkptr,"Net_Topo7",&int_array[6]);   0	op_pk_nfd_get(pkptr,"Net_Topo8",&int_array[7]);   0	op_pk_nfd_get(pkptr,"Net_Topo9",&int_array[8]);   "	for(i = 0; i < num_elements; i++)   		for(j = 0; j <4; j++)   D			bit_sequence[i * 4 + j] = (int_array[i] >> (8 * (3 - j))) & 0xFF;   	for (i=0;i<276;i++)   9        a_topo[i] = (bit_sequence[i / 8] >> (i % 8)) & 1;   	//зщжЏtopo   	for(i=0;i<24;i++)   !		for(j=0;j<24;j++) topo[i][j]=0;   	for(i=0;i<23;i++)   		for(j=i+1;j<24;j++)   			{   			topo[i][j]=a_topo[z];   			topo[j][i]=a_topo[z];   			z++;   			}   %	printf("\n ground get the topo:\n");   	for(i=0;i<node_num;i++)   	{   	for(j=0;j<node_num;j++)   		printf("%d   ",topo[i][j]);   	printf("\n");   	printf("%lf\n",op_sim_time());   	}   <	op_pk_nfd_get(pkptr,"Nei_longtitude0",&topo_address[0][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude1",&topo_address[1][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude2",&topo_address[2][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude3",&topo_address[3][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude4",&topo_address[4][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude5",&topo_address[5][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude6",&topo_address[6][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude7",&topo_address[7][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude8",&topo_address[8][0]);   <	op_pk_nfd_get(pkptr,"Nei_longtitude9",&topo_address[9][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude10",&topo_address[10][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude11",&topo_address[11][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude12",&topo_address[12][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude13",&topo_address[13][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude14",&topo_address[14][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude15",&topo_address[15][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude16",&topo_address[16][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude17",&topo_address[17][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude18",&topo_address[18][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude19",&topo_address[19][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude20",&topo_address[20][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude21",&topo_address[21][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude22",&topo_address[22][0]);   >	op_pk_nfd_get(pkptr,"Nei_longtitude23",&topo_address[23][0]);       :	op_pk_nfd_get(pkptr,"Nei_latitude0",&topo_address[0][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude1",&topo_address[1][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude2",&topo_address[2][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude3",&topo_address[3][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude4",&topo_address[4][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude5",&topo_address[5][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude6",&topo_address[6][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude7",&topo_address[7][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude8",&topo_address[8][1]);   :	op_pk_nfd_get(pkptr,"Nei_latitude9",&topo_address[9][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude10",&topo_address[10][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude11",&topo_address[11][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude12",&topo_address[12][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude13",&topo_address[13][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude14",&topo_address[14][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude15",&topo_address[15][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude16",&topo_address[16][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude17",&topo_address[17][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude18",&topo_address[18][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude19",&topo_address[19][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude20",&topo_address[20][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude21",&topo_address[21][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude22",&topo_address[22][1]);   <	op_pk_nfd_get(pkptr,"Nei_latitude23",&topo_address[23][1]);       8	op_pk_nfd_get(pkptr,"Nei_height0",&topo_address[0][2]);   8	op_pk_nfd_get(pkptr,"Nei_height1",&topo_address[1][2]);   8	op_pk_nfd_get(pkptr,"Nei_height2",&topo_address[2][2]);   8	op_pk_nfd_get(pkptr,"Nei_height3",&topo_address[3][2]);   8	op_pk_nfd_get(pkptr,"Nei_height4",&topo_address[4][2]);   8	op_pk_nfd_get(pkptr,"Nei_height5",&topo_address[5][2]);   8	op_pk_nfd_get(pkptr,"Nei_height6",&topo_address[6][2]);   8	op_pk_nfd_get(pkptr,"Nei_height7",&topo_address[7][2]);   8	op_pk_nfd_get(pkptr,"Nei_height8",&topo_address[8][2]);   8	op_pk_nfd_get(pkptr,"Nei_height9",&topo_address[9][2]);   :	op_pk_nfd_get(pkptr,"Nei_height10",&topo_address[10][2]);   :	op_pk_nfd_get(pkptr,"Nei_height11",&topo_address[11][2]);   :	op_pk_nfd_get(pkptr,"Nei_height12",&topo_address[12][2]);   :	op_pk_nfd_get(pkptr,"Nei_height13",&topo_address[13][2]);   :	op_pk_nfd_get(pkptr,"Nei_height14",&topo_address[14][2]);   :	op_pk_nfd_get(pkptr,"Nei_height15",&topo_address[15][2]);   :	op_pk_nfd_get(pkptr,"Nei_height16",&topo_address[16][2]);   :	op_pk_nfd_get(pkptr,"Nei_height17",&topo_address[17][2]);   :	op_pk_nfd_get(pkptr,"Nei_height18",&topo_address[18][2]);   :	op_pk_nfd_get(pkptr,"Nei_height19",&topo_address[19][2]);   :	op_pk_nfd_get(pkptr,"Nei_height20",&topo_address[20][2]);   :	op_pk_nfd_get(pkptr,"Nei_height21",&topo_address[21][2]);   :	op_pk_nfd_get(pkptr,"Nei_height22",&topo_address[22][2]);   :	op_pk_nfd_get(pkptr,"Nei_height23",&topo_address[23][2]);   	   	op_pk_destroy(pkptr);   	FOUT   	}       static void Appoint_send(void)   	{   	int start;   
	int last;   		int i,j;   	Packet *pkptr;   	   	FIN(Appoint_send(void))   	//ТЗОЖЙцЛЎ   	for(i=0;i<24;i++)   		for(j=0;j<24;j++)   			{   			if(topo[i][j]==1) A[i][j]=1;   			else A[i][j]=m;   			}   &	for(i=0;i<6;i++) final_route[i]=0xFF;   /	start=remote_control_node[remote_control_num];   	last=interactive_id;   	Initial();   	Update(start,last);   	Output(start,last);   2	for(i=0;i<6;i++) printf("%d--ЁЗ",final_route[i]);       	//зААќ   $	pkptr=op_pk_create_fmt("appoint1");   (	op_pk_nfd_set(pkptr,"SEND",my_address);   "	op_pk_nfd_set(pkptr,"TYPE",0x20);   #	op_pk_nfd_set(pkptr,"DEST",start);   2	op_pk_nfd_set(pkptr,"Pk_num",remote_control_num);   &	op_pk_nfd_set(pkptr,"Control_MES",1);   0	op_pk_nfd_set(pkptr,"Hop1_MES",final_route[1]);   0	op_pk_nfd_set(pkptr,"Hop2_MES",final_route[2]);   0	op_pk_nfd_set(pkptr,"Hop3_MES",final_route[3]);   0	op_pk_nfd_set(pkptr,"Hop4_MES",final_route[4]);   0	op_pk_nfd_set(pkptr,"Next_hop",interactive_id);   	op_pk_send(pkptr,TX_OUT);       	remote_control_num++;       	FOUT   	}                                          Z   в          
   init   
       
   /   //initial begin   my_id = op_id_self();   $my_node_id = op_topo_parent (my_id);   6op_ima_obj_attr_get(my_node_id,"Address",&my_address);   for(i=0;i<24;i++)    	for(j=0;j<24;j++) topo[i][j]=0;   for(i=0;i<24;i++)   '	for(j=0;j<3;j++) topo_address[i][j]=0;   node_num=14;   Iop_ima_sim_attr_get (OPC_IMA_INTEGER, "interactive_id", &interactive_id);       5printf("$$$$$$$$$$$$ground net over$$$$$$$$$$$$$\n");   <op_intrpt_schedule_self(op_sim_time()+200,SEND_DATA_S_CODE);   for(i=0;i<24;i++) dest[i]=i;   dest_num=1;   data_pk_num=0;   data_type=0;       //дЖГЬПижЦГѕЪМЛЏ   for(i=0;i<5;i++)   	{   	remote_control_node[i]=0xFF;   	remote_control_time[i]=0;   	}   
//ЖСШЁгУЛЇ   Pop_ima_obj_attr_get(my_node_id,"remote_control_node_0",&remote_control_node[0]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_node_1",&remote_control_node[1]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_node_2",&remote_control_node[2]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_node_3",&remote_control_node[3]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_node_4",&remote_control_node[4]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_time_0",&remote_control_time[0]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_time_1",&remote_control_time[1]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_time_2",&remote_control_time[2]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_time_3",&remote_control_time[3]);   Pop_ima_obj_attr_get(my_node_id,"remote_control_time_4",&remote_control_time[4]);   
//ЩшЖЈжаЖЯ   for(i=0;i<5;i++)   c	if(remote_control_node[i]!=0xFF) op_intrpt_schedule_self(remote_control_time[i],SEND_DATA_S_CODE);   remote_control_num=0;           //ЕЅНкЕуЛиДЋГѕЪМЛЏ   ?op_ima_obj_attr_get(my_node_id,"data_req_node",&data_req_node);   ?op_ima_obj_attr_get(my_node_id,"data_req_time",&data_req_time);   ?op_ima_obj_attr_get(my_node_id,"data_req_prio",&data_req_prio);   Gop_ima_obj_attr_get(my_node_id,"data_req_continue",&data_req_continue);   5op_intrpt_schedule_self(data_req_time,DATA_REQ_CODE);   
                     
   џџџџ   
          pr_state           в          
   idle   
                                       џџџџ             pr_state        J   Z          
   rx_rcv   
       
      //ЪеЕНВЛЭЌРраЭАќЕФааЮЊ   "pkptr=op_pk_get(op_intrpt_strm());   %op_pk_nfd_get (pkptr, "TYPE", &type);       if(type==0x10)//link_maintain   	{   	Link_maintain_rcv(pkptr);   	}       if(type==0x81)   	op_pk_send(pkptr,SINK_OUT);   
                     
   џџџџ   
          pr_state        Т            
   src_rcv   
       
       
                     
   џџџџ   
          pr_state         в  J          
   	data_send   
       
      Appoint_send();   
                     J   џџџџ   J          pr_state          	              Б   б      g   Ю   і   б          
   tr_0   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition               Ю            Ф   Х      §        Р          
   tr_1   
       
   default   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition              '           М  9   h          
   tr_3   
       
   
FROM_RX_PK   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition              5        H   j     П          
   tr_5   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition                 ф        к  Д   џ          
   tr_6   
       
   FROM_SRC_PK   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition              x   ч     А       р          
   tr_7   
       
џџџџ   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition               ь          л   ж  /          
   tr_12   
       
   SEND_DATA_S   
       џџџџ          
    џџџџ   
          џџџџ                       pr_transition               њ        ц  :     х          
   tr_13   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition              ^  e     !   ф  v  7  '  J     ш          
   tr_14   
       
   DATA_REQ   
       
   send_data_req()   
       
    џџџџ   
          џџџџ                       pr_transition                                             