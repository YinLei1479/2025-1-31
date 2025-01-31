/* Process model C form file: net_V1.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char net_V1_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A op_runsim 7 67600E6D 67600E6D 1 ray-laptop 28918 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                                       ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include <math.h>

/* Constant Definitions */
#define RX_IN		(0)
#define SRC_IN		(1)
#define TX_OUT		(0)
#define SINK_OUT	(1)
#define TX_OUT_2	(2)
#define RX_IN_2		(2)

#define COLLECT_FINISH	(5000)
#define TIME_OUT_SIGNAL (6000)
#define INTACT_PK_SEND	(7000)
#define ROUTE_CHANGE	(3110)
#define REQ_OVER_CODE	(3111)

/* Transition Condition Macros */ 
#define FROM_RX_PK			(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm () == RX_IN)
#define FROM_SRC_PK 		(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm () == SRC_IN)
#define FROM_RX2_PK			(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm () == RX_IN_2)

#define INTACT_START 		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == INTACT_PK_SEND)
#define COLLECT				(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == COLLECT_FINISH)
#define TIME_OUT			(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_SIGNAL)
#define CHANGE_ROUTE 		(op_intrpt_type() == OPC_INTRPT_REMOTE) && (op_intrpt_code () == ROUTE_CHANGE)
#define REQ_OVER 			(op_intrpt_type() == OPC_INTRPT_REMOTE) && (op_intrpt_code () == REQ_OVER_CODE)

#define m 5201314

/* End of Header Block */

#if !defined (VOSD_NO_FIN)
#undef	BIN
#undef	BOUT
#define	BIN		FIN_LOCAL_FIELD(_op_last_line_passed) = __LINE__ - _op_block_origin;
#define	BOUT	BIN
#define	BINIT	FIN_LOCAL_FIELD(_op_last_line_passed) = 0; _op_block_origin = __LINE__;
#else
#define	BINIT
#endif /* #if !defined (VOSD_NO_FIN) */



/* State variable definitions */
typedef struct
	{
	/* Internal state tracking for FSM */
	FSM_SYS_STATE
	/* State Variables */
	int	                    		my_address                                      ;	/* my node address */
	Objid	                  		my_id                                           ;
	Objid	                  		my_node_id                                      ;
	int	                    		type                                            ;	/* rcv pk type */
	int	                    		interactive_id                                  ;
	int	                    		to_interact_next_hop                            ;	/* the next hop to the interact node */
	int	                    		interact_pk_num                                 ;	/* the Num of the pk */
	int	                    		topo[24][24]                                    ;	/* 0 or 1 for the net topo */
	int	                    		link_interact_pk_num                            ;	/* pk num of interact collection */
	Evhandle	               		evh                                             ;	/* time out self intrpt */
	int	                    		node_num                                        ;
	int	                    		topo_address[24][3]                             ;
	int	                    		route_des[24]                                   ;
	int	                    		node_in_net[24]                                 ;
	int	                    		data_control_num                                ;
	int	                    		node_in_net_last[24]                            ;
	int	                    		in_net_num                                      ;
	double	                 		data_req_over                                   ;
	int	                    		last_topo[24][24]                               ;
	int	                    		route_num                                       ;
	int	                    		route[5]                                        ;
	int	                    		dist[24]                                        ;
	int	                    		mark[24]                                        ;
	int	                    		path[24]                                        ;
	int	                    		A[24][24]                                       ;
	} net_V1_state;

#define my_address              		op_sv_ptr->my_address
#define my_id                   		op_sv_ptr->my_id
#define my_node_id              		op_sv_ptr->my_node_id
#define type                    		op_sv_ptr->type
#define interactive_id          		op_sv_ptr->interactive_id
#define to_interact_next_hop    		op_sv_ptr->to_interact_next_hop
#define interact_pk_num         		op_sv_ptr->interact_pk_num
#define topo                    		op_sv_ptr->topo
#define link_interact_pk_num    		op_sv_ptr->link_interact_pk_num
#define evh                     		op_sv_ptr->evh
#define node_num                		op_sv_ptr->node_num
#define topo_address            		op_sv_ptr->topo_address
#define route_des               		op_sv_ptr->route_des
#define node_in_net             		op_sv_ptr->node_in_net
#define data_control_num        		op_sv_ptr->data_control_num
#define node_in_net_last        		op_sv_ptr->node_in_net_last
#define in_net_num              		op_sv_ptr->in_net_num
#define data_req_over           		op_sv_ptr->data_req_over
#define last_topo               		op_sv_ptr->last_topo
#define route_num               		op_sv_ptr->route_num
#define route                   		op_sv_ptr->route
#define dist                    		op_sv_ptr->dist
#define mark                    		op_sv_ptr->mark
#define path                    		op_sv_ptr->path
#define A                       		op_sv_ptr->A

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	net_V1_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((net_V1_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

static void change_route(void)
	{
	int min;
	int i;
	FIN(change_route(void));
	min=8;
	for(i=0;i<24;i++)
		{
		if(route_des[i]<min)
			{
			min=route_des[i];
			to_interact_next_hop=i;
			}
		}
	for(i=0;i<24;i++) route_des[i]=8;
	FOUT;
	}

static void req_over(void)
	{
	FIN(req_over(void));
	data_req_over=1;
	FOUT;
	}

static void Initial(void)
	{
	int i;
	FIN(Initial(void));
	for(i=0;i<24;i++)
		{
		dist[i] = m;
		path[i] = -1;
		mark[i] = 0;
		}
	FOUT;
	}

static int Min(void)
	{
	int l=0,i;
	FIN(Min(void));
	while (mark[l]!=0)
		l++;
	for(i=0;i<24;i++)
		if (mark[i] == 0 && dist[l] > dist[i])
			l = i;
	FRET(l);
	}

static void Update(int start, int last)
	{
	int number = 0, k, i;
	FIN(Update(int start, int last));
	k = start;
	dist[k] = 0;
	number++;
	while (number <= 24)
		{
		for(i=0;i<24;i++)
			{
			if(mark[i] == 0)
				{
				if (dist[i] > dist[k] + A[k][i])
					{
					dist[i] = dist[k] + A[k][i];
					path[i] = k;
					}
				}
			}
		k = Min();
		number++;
		mark[k] = 1;
		}
	FOUT;
	}
	
static void Output(int start, int last)
	{
	int k, i;
	int res[20], n = 0;
	FIN(Output(int start, int last));
	res[n++] = last;
	k = last;
	printf("%d--->%d\n",start,last);
	while (k != start)
	{
		res[n++] = path[k];
		k = path[k];
	}
	printf("\n path:");
	for (i = n - 1; i >= 0; i--)
	{
		if (i == n - 1)
			printf("%d", res[i]);
		else
			printf("->%d", res[i]);
	}
	for(i=0;i<5;i++)
		route[i]=res[i];
	route_num=n;
	FOUT;
}

static void QWZ_netrcv(Packet *pkptr)
	{
	int source;
	
	FIN(QWZ_netrcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"SEND",&source);
	op_pk_nfd_get(pkptr,"Clock_level", &route_des[source]);
	op_pk_destroy(pkptr);
	FOUT
	}

static void Link_report_netrcv(Packet *pkptr)
	{
	int source;
	int nei_num;
	int nei_id[8];
	int nei_longitude;
	int nei_latitude;
	int nei_height;
	int i;
	int pk_interact_num;
	
	FIN(Link_report_netrcv(Packet *pkptr))
	for(i=0;i<8;i++) nei_id[i]=0xFF;
	op_pk_nfd_get (pkptr, "Source",&source);
	if(source==my_address && my_address!=interactive_id)
		{
		op_pk_nfd_set(pkptr,"Next_Hop",to_interact_next_hop);
		op_pk_nfd_set(pkptr,"Num",interact_pk_num);
		op_pk_send(pkptr,TX_OUT);
		printf("%d not intact node,ready to send tdma,next hop:%d\n",my_address,to_interact_next_hop);
		interact_pk_num+=1;
		if(interact_pk_num==16) interact_pk_num=0;//Num
		}
	else if(source==my_address && my_address==interactive_id)
		{
		//记录信息
		op_pk_nfd_get (pkptr, "Nei_num",&nei_num);
		op_pk_nfd_get (pkptr, "Nei_address_0", &nei_id[0]);
		op_pk_nfd_get (pkptr, "Nei_address_1", &nei_id[1]);
		op_pk_nfd_get (pkptr, "Nei_address_2", &nei_id[2]);
		op_pk_nfd_get (pkptr, "Nei_address_3", &nei_id[3]);
		op_pk_nfd_get (pkptr, "Nei_address_4", &nei_id[4]);
		op_pk_nfd_get (pkptr, "Nei_address_5", &nei_id[5]);
		op_pk_nfd_get (pkptr, "Nei_address_6", &nei_id[6]);
		op_pk_nfd_get (pkptr, "Nei_address_7", &nei_id[7]);
		op_pk_nfd_get(pkptr,"Longitude", &nei_longitude);
		op_pk_nfd_get(pkptr,"Latitude", &nei_latitude);
		op_pk_nfd_get(pkptr,"Height", &nei_height);
		
		for(i=0;i<nei_num;i++)
			{
			topo[source][nei_id[i]]=1;
			}
		topo_address[source][0]=nei_longitude;
		topo_address[source][1]=nei_latitude;
		topo_address[source][2]=nei_height;
		
		op_pk_destroy(pkptr);
		//开启定时器
		evh=op_intrpt_schedule_self(op_sim_time()+60,TIME_OUT_SIGNAL);////can change!!!
		}
	else
		{
		op_pk_nfd_get (pkptr, "Num", &pk_interact_num);
		if(my_address!=interactive_id)
			{
			op_pk_nfd_set (pkptr, "Next_Hop", to_interact_next_hop);
			op_pk_send(pkptr,TX_OUT);;
			}
		else/////交互节点
			{
			link_interact_pk_num++;
			op_pk_nfd_get (pkptr, "Source",&source);
			op_pk_nfd_get (pkptr, "Nei_num",&nei_num);
			op_pk_nfd_get (pkptr, "Nei_address_0", &nei_id[0]);
			op_pk_nfd_get (pkptr, "Nei_address_1", &nei_id[1]);
			op_pk_nfd_get (pkptr, "Nei_address_2", &nei_id[2]);
			op_pk_nfd_get (pkptr, "Nei_address_3", &nei_id[3]);
			op_pk_nfd_get (pkptr, "Nei_address_4", &nei_id[4]);
			op_pk_nfd_get (pkptr, "Nei_address_5", &nei_id[5]);
			op_pk_nfd_get (pkptr, "Nei_address_6", &nei_id[6]);
			op_pk_nfd_get (pkptr, "Nei_address_7", &nei_id[7]);
			op_pk_nfd_get (pkptr, "Longitude", &nei_longitude);
			op_pk_nfd_get (pkptr, "Latitude", &nei_latitude);
			op_pk_nfd_get (pkptr, "Height", &nei_height);
				
			op_pk_destroy(pkptr);
			for(i=0;i<nei_num;i++)
				{
				topo[source][nei_id[i]]=1;
				}
			topo_address[source][0]=nei_longitude;
			topo_address[source][1]=nei_latitude;
			topo_address[source][2]=nei_height;
			
			printf("intact get pk from %d,time:%lf\n",source,op_sim_time());
			if(link_interact_pk_num==node_num-1)
				{
				printf("收集完毕！！！\n");
				op_ev_cancel(evh);
				op_intrpt_schedule_self(op_sim_time(),COLLECT_FINISH);
				}
			}	
		}
	FOUT
	}

static void DATA_netrcv(Packet *pkptr)
	{
	int dest;
	int source;
	
	FIN(DATA_netrcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"DEST",&dest);
	op_pk_nfd_get(pkptr,"Src",&source);
	if(dest==my_address) 
		{
		op_pk_send(pkptr,TX_OUT_2);
		}
	else
		{
		op_pk_nfd_set(pkptr,"Recv",to_interact_next_hop);
		op_pk_send(pkptr,TX_OUT);
		}
	FOUT
	}

static void DATA_create(Packet *pkptr)
	{
	FIN(DATA_create(Packet *pkptr));
	op_pk_nfd_set(pkptr,"SEND",my_address);
	op_pk_nfd_set(pkptr,"TYPE",0x81);
	op_pk_nfd_set(pkptr,"DEST",interactive_id);
	op_pk_nfd_set(pkptr,"Src",my_address);
	op_pk_nfd_set(pkptr,"Recv",to_interact_next_hop);
	op_pk_nfd_set(pkptr,"Cos",1);
	if(my_address==interactive_id) op_pk_send(pkptr,TX_OUT_2);
	else op_pk_send(pkptr,TX_OUT);
	FOUT
	}

static void Data_request_rcv(Packet *pkptr)
	{
	int i;
	int j;
	int req_node;
	int req_prio;
	int req_continue;
	int start;
	int last;
	int z;
	int in_net[24];
	
	FIN(Data_request_rcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"REQ",&req_node);
	op_pk_nfd_get(pkptr,"Prio",&req_prio);
	op_pk_nfd_get(pkptr,"Continue",&req_continue);
	op_pk_destroy(pkptr);
	//路径计算
	for(i=0;i<24;i++)
		for(j=0;j<24;j++)
			{
			if(last_topo[i][j]==1) A[i][j]=1;
			else A[i][j]=m;
			}
	start=req_node;
	last=interactive_id;
	Initial();
	Update(start,last);
	Output(start,last);
	for(i=0;i<5;i++) printf("%d--》",route[i]);
	
	//时隙分配
	z=in_net_num/(route_num-1);
	for(i=0;i<24;i++) in_net[i]=0xFF;
	for(i=1;i<route_num;i++) 
		for(j=0;j<z;j++)
			in_net[(i-1)*z+j]=route[route_num-i];
	
	
	pkptr=op_pk_create_fmt("data_contro");
	op_pk_nfd_set(pkptr,"TYPE",0x02);
	op_pk_nfd_set(pkptr,"NUM",data_control_num);
	op_pk_nfd_set(pkptr,"TTL",7);
	op_pk_nfd_set(pkptr,"Priority",req_prio);
	op_pk_nfd_set(pkptr,"Continue",req_continue);
	
	op_pk_nfd_set(pkptr,"Slot_0",in_net[0]);
	op_pk_nfd_set(pkptr,"Slot_1",in_net[1]);
	op_pk_nfd_set(pkptr,"Slot_2",in_net[2]);
	op_pk_nfd_set(pkptr,"Slot_3",in_net[3]);
	op_pk_nfd_set(pkptr,"Slot_4",in_net[4]);
	op_pk_nfd_set(pkptr,"Slot_5",in_net[5]);
	op_pk_nfd_set(pkptr,"Slot_6",in_net[6]);
	op_pk_nfd_set(pkptr,"Slot_7",in_net[7]);
	op_pk_nfd_set(pkptr,"Slot_8",in_net[8]);
	op_pk_nfd_set(pkptr,"Slot_9",in_net[9]);
	op_pk_nfd_set(pkptr,"Slot_10",in_net[10]);
	op_pk_nfd_set(pkptr,"Slot_11",in_net[11]);
	op_pk_nfd_set(pkptr,"Slot_12",in_net[12]);
	op_pk_nfd_set(pkptr,"Slot_13",in_net[13]);
	op_pk_nfd_set(pkptr,"Slot_14",in_net[14]);
	op_pk_nfd_set(pkptr,"Slot_15",in_net[15]);
	op_pk_nfd_set(pkptr,"Slot_16",in_net[16]);
	op_pk_nfd_set(pkptr,"Slot_17",in_net[17]);
	op_pk_nfd_set(pkptr,"Slot_18",in_net[18]);
	op_pk_nfd_set(pkptr,"Slot_19",in_net[19]);
	op_pk_nfd_set(pkptr,"Slot_20",in_net[20]);
	op_pk_nfd_set(pkptr,"Slot_21",in_net[21]);
	op_pk_nfd_set(pkptr,"Slot_22",in_net[22]);
	op_pk_nfd_set(pkptr,"Slot_23",in_net[23]);
	
	op_pk_send(pkptr,TX_OUT);
	
	data_req_over=0;
	data_control_num++;
	if(data_control_num==16) data_control_num=0;
	FOUT
	}

static void Link_collect_over(void)
	{
	int z=0;
	int i,j;
	int num_elements=9;
	int a_topo[276];
	int int_array[9];
	unsigned char bit_sequence[35] = {0};
	int in_net[24];
	int FF=0;
	Packet *pkptr;

	FIN(Link_collect_over(void))
	//数传时隙规划
	for(i=0;i<24;i++) node_in_net_last[i]=node_in_net[i];
	for(i=0;i<24;i++) node_in_net[i]=0;
	for(i=0;i<23;i++)
		{
		for(j=i+1;j<24;j++)
			{
			if(topo[i][j]==1 || topo[j][i]==1)
				{
				node_in_net[i]=1;
				node_in_net[j]=1;
				}
			}
		}
	for(i=0;i<24;i++)
		if(node_in_net[i]!=node_in_net_last[i]) FF=1;

	if(FF==1 && data_req_over==1)//CHANGE
		{
		j=0;
		for(i=0;i<24;i++) in_net[i]=31;
		for(i=0;i<24;i++) if(node_in_net[i]==1)
			{
			in_net[j]=i;
			j++;
			}
		in_net_num=j;
		pkptr=op_pk_create_fmt("data_contro");
		op_pk_nfd_set(pkptr,"TYPE",0x02);
		op_pk_nfd_set(pkptr,"NUM",data_control_num);
		op_pk_nfd_set(pkptr,"TTL",7);
		op_pk_nfd_set(pkptr,"Priority",1);///
		op_pk_nfd_set(pkptr,"Continue",0);
	
		op_pk_nfd_set(pkptr,"Slot_0",in_net[0]);
		op_pk_nfd_set(pkptr,"Slot_1",in_net[1]);
		op_pk_nfd_set(pkptr,"Slot_2",in_net[2]);
		op_pk_nfd_set(pkptr,"Slot_3",in_net[3]);
		op_pk_nfd_set(pkptr,"Slot_4",in_net[4]);
		op_pk_nfd_set(pkptr,"Slot_5",in_net[5]);
		op_pk_nfd_set(pkptr,"Slot_6",in_net[6]);
		op_pk_nfd_set(pkptr,"Slot_7",in_net[7]);
		op_pk_nfd_set(pkptr,"Slot_8",in_net[8]);
		op_pk_nfd_set(pkptr,"Slot_9",in_net[9]);
		op_pk_nfd_set(pkptr,"Slot_10",in_net[10]);
		op_pk_nfd_set(pkptr,"Slot_11",in_net[11]);
		op_pk_nfd_set(pkptr,"Slot_12",in_net[12]);
		op_pk_nfd_set(pkptr,"Slot_13",in_net[13]);
		op_pk_nfd_set(pkptr,"Slot_14",in_net[14]);
		op_pk_nfd_set(pkptr,"Slot_15",in_net[15]);
		op_pk_nfd_set(pkptr,"Slot_16",in_net[16]);
		op_pk_nfd_set(pkptr,"Slot_17",in_net[17]);
		op_pk_nfd_set(pkptr,"Slot_18",in_net[18]);
		op_pk_nfd_set(pkptr,"Slot_19",in_net[19]);
		op_pk_nfd_set(pkptr,"Slot_20",in_net[20]);
		op_pk_nfd_set(pkptr,"Slot_21",in_net[21]);
		op_pk_nfd_set(pkptr,"Slot_22",in_net[22]);
		op_pk_nfd_set(pkptr,"Slot_23",in_net[23]);
	
		op_pk_send(pkptr,TX_OUT);
	
		data_control_num++;
		if(data_control_num==16) data_control_num=0;
		}

	//向地面上报topo
	printf("ready to pk up to ground\n");
	printf("time:%f\n",op_sim_time());
	for(i=0;i<node_num;i++)
		{
		for(j=0;j<node_num;j++)
			printf("%d   ",topo[i][j]);
		printf("\n");
		}


	pkptr=op_pk_create_fmt("link_maintain");
	op_pk_nfd_set(pkptr,"TYPE",0x10);
	op_pk_nfd_set(pkptr,"Num",interact_pk_num);
	op_pk_nfd_set(pkptr,"None",0);

	for(i=0;i<23;i++)
		{
		for(j=i+1;j<24;j++)
			{
			if(topo[i][j]==1 || topo[j][i]==1)
				a_topo[z]=1;
			else
				a_topo[z]=0;
			z++;
			}
		}
	for(i=0;i<276;i++)
		{
		if(a_topo[i]==1)
			bit_sequence[i / 8] |= (1 << (i % 8));
		else
			bit_sequence[i / 8] &= ~(1 << (i % 8));
		}
	for(i=0;i<num_elements;i++)
		{
		int_array[i]=0;
		for(j=0;j<4;j++)
			int_array[i] |= (bit_sequence[i * 4 + j] << (8 * (3- j)));
	}

	op_pk_nfd_set(pkptr,"Net_Topo1",int_array[0]);
	op_pk_nfd_set(pkptr,"Net_Topo2",int_array[1]);
	op_pk_nfd_set(pkptr,"Net_Topo3",int_array[2]);
	op_pk_nfd_set(pkptr,"Net_Topo4",int_array[3]);
	op_pk_nfd_set(pkptr,"Net_Topo5",int_array[4]);
	op_pk_nfd_set(pkptr,"Net_Topo6",int_array[5]);
	op_pk_nfd_set(pkptr,"Net_Topo7",int_array[6]);
	op_pk_nfd_set(pkptr,"Net_Topo8",int_array[7]);
	op_pk_nfd_set(pkptr,"Net_Topo9",int_array[8]);

	op_pk_nfd_set(pkptr,"Nei_longtitude0",topo_address[0][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude1",topo_address[1][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude2",topo_address[2][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude3",topo_address[3][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude4",topo_address[4][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude5",topo_address[5][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude6",topo_address[6][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude7",topo_address[7][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude8",topo_address[8][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude9",topo_address[9][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude10",topo_address[10][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude11",topo_address[11][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude12",topo_address[12][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude13",topo_address[13][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude14",topo_address[14][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude15",topo_address[15][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude16",topo_address[16][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude17",topo_address[17][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude18",topo_address[18][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude19",topo_address[19][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude20",topo_address[20][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude21",topo_address[21][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude22",topo_address[22][0]);
	op_pk_nfd_set(pkptr,"Nei_longtitude23",topo_address[23][0]);

	op_pk_nfd_set(pkptr,"Nei_latitude0",topo_address[0][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude1",topo_address[1][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude2",topo_address[2][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude3",topo_address[3][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude4",topo_address[4][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude5",topo_address[5][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude6",topo_address[6][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude7",topo_address[7][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude8",topo_address[8][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude9",topo_address[9][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude10",topo_address[10][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude11",topo_address[11][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude12",topo_address[12][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude13",topo_address[13][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude14",topo_address[14][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude15",topo_address[15][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude16",topo_address[16][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude17",topo_address[17][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude18",topo_address[18][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude19",topo_address[19][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude20",topo_address[20][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude21",topo_address[21][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude22",topo_address[22][1]);
	op_pk_nfd_set(pkptr,"Nei_latitude23",topo_address[23][1]);

	op_pk_nfd_set(pkptr,"Nei_height0",topo_address[0][2]);
	op_pk_nfd_set(pkptr,"Nei_height1",topo_address[1][2]);
	op_pk_nfd_set(pkptr,"Nei_height2",topo_address[2][2]);
	op_pk_nfd_set(pkptr,"Nei_height3",topo_address[3][2]);
	op_pk_nfd_set(pkptr,"Nei_height4",topo_address[4][2]);
	op_pk_nfd_set(pkptr,"Nei_height5",topo_address[5][2]);
	op_pk_nfd_set(pkptr,"Nei_height6",topo_address[6][2]);
	op_pk_nfd_set(pkptr,"Nei_height7",topo_address[7][2]);
	op_pk_nfd_set(pkptr,"Nei_height8",topo_address[8][2]);
	op_pk_nfd_set(pkptr,"Nei_height9",topo_address[9][2]);
	op_pk_nfd_set(pkptr,"Nei_height10",topo_address[10][2]);
	op_pk_nfd_set(pkptr,"Nei_height11",topo_address[11][2]);
	op_pk_nfd_set(pkptr,"Nei_height12",topo_address[12][2]);
	op_pk_nfd_set(pkptr,"Nei_height13",topo_address[13][2]);
	op_pk_nfd_set(pkptr,"Nei_height14",topo_address[14][2]);
	op_pk_nfd_set(pkptr,"Nei_height15",topo_address[15][2]);
	op_pk_nfd_set(pkptr,"Nei_height16",topo_address[16][2]);
	op_pk_nfd_set(pkptr,"Nei_height17",topo_address[17][2]);
	op_pk_nfd_set(pkptr,"Nei_height18",topo_address[18][2]);
	op_pk_nfd_set(pkptr,"Nei_height19",topo_address[19][2]);
	op_pk_nfd_set(pkptr,"Nei_height20",topo_address[20][2]);
	op_pk_nfd_set(pkptr,"Nei_height21",topo_address[21][2]);
	op_pk_nfd_set(pkptr,"Nei_height22",topo_address[22][2]);
	op_pk_nfd_set(pkptr,"Nei_height23",topo_address[23][2]);

	op_pk_send(pkptr,TX_OUT_2);

	//ready for next circle
	interact_pk_num+=1;
	if(interact_pk_num==16) interact_pk_num=0;//Num

	link_interact_pk_num=0;//pk collection

	for(i=0;i<24;i++)
		for(j=0;j<24;j++) //topo
			{
			last_topo[i][j]=topo[i][j];
			topo[i][j]=0;
			}

	FOUT
	}

/* End of Function Block */

/* Undefine optional tracing in FIN/FOUT/FRET */
/* The FSM has its own tracing code and the other */
/* functions should not have any tracing.		  */
#undef FIN_TRACING
#define FIN_TRACING

#undef FOUTRET_TRACING
#define FOUTRET_TRACING

#if defined (__cplusplus)
extern "C" {
#endif
	void net_V1 (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_net_V1_init (int * init_block_ptr);
	void _op_net_V1_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_net_V1_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_net_V1_alloc (VosT_Obtype, int);
	void _op_net_V1_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
net_V1 (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (net_V1 ());

		{
		/* Temporary Variables */
		Packet* pkptr;
		int i,j;
		
		/* End of Temporary Variables */


		FSM_ENTER ("net_V1")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "net_V1 [init enter execs]")
				FSM_PROFILE_SECTION_IN ("net_V1 [init enter execs]", state0_enter_exec)
				{
				//initial begin
				my_id = op_id_self();
				my_node_id = op_topo_parent (my_id);
				op_ima_obj_attr_get(my_node_id,"Address",&my_address);
				op_ima_sim_attr_get (OPC_IMA_INTEGER, "interactive_id", &interactive_id);
				
				in_net_num=0;
				to_interact_next_hop=0xFF;
				interact_pk_num=0;
				link_interact_pk_num=0;
				data_req_over=1;
				data_control_num=0;
				
				node_num=16;///
				
				for(i=0;i<24;i++)
					for(j=0;j<24;j++)
						{
						topo[i][j]=0;
						}
				for(i=0;i<24;i++)
					{
					topo_address[i][0]=0;
					topo_address[i][1]=0;
					topo_address[i][2]=0;
					}
				for(i=0;i<24;i++) route_des[i]=8;
				for(i=0;i<24;i++) node_in_net[i]=0;	
				for(i=0;i<24;i++) node_in_net_last[i]=0;	
				
				printf("$$$$$$$$$$$$net over$$$$$$$$$$$$$\n");
				
				
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "net_V1 [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_0", "net_V1 [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "net_V1 [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"net_V1")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "net_V1 [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("net_V1 [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (FROM_RX_PK)
			FSM_TEST_COND (FROM_SRC_PK)
			FSM_TEST_COND (COLLECT || TIME_OUT)
			FSM_TEST_COND (FROM_RX2_PK)
			FSM_TEST_COND (CHANGE_ROUTE )
			FSM_TEST_COND (REQ_OVER)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "FROM_RX_PK", "", "idle", "rx_rcv", "tr_3", "net_V1 [idle -> rx_rcv : FROM_RX_PK / ]")
				FSM_CASE_TRANSIT (1, 3, state3_enter_exec, ;, "FROM_SRC_PK", "", "idle", "src_rcv", "tr_6", "net_V1 [idle -> src_rcv : FROM_SRC_PK / ]")
				FSM_CASE_TRANSIT (2, 4, state4_enter_exec, ;, "COLLECT || TIME_OUT", "", "idle", "link_state_feedback", "tr_10", "net_V1 [idle -> link_state_feedback : COLLECT || TIME_OUT / ]")
				FSM_CASE_TRANSIT (3, 5, state5_enter_exec, ;, "FROM_RX2_PK", "", "idle", "rx2_rcv", "tr_12", "net_V1 [idle -> rx2_rcv : FROM_RX2_PK / ]")
				FSM_CASE_TRANSIT (4, 1, state1_enter_exec, change_route();, "CHANGE_ROUTE ", "change_route()", "idle", "idle", "tr_14", "net_V1 [idle -> idle : CHANGE_ROUTE  / change_route()]")
				FSM_CASE_TRANSIT (5, 1, state1_enter_exec, req_over();, "REQ_OVER", "req_over()", "idle", "idle", "tr_15", "net_V1 [idle -> idle : REQ_OVER / req_over()]")
				FSM_CASE_TRANSIT (6, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_1", "net_V1 [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (rx_rcv) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "rx_rcv", state2_enter_exec, "net_V1 [rx_rcv enter execs]")
				FSM_PROFILE_SECTION_IN ("net_V1 [rx_rcv enter execs]", state2_enter_exec)
				{
				//收到不同类型包的行为
				pkptr=op_pk_get(op_intrpt_strm());
				op_pk_nfd_get (pkptr, "TYPE", &type);
				
				if(type==0x00)//QWZ
					{
					QWZ_netrcv(pkptr);
					}
				
				if(type==0x01)//向交互节点发送的上报帧
					{
					Link_report_netrcv(pkptr);
					}
				
				if(type==0x12)//D_REP
					{
					op_pk_send(pkptr,TX_OUT_2);
					}
				
				if(type==0x20)
					{
					op_pk_send(pkptr,SINK_OUT);
					}
				
				if(type==0x81)//DATA
					{
					DATA_netrcv(pkptr);
					}
				
				
				
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (rx_rcv) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "rx_rcv", "net_V1 [rx_rcv exit execs]")


			/** state (rx_rcv) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "rx_rcv", "idle", "tr_5", "net_V1 [rx_rcv -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (src_rcv) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "src_rcv", state3_enter_exec, "net_V1 [src_rcv enter execs]")
				FSM_PROFILE_SECTION_IN ("net_V1 [src_rcv enter execs]", state3_enter_exec)
				{
				pkptr=op_pk_get(op_intrpt_strm());
				DATA_create(pkptr);
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (src_rcv) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "src_rcv", "net_V1 [src_rcv exit execs]")


			/** state (src_rcv) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "src_rcv", "idle", "tr_7", "net_V1 [src_rcv -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (link_state_feedback) enter executives **/
			FSM_STATE_ENTER_FORCED (4, "link_state_feedback", state4_enter_exec, "net_V1 [link_state_feedback enter execs]")
				FSM_PROFILE_SECTION_IN ("net_V1 [link_state_feedback enter execs]", state4_enter_exec)
				{
				Link_collect_over();
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** state (link_state_feedback) exit executives **/
			FSM_STATE_EXIT_FORCED (4, "link_state_feedback", "net_V1 [link_state_feedback exit execs]")


			/** state (link_state_feedback) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "link_state_feedback", "idle", "tr_11", "net_V1 [link_state_feedback -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (rx2_rcv) enter executives **/
			FSM_STATE_ENTER_FORCED (5, "rx2_rcv", state5_enter_exec, "net_V1 [rx2_rcv enter execs]")
				FSM_PROFILE_SECTION_IN ("net_V1 [rx2_rcv enter execs]", state5_enter_exec)
				{
				pkptr =  op_pk_get (op_intrpt_strm());
				op_pk_nfd_get (pkptr, "TYPE", &type);
				
				if(type==0x12)//D_REP
					{
					op_pk_send(pkptr,TX_OUT);
					}
				
				if(type==0x20)
					op_pk_send(pkptr,TX_OUT);
				
				if(type==0x11 && my_address==interactive_id)
					{
					Data_request_rcv(pkptr);
					}
				if(type==0x11 && my_address!=interactive_id) op_pk_destroy(pkptr);
				}
				FSM_PROFILE_SECTION_OUT (state5_enter_exec)

			/** state (rx2_rcv) exit executives **/
			FSM_STATE_EXIT_FORCED (5, "rx2_rcv", "net_V1 [rx2_rcv exit execs]")


			/** state (rx2_rcv) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "rx2_rcv", "idle", "tr_13", "net_V1 [rx2_rcv -> idle : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"net_V1")
		}
	}




void
_op_net_V1_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_net_V1_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_net_V1_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_net_V1_svar function. */
#undef my_address
#undef my_id
#undef my_node_id
#undef type
#undef interactive_id
#undef to_interact_next_hop
#undef interact_pk_num
#undef topo
#undef link_interact_pk_num
#undef evh
#undef node_num
#undef topo_address
#undef route_des
#undef node_in_net
#undef data_control_num
#undef node_in_net_last
#undef in_net_num
#undef data_req_over
#undef last_topo
#undef route_num
#undef route
#undef dist
#undef mark
#undef path
#undef A

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_net_V1_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_net_V1_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (net_V1)",
		sizeof (net_V1_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_net_V1_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	net_V1_state * ptr;
	FIN_MT (_op_net_V1_alloc (obtype))

	ptr = (net_V1_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "net_V1 [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_net_V1_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	net_V1_state		*prs_ptr;

	FIN_MT (_op_net_V1_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (net_V1_state *)gen_ptr;

	if (strcmp ("my_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_address);
		FOUT
		}
	if (strcmp ("my_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_id);
		FOUT
		}
	if (strcmp ("my_node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_node_id);
		FOUT
		}
	if (strcmp ("type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->type);
		FOUT
		}
	if (strcmp ("interactive_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->interactive_id);
		FOUT
		}
	if (strcmp ("to_interact_next_hop" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->to_interact_next_hop);
		FOUT
		}
	if (strcmp ("interact_pk_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->interact_pk_num);
		FOUT
		}
	if (strcmp ("topo" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->topo);
		FOUT
		}
	if (strcmp ("link_interact_pk_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->link_interact_pk_num);
		FOUT
		}
	if (strcmp ("evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->evh);
		FOUT
		}
	if (strcmp ("node_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_num);
		FOUT
		}
	if (strcmp ("topo_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->topo_address);
		FOUT
		}
	if (strcmp ("route_des" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->route_des);
		FOUT
		}
	if (strcmp ("node_in_net" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->node_in_net);
		FOUT
		}
	if (strcmp ("data_control_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_control_num);
		FOUT
		}
	if (strcmp ("node_in_net_last" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->node_in_net_last);
		FOUT
		}
	if (strcmp ("in_net_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->in_net_num);
		FOUT
		}
	if (strcmp ("data_req_over" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_req_over);
		FOUT
		}
	if (strcmp ("last_topo" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->last_topo);
		FOUT
		}
	if (strcmp ("route_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->route_num);
		FOUT
		}
	if (strcmp ("route" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->route);
		FOUT
		}
	if (strcmp ("dist" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->dist);
		FOUT
		}
	if (strcmp ("mark" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->mark);
		FOUT
		}
	if (strcmp ("path" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->path);
		FOUT
		}
	if (strcmp ("A" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->A);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

