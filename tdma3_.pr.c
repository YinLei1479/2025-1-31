/* Process model C form file: tdma3_.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char tdma3__pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A op_runsim 7 6788A058 6788A058 1 ray-laptop 28918 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                                       ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include <math.h>

/* Constant Definitions */
#define RX_IN_STRM		(1)
#define SRC_IN_STRM		(0)
#define TX_OUT_STRM		(1)
#define SINK_OUT_STRM	(0)

#define EPSILON  		(1e-10)  /* rounding error factor */
#define TDMA_COMPLETE	(-10)
#define TIME_OUT_1		(2501)////
#define TIME_OUT_2		(2502)
#define TIME_OUT_3		(2503)
#define TIME_OUT_4		(2504)
#define TIME_OUT_5		(2505)
#define ROUTE_CHANGE	(3110)
#define REQ_OVER_CODE	(3111)


/* Transition Condition Macros */ 
#define NET_BUILD		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == 11)
#define RX_STRM			(op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm() == RX_IN_STRM)
#define FROM_RX			(current_intrpt_type == OPC_INTRPT_STRM) && (op_intrpt_strm () == RX_IN_STRM)
#define FROM_SRC 		(current_intrpt_type == OPC_INTRPT_STRM) && (op_intrpt_strm () == SRC_IN_STRM) 
#define SLOT 			(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == 0)
#define NO_DATA 		((op_subq_empty (0)) && (op_subq_empty (1)) && (op_subq_empty (2)))
#define RE_SEND_1		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_1)////
#define RE_SEND_2		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_2)////
#define RE_SEND_3		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_3)////
#define RE_SEND_4		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_4)////
#define RE_SEND_5		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_5)////
#define RE_SEND			RE_SEND_1||RE_SEND_2||RE_SEND_3||RE_SEND_4||RE_SEND_5

static void QWZ_create(void);
static void QWZ_rcv(Packet* pkptr);


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
	int	                    		my_offset                                       ;
	double	                 		slot_length                                     ;
	double	                 		tx_data_rate                                    ;
	Objid	                  		my_node_id                                      ;
	Objid	                  		my_id                                           ;
	int	                    		my_address                                      ;
	int	                    		type                                            ;
	int	                    		nei_count                                       ;	/* number of neighbor */
	int	                    		my_two_nei[24]                                  ;	/* two hop neighbor address */
	int	                    		is_my_slot                                      ;	/* 0 or 1 for is my slot ? */
	int	                    		interactive_id                                  ;	/* interactive node address */
	int	                    		WAIT                                            ;	/* need still to wait */
	int	                    		num_slots                                       ;	/* all slot number */
	int	                    		my_node_state                                   ;
	int	                    		my_clock_level                                  ;
	int	                    		a_frame_record[24][7]                           ;	/* record in a frame                                         */
	                        		                                                	/* node_id + nei_offset + CL + net_id + node_state +rcv_time */
	int	                    		a_frame_record_last[24][7]                      ;	/* neighbor in last frame */
	int	                    		nei_num_last                                    ;
	int	                    		node_num                                        ;
	double	                 		Longitude                                       ;
	double	                 		Latitude                                        ;
	double	                 		Height                                          ;
	int	                    		time[24]                                        ;
	int	                    		TTL                                             ;
	Packet *	               		remain_pk[5]                                    ;
	Evhandle	               		evh[5]                                          ;
	int	                    		remain_num[5]                                   ;
	int	                    		control_num                                     ;
	int	                    		control_continue                                ;
	int	                    		my_data_slot                                    ;
	int	                    		data_slot_num                                   ;
	int	                    		data_frame_num                                  ;
	int	                    		DATA_frame[240]                                 ;
	int	                    		data_slot_num_all                               ;
	int	                    		data_slot_now                                   ;
	int	                    		link_maintain_TTL                               ;
	Packet *	               		link_maintain_pk                                ;
	double	                 		link_maintain_time                              ;
	int	                    		last_data_frame[240]                            ;
	int	                    		TTL2                                            ;
	int	                    		new_data_frame[240]                             ;
	int	                    		is_send                                         ;
	int	                    		data_next                                       ;
	int	                    		my_net_id                                       ;
	int	                    		NO_NEI                                          ;
	double	                 		time_left_in_slot1                              ;
	int	                    		FLAG1                                           ;
	int	                    		n                                               ;
	int	                    		sync_state                                      ;
	double	                 		rcv_time[24]                                    ;
	Evhandle	               		evh2                                            ;
	double	                 		slot_start                                      ;
	int	                    		sy_base_id                                      ;
	int	                    		retr                                            ;
	int	                    		round_num                                       ;
	int	                    		round_num_now                                   ;
	} tdma3__state;

#define my_offset               		op_sv_ptr->my_offset
#define slot_length             		op_sv_ptr->slot_length
#define tx_data_rate            		op_sv_ptr->tx_data_rate
#define my_node_id              		op_sv_ptr->my_node_id
#define my_id                   		op_sv_ptr->my_id
#define my_address              		op_sv_ptr->my_address
#define type                    		op_sv_ptr->type
#define nei_count               		op_sv_ptr->nei_count
#define my_two_nei              		op_sv_ptr->my_two_nei
#define is_my_slot              		op_sv_ptr->is_my_slot
#define interactive_id          		op_sv_ptr->interactive_id
#define WAIT                    		op_sv_ptr->WAIT
#define num_slots               		op_sv_ptr->num_slots
#define my_node_state           		op_sv_ptr->my_node_state
#define my_clock_level          		op_sv_ptr->my_clock_level
#define a_frame_record          		op_sv_ptr->a_frame_record
#define a_frame_record_last     		op_sv_ptr->a_frame_record_last
#define nei_num_last            		op_sv_ptr->nei_num_last
#define node_num                		op_sv_ptr->node_num
#define Longitude               		op_sv_ptr->Longitude
#define Latitude                		op_sv_ptr->Latitude
#define Height                  		op_sv_ptr->Height
#define time                    		op_sv_ptr->time
#define TTL                     		op_sv_ptr->TTL
#define remain_pk               		op_sv_ptr->remain_pk
#define evh                     		op_sv_ptr->evh
#define remain_num              		op_sv_ptr->remain_num
#define control_num             		op_sv_ptr->control_num
#define control_continue        		op_sv_ptr->control_continue
#define my_data_slot            		op_sv_ptr->my_data_slot
#define data_slot_num           		op_sv_ptr->data_slot_num
#define data_frame_num          		op_sv_ptr->data_frame_num
#define DATA_frame              		op_sv_ptr->DATA_frame
#define data_slot_num_all       		op_sv_ptr->data_slot_num_all
#define data_slot_now           		op_sv_ptr->data_slot_now
#define link_maintain_TTL       		op_sv_ptr->link_maintain_TTL
#define link_maintain_pk        		op_sv_ptr->link_maintain_pk
#define link_maintain_time      		op_sv_ptr->link_maintain_time
#define last_data_frame         		op_sv_ptr->last_data_frame
#define TTL2                    		op_sv_ptr->TTL2
#define new_data_frame          		op_sv_ptr->new_data_frame
#define is_send                 		op_sv_ptr->is_send
#define data_next               		op_sv_ptr->data_next
#define my_net_id               		op_sv_ptr->my_net_id
#define NO_NEI                  		op_sv_ptr->NO_NEI
#define time_left_in_slot1      		op_sv_ptr->time_left_in_slot1
#define FLAG1                   		op_sv_ptr->FLAG1
#define n                       		op_sv_ptr->n
#define sync_state              		op_sv_ptr->sync_state
#define rcv_time                		op_sv_ptr->rcv_time
#define evh2                    		op_sv_ptr->evh2
#define slot_start              		op_sv_ptr->slot_start
#define sy_base_id              		op_sv_ptr->sy_base_id
#define retr                    		op_sv_ptr->retr
#define round_num               		op_sv_ptr->round_num
#define round_num_now           		op_sv_ptr->round_num_now

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	tdma3__state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((tdma3__state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

static void QWZ_create(void)//QWZ_CREAT
	{
	Packet* pkptr;
	int min;
	int i;
	
	FIN(QWZ_create())

	pkptr=op_pk_create_fmt("QWZ");

	op_pk_nfd_set (pkptr, "SEND", my_address);
	op_pk_nfd_set (pkptr, "TYPE", 0x00);
	op_pk_nfd_set (pkptr, "node_state", my_node_state);
	op_pk_nfd_set (pkptr, "NET_ID", my_net_id);
	op_pk_nfd_set (pkptr, "None",0);
	op_pk_nfd_set (pkptr, "Nei_num", nei_count);
	op_pk_nfd_set (pkptr, "Slot", my_offset);
	op_pk_nfd_set (pkptr, "Nei_address_0",a_frame_record[round_num_now*8+0][0]);
	op_pk_nfd_set (pkptr, "Nei_address_1",a_frame_record[round_num_now*8+1][0]);
	op_pk_nfd_set (pkptr, "Nei_address_2",a_frame_record[round_num_now*8+2][0]);
	op_pk_nfd_set (pkptr, "Nei_address_3",a_frame_record[round_num_now*8+3][0]);
	op_pk_nfd_set (pkptr, "Nei_address_4",a_frame_record[round_num_now*8+4][0]);
	op_pk_nfd_set (pkptr, "Nei_address_5",a_frame_record[round_num_now*8+5][0]);
	op_pk_nfd_set (pkptr, "Nei_address_6",a_frame_record[round_num_now*8+6][0]);
	op_pk_nfd_set (pkptr, "Nei_address_7",a_frame_record[round_num_now*8+7][0]);

	op_pk_nfd_set (pkptr, "Timer_Value_0",a_frame_record[round_num_now*8+0][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_1",a_frame_record[round_num_now*8+1][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_2",a_frame_record[round_num_now*8+2][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_3",a_frame_record[round_num_now*8+3][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_4",a_frame_record[round_num_now*8+4][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_5",a_frame_record[round_num_now*8+5][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_6",a_frame_record[round_num_now*8+6][6]);
	op_pk_nfd_set (pkptr, "Timer_Value_7",a_frame_record[round_num_now*8+7][6]);
	min=0xFF;
	for(i=0;i<24;i++)
		{
		if(a_frame_record[i][0]!=0xFF && a_frame_record[i][2]<min)
			{
			min=a_frame_record[i][2];
			}
		}
	if(my_clock_level>min) my_clock_level=min+1;
	
	op_pk_nfd_set (pkptr, "Clock_level", my_clock_level);
	op_pk_print(pkptr);
	op_pk_send(pkptr,TX_OUT_STRM);
	round_num_now++;
	if(round_num_now==round_num) round_num_now=0;
	FOUT
	}


static void QWZ_rcv(Packet* pkptr)//QWZ_RCV
	{
	int nei_id;
	int nei_slot;
	int nei_CL;
	int nei_state;
	int nei_net_id;
	int two_nei_num;
	int two_nei_id[8];
	int error11[8];
	int i;
	
	FIN(QWZ_rcv(Packet* pkptr))
	retr=-1;
	op_pk_nfd_get (pkptr, "SEND", &nei_id);
	op_pk_nfd_get (pkptr, "Slot", &nei_slot);
	op_pk_nfd_get (pkptr, "node_state", &nei_state);
	op_pk_nfd_get (pkptr, "Clock_level", &nei_CL);
	op_pk_nfd_get (pkptr, "NET_ID", &nei_net_id);
	
	//record
	for(i=0;i<nei_count;i++)
		{
		if(a_frame_record[i][0]==nei_id)//本轮已经记录过
			{
			a_frame_record[i][0]=nei_id;
			a_frame_record[i][1]=nei_slot;
			a_frame_record[i][2]=nei_CL;
			a_frame_record[i][3]=nei_net_id;
			a_frame_record[i][4]=nei_state;
			rcv_time[i]=op_sim_time();
			break;
			}
		}
	if(i==nei_count && nei_count<24)
		{
		a_frame_record[nei_count][0]=nei_id;
		a_frame_record[nei_count][1]=nei_slot;
		a_frame_record[nei_count][2]=nei_CL;
		a_frame_record[nei_count][3]=nei_net_id;
		a_frame_record[nei_count][4]=nei_state;
		//rcv_time[i]=op_sim_time();
		nei_count++;
		}
					
	//neighbor			
	op_pk_nfd_get (pkptr, "Nei_num", &two_nei_num);
	op_pk_nfd_get (pkptr, "Nei_address_0", &two_nei_id[0]);
	op_pk_nfd_get (pkptr, "Nei_address_1", &two_nei_id[1]);
	op_pk_nfd_get (pkptr, "Nei_address_2", &two_nei_id[2]);
	op_pk_nfd_get (pkptr, "Nei_address_3", &two_nei_id[3]);
	op_pk_nfd_get (pkptr, "Nei_address_4", &two_nei_id[4]);
	op_pk_nfd_get (pkptr, "Nei_address_5", &two_nei_id[5]);
	op_pk_nfd_get (pkptr, "Nei_address_6", &two_nei_id[6]);
	op_pk_nfd_get (pkptr, "Nei_address_7", &two_nei_id[7]);
	for(i=0;i<8;i++)
		if(two_nei_id[0]!=0xFF)
		  my_two_nei[two_nei_id[i]]=2;
	//节点状态更改
	for(i=0;i<8;i++)
		{
		if(two_nei_id[i]==my_address && my_node_state==1)
			my_node_state=2;
		}
	if(nei_id==sy_base_id)
		{
		op_pk_nfd_get (pkptr, "Timer_Value_0", &error11[0]);
		op_pk_nfd_get (pkptr, "Timer_Value_1", &error11[1]);
		op_pk_nfd_get (pkptr, "Timer_Value_2", &error11[2]);
		op_pk_nfd_get (pkptr, "Timer_Value_3", &error11[3]);
		op_pk_nfd_get (pkptr, "Timer_Value_4", &error11[4]);
		op_pk_nfd_get (pkptr, "Timer_Value_5", &error11[5]);
		op_pk_nfd_get (pkptr, "Timer_Value_6", &error11[6]);
		op_pk_nfd_get (pkptr, "Timer_Value_7", &error11[7]);
		
		for(i=0;i<8;i++)
			if(my_address==two_nei_id[i])
				retr=error11[i];
		}
	
	op_pk_send(pkptr,SINK_OUT_STRM);
	FOUT
	//FRET(retr)
	}

static void Net_build(void)
	{
	double current_time;
	int used_slots;
	int current_offset;
	
	FIN(Net_build(void))
	my_net_id=my_address;
	if(my_address!=interactive_id) my_node_state=2;
	if(my_address!=interactive_id) my_clock_level=1;
	current_time=op_sim_time();
	used_slots=(int)floor((current_time/slot_length)+EPSILON);
	current_offset=used_slots%num_slots;
	my_offset=current_offset;
	time[my_offset]=1;
	printf("current_time:%lf, used_slots:%d, current_offset:%d num_slots:%d\n",current_time,used_slots,current_offset,num_slots);
	printf("建网%d::%d::%d\n",my_address,my_offset,my_net_id);
	FOUT
	}

static int Net_in(void)
	{
	int base_num;
	int FF;
	int i;

//	double current_time;
//	int used_slots;
//	int current_offset;
	
	FIN(Net_in(void))
	//计算时基节点a_frame_record[base_num][0]
	base_num=0;
	FF=0;
	for(i=0;i<nei_count;i++)
		{
		if(a_frame_record[i][3]==interactive_id)
			{
			base_num=i;
			FF=1;
			break;
			}
		}
	if(FF==1)//交互在网
		{
		for(i=base_num;i<nei_count;i++)
			{
			if(a_frame_record[i][3]==interactive_id && a_frame_record[i][2]<a_frame_record[base_num][2])
				base_num=i;
			}
		}
	else
		{
		for(i=1;i<nei_count;i++)
			{
			if(a_frame_record[i][3]<a_frame_record[base_num][3]) base_num=i;
			if(a_frame_record[i][3]==a_frame_record[base_num][3] && a_frame_record[i][2]<a_frame_record[base_num][2])
				base_num=i;
			}
		}
	
	//调整自身参数
	my_net_id=a_frame_record[base_num][3];
	my_node_state=1;
	my_clock_level=a_frame_record[base_num][2]+1;
	my_offset=my_address-a_frame_record[base_num][0]+a_frame_record[base_num][1];
	if(my_offset<0) my_offset=my_offset+num_slots;
	my_offset=my_offset%num_slots;
	time[my_offset]=1;
	/*current_time=op_sim_time();
	used_slots=(int)floor((current_time/slot_length)+EPSILON);
	current_offset=used_slots%num_slots;
	printf("current_time:%lf, used_slots:%d, current_offset:%d num_slots:%d\n",current_time,used_slots,current_offset,num_slots);
	printf("待入网%d::%d::%d\n",my_address,my_offset,my_net_id);
	printf("基准节点：%d\n",a_frame_record[base_num][0]);
	for(i=0;i<8;i++)
		{
		for(j=0;j<5;j++)
			printf("%d  ",a_frame_record[i][j]);
		printf("\n");
		}*/
	FRET(base_num)
	}

static void Link_report_mac_rx(Packet *pkptr)
	{
	int next_hop;
	
	FIN(Link_report_mac_rx(Packet *pkptr))
	op_pk_nfd_get (pkptr, "Next_Hop", &next_hop);
	if(next_hop!=my_address) 
		{
		op_pk_destroy(pkptr);
		}
	else 
		{
		op_pk_send(pkptr,SINK_OUT_STRM);
		}
	FOUT
	}

static void Data_control_mac_rx(Packet *pkptr)
	{
	int pknum;
	int in_net[24];
	int i;
	int j;
	int prio;
	
	FIN(Data_control_mac_rx(Packet *pkptr))
	op_pk_nfd_get(pkptr,"NUM",&pknum);
	if(pknum!=control_num) op_pk_destroy(pkptr);
	else
		{
		control_num=(control_num+1)%16;
		op_pk_nfd_get(pkptr,"TTL",&TTL);
		op_pk_nfd_get(pkptr,"Continue",&control_continue);
		
		op_pk_nfd_get(pkptr,"Slot_0",&in_net[0]);
		op_pk_nfd_get(pkptr,"Slot_1",&in_net[1]);
		op_pk_nfd_get(pkptr,"Slot_2",&in_net[2]);
		op_pk_nfd_get(pkptr,"Slot_3",&in_net[3]);
		op_pk_nfd_get(pkptr,"Slot_4",&in_net[4]);
		op_pk_nfd_get(pkptr,"Slot_5",&in_net[5]);
		op_pk_nfd_get(pkptr,"Slot_6",&in_net[6]);
		op_pk_nfd_get(pkptr,"Slot_7",&in_net[7]);
		op_pk_nfd_get(pkptr,"Slot_8",&in_net[8]);
		op_pk_nfd_get(pkptr,"Slot_9",&in_net[9]);
		op_pk_nfd_get(pkptr,"Slot_10",&in_net[10]);
		op_pk_nfd_get(pkptr,"Slot_11",&in_net[11]);
		op_pk_nfd_get(pkptr,"Slot_12",&in_net[12]);
		op_pk_nfd_get(pkptr,"Slot_13",&in_net[13]);
		op_pk_nfd_get(pkptr,"Slot_14",&in_net[14]);
		op_pk_nfd_get(pkptr,"Slot_15",&in_net[15]);
		op_pk_nfd_get(pkptr,"Slot_16",&in_net[16]);
		op_pk_nfd_get(pkptr,"Slot_17",&in_net[17]);
		op_pk_nfd_get(pkptr,"Slot_18",&in_net[18]);
		op_pk_nfd_get(pkptr,"Slot_19",&in_net[19]);
		op_pk_nfd_get(pkptr,"Slot_20",&in_net[20]);
		op_pk_nfd_get(pkptr,"Slot_21",&in_net[21]);
		op_pk_nfd_get(pkptr,"Slot_22",&in_net[22]);
		op_pk_nfd_get(pkptr,"Slot_23",&in_net[23]);
		
		if(control_continue==0)
			{
			for(i=0;i<24;i++)
				{
				if(in_net[i]==my_address) my_data_slot=i;
				if(in_net[i]==31) break;
				}
			data_slot_num=i;
			}
		else
			{
			is_send=in_net[0];
			for(i=0;i<24;i++)
				{
				if(my_address==in_net[i])
					{
					for(j=i;j<24;j++)
						if(in_net[j]!=my_address)
							{
							if(in_net[j]==0xFF) data_next=interactive_id;
							else data_next=in_net[j];
							break;
							}
					}
				break;
				}
			for(i=0;i<240;i++) new_data_frame[i]=0;
			for(i=0;i<data_frame_num;i++)
				for(j=0;j<data_slot_num;j++)
					if(my_address==in_net[j]) new_data_frame[i*data_slot_num+j]=1;
			new_data_frame[data_slot_num*data_frame_num+my_offset]=2;
			}
		
		op_pk_nfd_get(pkptr,"Priority",&prio);
		if(prio==0) op_subq_pk_insert (1, pkptr, OPC_QPOS_TAIL);
		else op_subq_pk_insert (0, pkptr, OPC_QPOS_TAIL);
		}
	FOUT
	}

static void Appoint_mac_rcv(Packet *pkptr)
	{
	int next_hop;
	int send;
	int pknum;
	int dest;
	int i;
	int remote_control_route[5];
	
	FIN(Appoint_mac_rcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"Next_hop",&next_hop);
	if(my_address!=next_hop) op_pk_destroy(pkptr);
	else
		{
		//ACK
		op_pk_nfd_get(pkptr,"SEND",&send);
		op_pk_nfd_get(pkptr,"Pk_num",&pknum);	
		//处理
		op_pk_nfd_get(pkptr,"DEST",&dest);
		if(my_address==dest)
			op_pk_send(pkptr,SINK_OUT_STRM);
		else
			{
			for(i=0;i<5;i++)
				{
				if(remain_num[i]==-1)
					break;
				}
			op_pk_nfd_get(pkptr,"Hop1_MES",&remote_control_route[0]);
			op_pk_nfd_get(pkptr,"Hop2_MES",&remote_control_route[1]);
			op_pk_nfd_get(pkptr,"Hop3_MES",&remote_control_route[2]);
			op_pk_nfd_get(pkptr,"Hop4_MES",&remote_control_route[3]);
			remote_control_route[4]=dest;
			for(i=0;i<4;i++)
				if(remote_control_route[i]==my_address)
					{
					op_pk_nfd_set(pkptr,"Next_hop",remote_control_route[i+1]);
					break;
					}
			//in fifo
			remain_num[i]=pknum;
			remain_pk[i]=op_pk_copy (pkptr);
			if(i==0) evh[0]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_1);
			if(i==1) evh[1]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_2);
			if(i==2) evh[2]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_3);
			if(i==3) evh[3]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_4);
			if(i==4) evh[4]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_5);
			op_subq_pk_insert (1, pkptr, OPC_QPOS_TAIL);
			}
		pkptr=op_pk_create_fmt("ACK");
		op_pk_nfd_set(pkptr,"SEND",my_address);
		op_pk_nfd_set(pkptr,"TYPE",0x21);
		op_pk_nfd_set(pkptr,"DEST",send);
		op_pk_nfd_set(pkptr,"PK_NUM",pknum);
		op_subq_pk_insert (0, pkptr, OPC_QPOS_TAIL);
		
		}
	FOUT
	}

static void ACK_rcv(Packet *pkptr)
	{
	int dest;
	int send;
	int pknum;
	int FF;
	int i;
	
	FIN(ACK_rcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"SEND",&dest);
	op_pk_nfd_get(pkptr,"DEST",&send);
	op_pk_nfd_get(pkptr,"PK_NUM",&pknum);
	if(my_address!=send) op_pk_destroy(pkptr);
	else
		{
		FF=0;
		for(i=0;i<5;i++)
			{
			if(pknum==remain_num[i])
				{
				FF=1;
				printf("%d  ACK get form %d !!! time:%lf\n",my_address,dest,op_sim_time());
				op_ev_cancel(evh[i]);
				op_pk_destroy(remain_pk[i]);
				remain_num[i]=-1;
				}
			}
		if(FF==0) op_pk_destroy(pkptr);
		}
	FOUT
	}

static void DATA_rcv(Packet *pkptr)
	{
	int next_hop;
	
	FIN(DATA_rcv(Packet *pkptr))
	op_pk_nfd_get(pkptr,"Recv",&next_hop);
	if(next_hop!=my_address) op_pk_destroy(pkptr);
	else 
		{
		op_pk_send(pkptr,SINK_OUT_STRM);
		}
	FOUT
	}

static void Link_report_frnet(Packet *pkptr)
	{
	int dest;
	double pk_len;
	
	FIN(Link_report_frnet(Packet *pkptr))
	op_pk_nfd_get(pkptr,"SEND",&dest);
	if(dest==my_address)
		{
		link_maintain_pk=op_pk_copy(pkptr);
		link_maintain_TTL=3;
		op_pk_destroy(pkptr);
		pk_len = (double) op_pk_total_size_get (link_maintain_pk);
		link_maintain_time=(double) pk_len / tx_data_rate;
		}
	else op_subq_pk_insert (0, pkptr, OPC_QPOS_TAIL);
	FOUT
	}

static void Appoint_frnet(Packet *pkptr)
	{
	int dest;
	int i;
	FIN(Appoint_frnet(Packet *pkptr))
	op_pk_nfd_get(pkptr,"DEST",&dest);
	if(dest==my_address) op_pk_send(pkptr,SINK_OUT_STRM);
	else
		{
		for(i=0;i<5;i++)
			{
			if(remain_num[i]==-1)
				break;
			}
		op_pk_nfd_get(pkptr,"Hop1_MES",&dest);
		op_pk_nfd_set(pkptr,"Next_hop",dest);
		op_pk_nfd_get(pkptr,"Pk_num",&remain_num[i]);	
		remain_pk[i]=op_pk_copy (pkptr);
		op_subq_pk_insert (1, pkptr, OPC_QPOS_TAIL);
		if(i==0) evh[0]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_1);
		if(i==1) evh[1]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_2);
		if(i==2) evh[2]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_3);
		if(i==3) evh[3]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_4);
		if(i==4) evh[4]=op_intrpt_schedule_self(op_sim_time()+20,TIME_OUT_5);
		}
	FOUT
	}

static void Data_control_frnet(Packet *pkptr)
	{
	int pknum1;
	int in_net[24];
	int i;
	int j;
	int prio;
	
	FIN(Data_control_frnet(Packet *pkptr))
	op_pk_nfd_get(pkptr,"NUM",&pknum1);
	if(pknum1!=control_num) op_pk_destroy(pkptr);
	else
		{
		control_num=(control_num+1)%16;
		op_pk_nfd_get(pkptr,"TTL",&TTL);
		op_pk_nfd_get(pkptr,"Continue",&control_continue);
		
		op_pk_nfd_get(pkptr,"Slot_0",&in_net[0]);
		op_pk_nfd_get(pkptr,"Slot_1",&in_net[1]);
		op_pk_nfd_get(pkptr,"Slot_2",&in_net[2]);
		op_pk_nfd_get(pkptr,"Slot_3",&in_net[3]);
		op_pk_nfd_get(pkptr,"Slot_4",&in_net[4]);
		op_pk_nfd_get(pkptr,"Slot_5",&in_net[5]);
		op_pk_nfd_get(pkptr,"Slot_6",&in_net[6]);
		op_pk_nfd_get(pkptr,"Slot_7",&in_net[7]);
		op_pk_nfd_get(pkptr,"Slot_8",&in_net[8]);
		op_pk_nfd_get(pkptr,"Slot_9",&in_net[9]);
		op_pk_nfd_get(pkptr,"Slot_10",&in_net[10]);
		op_pk_nfd_get(pkptr,"Slot_11",&in_net[11]);
		op_pk_nfd_get(pkptr,"Slot_12",&in_net[12]);
		op_pk_nfd_get(pkptr,"Slot_13",&in_net[13]);
		op_pk_nfd_get(pkptr,"Slot_14",&in_net[14]);
		op_pk_nfd_get(pkptr,"Slot_15",&in_net[15]);
		op_pk_nfd_get(pkptr,"Slot_16",&in_net[16]);
		op_pk_nfd_get(pkptr,"Slot_17",&in_net[17]);
		op_pk_nfd_get(pkptr,"Slot_18",&in_net[18]);
		op_pk_nfd_get(pkptr,"Slot_19",&in_net[19]);
		op_pk_nfd_get(pkptr,"Slot_20",&in_net[20]);
		op_pk_nfd_get(pkptr,"Slot_21",&in_net[21]);
		op_pk_nfd_get(pkptr,"Slot_22",&in_net[22]);
		op_pk_nfd_get(pkptr,"Slot_23",&in_net[23]);
		
		if(control_continue==0)
			{
			for(i=0;i<24;i++)
				{
				if(in_net[i]==my_address) my_data_slot=i;
				if(in_net[i]==31) break;
				}
			data_slot_num=i;
			}
		else
			{
			is_send=in_net[0];
			for(i=0;i<24;i++)
				{
				if(my_address==in_net[i])
					{
					for(j=i;j<24;j++)
						if(in_net[j]!=my_address)
							{
							if(in_net[j]==0xFF) data_next=interactive_id;
							else data_next=in_net[j];
							break;
							}
					}
				break;
				}
			for(i=0;i<240;i++) new_data_frame[i]=0;
			for(i=0;i<data_frame_num;i++)
				for(j=0;j<data_slot_num;j++)
					if(my_address==in_net[j]) new_data_frame[i*data_slot_num+j]=1;
			new_data_frame[data_slot_num*data_frame_num+my_offset]=2;
			}
		
		op_pk_nfd_get(pkptr,"Priority",&prio);
		if(prio==0) op_subq_pk_insert (1, pkptr, OPC_QPOS_TAIL);
		else op_subq_pk_insert (0, pkptr, OPC_QPOS_TAIL);
		}	
	FOUT
	}

static void Data_frnet(Packet *pkptr)
	{
	int dest;
	
	FIN(Data_frnet(Packet *pkptr))
	if(TTL2!=-1)
		{
		if(data_next!=0xFF) op_pk_nfd_set(pkptr,"Recv",data_next);
		op_pk_nfd_get(pkptr,"Src",&dest);
		if(is_send==my_address) op_subq_pk_insert (2, pkptr, OPC_QPOS_TAIL);
		else if(is_send!=my_address && dest==is_send) op_subq_pk_insert (0, pkptr, OPC_QPOS_TAIL);
		else op_pk_destroy(pkptr);
		}
	else op_subq_pk_insert (2, pkptr, OPC_QPOS_TAIL);
	FOUT
	}

static void Re_send(void)
	{
	Packet *pkptr;
	FIN(Re_send(void))
	if(op_intrpt_type ()==RE_SEND_1)
		{
		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());
		pkptr=op_pk_copy (remain_pk[0]);
		op_pk_send(pkptr,TX_OUT_STRM);
		}
	if(op_intrpt_type ()==RE_SEND_2)
		{
		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());
		pkptr=op_pk_copy (remain_pk[1]);
		op_pk_send(pkptr,TX_OUT_STRM);
		}
	if(op_intrpt_type ()==RE_SEND_3)
		{
		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());
		pkptr=op_pk_copy (remain_pk[2]);
		op_pk_send(pkptr,TX_OUT_STRM);
		}
	if(op_intrpt_type ()==RE_SEND_4)
		{
		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());
		pkptr=op_pk_copy (remain_pk[3]);
		op_pk_send(pkptr,TX_OUT_STRM);
		}
	if(op_intrpt_type ()==RE_SEND_5)
		{
		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());
		pkptr=op_pk_copy (remain_pk[4]);
		op_pk_send(pkptr,TX_OUT_STRM);
		}
	FOUT
	}

static void Link_report_create(void)
	{
	int i;
	Packet *pkptr;
	
	FIN(Link_report_create(void))
	pkptr=op_pk_create_fmt("link_report");
	op_pk_nfd_set(pkptr,"TYPE",0x01);
	op_pk_nfd_set(pkptr,"Source",my_address);
	op_pk_nfd_set (pkptr, "Nei_address_0", a_frame_record_last[0][0]);
	op_pk_nfd_set (pkptr, "Nei_address_1", a_frame_record_last[1][0]);
	op_pk_nfd_set (pkptr, "Nei_address_2", a_frame_record_last[2][0]);
	op_pk_nfd_set (pkptr, "Nei_address_3", a_frame_record_last[3][0]);
	op_pk_nfd_set (pkptr, "Nei_address_4", a_frame_record_last[4][0]);
	op_pk_nfd_set (pkptr, "Nei_address_5", a_frame_record_last[5][0]);
	op_pk_nfd_set (pkptr, "Nei_address_6", a_frame_record_last[6][0]);
	op_pk_nfd_set (pkptr, "Nei_address_7", a_frame_record_last[7][0]);
	op_pk_nfd_set(pkptr,"Longitude",Longitude);
	op_pk_nfd_set(pkptr,"Latitude",Latitude);
	op_pk_nfd_set(pkptr,"Height",Height);
	//发送至NET
	for(i=0;i<8;i++)
		if(a_frame_record_last[i][0]==0xFF)
			break;
	op_pk_nfd_set (pkptr, "Nei_num",i);
	op_pk_send(pkptr,SINK_OUT_STRM);
	FOUT
	}

static void Dequeue(void)
	{
	int FLAG=0;
	double used_time;
	Packet *pkptr;
	double pk_len;
	double pk_time;
	
	
	FIN(Dequeue(void))
	if(NO_DATA && link_maintain_TTL!=-1 )
		{
		pkptr=op_pk_copy(link_maintain_pk);
		FLAG=1;
		op_pk_destroy(link_maintain_pk);
		link_maintain_TTL=-1;
		used_time=0.088;
		}

	/* dequeue the packet */
	if(my_node_state!=3 || (my_node_state==3 && DATA_frame[data_slot_now]==2))
		{
		if(op_subq_empty(0)==0 && FLAG==0)
			{
			pkptr=op_subq_pk_access (0, OPC_QPOS_HEAD);
			pk_len = (double) op_pk_total_size_get (pkptr); 
			pk_time = (double) pk_len / tx_data_rate;
			if(pk_time < time_left_in_slot1)
				{
				pkptr=op_subq_pk_remove(0, OPC_QPOS_HEAD);
				FLAG=1;
				used_time=pk_time;
				}
			}
		else if(op_subq_empty(1)==0 && FLAG==0)
			{
			pkptr=op_subq_pk_access (1, OPC_QPOS_HEAD);
			pk_len = (double) op_pk_total_size_get (pkptr); 
			pk_time = (double) pk_len / tx_data_rate;
			if(pk_time < time_left_in_slot1)
				{
				pkptr=op_subq_pk_remove(1, OPC_QPOS_HEAD);
				FLAG=1;
				used_time=pk_time;
				}
			}
		else if(op_subq_empty(2)==0 && FLAG==0)
			{
			if(link_maintain_TTL!=-1 && link_maintain_time< time_left_in_slot1)
				{
				pkptr=op_pk_copy(link_maintain_pk);
				FLAG=1;
				op_pk_destroy(link_maintain_pk);
				link_maintain_TTL=-1;
				used_time=pk_time;
				}
			else
				{
				pkptr=op_subq_pk_access (2, OPC_QPOS_HEAD);
				pk_len = (double) op_pk_total_size_get (pkptr); 
				pk_time = (double) pk_len / tx_data_rate;
				if(pk_time < time_left_in_slot1)
					{
					pkptr=op_subq_pk_remove(2, OPC_QPOS_HEAD);
					FLAG=1;
					used_time=pk_time;
					}
				}
			}

		else
			{
			is_my_slot=0;
			time_left_in_slot1=0.0;
			}
	
		if (FLAG==1)
			{
			/* full pk */
			op_pk_nfd_set(pkptr,"SEND",my_address);
			op_pk_nfd_set(pkptr,"FL",(int)op_pk_total_size_get (pkptr));
			op_pk_nfd_get(pkptr,"TYPE",&type);
			if(type==0x02) op_pk_nfd_set(pkptr,"TTL",TTL);
			/* send it! */
			op_pk_print(pkptr);
			time_left_in_slot1-=used_time;
			printf("time left:%lf\n",time_left_in_slot1);
			op_pk_send (pkptr, TX_OUT_STRM);
			}
		else
			{
			is_my_slot=0;
			time_left_in_slot1=0.0;
			}
		}

	else
		{
		if(op_subq_empty(0)==0)
			{
			pkptr=op_subq_pk_access (0, OPC_QPOS_HEAD);
			pk_len = (double) op_pk_total_size_get (pkptr); 
			pk_time = (double) pk_len / tx_data_rate;
			if(pk_time < time_left_in_slot1)
				{
				pkptr=op_subq_pk_remove(0, OPC_QPOS_HEAD);
				FLAG=1;
				used_time=pk_time;
				}
			}
		else if(op_subq_empty(2)==0 && FLAG==0)
			{
			pkptr=op_subq_pk_access (2, OPC_QPOS_HEAD);
			pk_len = (double) op_pk_total_size_get (pkptr); 
			pk_time = (double) pk_len / tx_data_rate;
			if(pk_time < time_left_in_slot1)
				{
				pkptr=op_subq_pk_remove(2, OPC_QPOS_HEAD);
				FLAG=1;
				used_time=pk_time;
				}
			}
		else if(op_subq_empty(1)==0 && FLAG==0)
			{
			pkptr=op_subq_pk_access (1, OPC_QPOS_HEAD);
			pk_len = (double) op_pk_total_size_get (pkptr); 
			pk_time = (double) pk_len / tx_data_rate;
			if(pk_time < time_left_in_slot1)
				{
				pkptr=op_subq_pk_remove(1, OPC_QPOS_HEAD);
				FLAG=1;
				used_time=pk_time;
				}
			}
		else
			{
			is_my_slot=0;
			time_left_in_slot1=0.0;
			}
	
		if (FLAG==1)
			{
			/* full pk */
			op_pk_nfd_set(pkptr,"SEND",my_address);
			op_pk_nfd_set(pkptr,"FL",(int)op_pk_total_size_get (pkptr));
			op_pk_nfd_get(pkptr,"TYPE",&type);
			if(type==0x02) op_pk_nfd_set(pkptr,"TTL",TTL);
			/* send it! */
			if(type==0x01) op_pk_print(pkptr);
			time_left_in_slot1-=used_time;
			printf("time left:%lf\n",time_left_in_slot1);
			op_pk_send (pkptr, TX_OUT_STRM);
			}
		else
			{
			is_my_slot=0;
			time_left_in_slot1=0.0;
			}
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
	void tdma3_ (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_tdma3__init (int * init_block_ptr);
	void _op_tdma3__diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_tdma3__terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_tdma3__alloc (VosT_Obtype, int);
	void _op_tdma3__svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
tdma3_ (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (tdma3_ ());

		{
		/* Temporary Variables */
		Packet*		pkptr;
		Objid 		tx_id, comp_id, tx_ch_id; 
		int			used_slots;
		int			current_offset;
		int			next_offset;
		double		current_time;
		//double		time_left_in_slot;
		//double		pk_len;
		//double		pk_time;
		double		my_next_slot_time;
		int			current_intrpt_type;
		int 		i,j;
		
		/* End of Temporary Variables */


		FSM_ENTER ("tdma3_")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "tdma3_ [init enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [init enter execs]", state0_enter_exec)
				{
				Distribution* random_integer_dist;
				int rn;
				
				/*important setting*/
				op_ima_sim_attr_get (OPC_IMA_DOUBLE, "Slot Length", &slot_length);
				op_ima_sim_attr_get (OPC_IMA_INTEGER, "DATA Frame Num", &data_frame_num);
				op_ima_sim_attr_get (OPC_IMA_INTEGER, "interactive_id", &interactive_id);
				op_ima_sim_attr_get (OPC_IMA_INTEGER, "QW SLOT NUM", &num_slots);
				
				node_num=16;//test
				
				/*initial begin*/
				my_id = op_id_self();
				my_node_id = op_topo_parent (my_id);
				op_ima_obj_attr_get(my_node_id,"Address",&my_address);
				round_num=node_num/8;///
				round_num_now=0;
				if(my_address==interactive_id) my_clock_level=0; else my_clock_level=7;
				if(my_address==interactive_id) my_node_state=7; else my_node_state=1;
				nei_count=0;
				sy_base_id=-1;
				nei_num_last=0;
				is_my_slot=0;
				data_slot_num_all=0;
				NO_NEI=0;
				Longitude=0;
				Latitude=0;
				Height=0;
				TTL=-1;
				TTL2=-1;
				control_num=0;
				control_continue=0;
				link_maintain_TTL=-1;
				is_send=0xFF;
				data_next=0xFF;
				WAIT=0;
				time_left_in_slot1=0.0;
				FLAG1=0;//
				//n=0;//
				sync_state=0;
				slot_start=0;//
				if(my_address==interactive_id)
					{
					sync_state=1;
					FLAG1=1;
					}
				
				/*for(i=0;i<4;i++)
					{
					for(j=0;j<8;j++)
						{
						array[i][j]=30.0;
						}
					}*/
				
				
				for(i=0;i<24;i++)
					{
					for(j=0;j<7;j++)
						{
						a_frame_record[i][j]=0xFF;
						a_frame_record_last[i][j]=0xFF;
						}
					}
				for(i=0;i<24;i++) my_two_nei[i]=0;
				for(i=0;i<24;i++) time[i]=0;
				for(i=0;i<240;i++) DATA_frame[i]=0;
				for(i=0;i<240;i++) last_data_frame[i]=0;
				for(i=0;i<240;i++) new_data_frame[i]=0;
				for(i=0;i<5;i++) remain_num[i]=-1;
				
				/* Determine the data rate for the transmitter */
				tx_id =  op_topo_assoc (my_id, OPC_TOPO_ASSOC_OUT, OPC_OBJTYPE_RATX, 0); 
				comp_id = op_topo_child (tx_id, OPC_OBJTYPE_COMP, 0);
				tx_ch_id = op_topo_child (comp_id, OPC_OBJTYPE_RATXCH, 0);
				op_ima_obj_attr_get (tx_ch_id, "data rate", &tx_data_rate); 
				
				
				/* Schedule interupt to complete initialization in the exit execs */
				printf("%%%%%%%%%%% %d tdma init is over%%%%%%%%%%%%%\n",my_address);
				if(my_address==interactive_id)//交互节点侦听1帧长
					{
					op_intrpt_schedule_self (op_sim_time()+num_slots*slot_length,11);
					}
				else//非交互节点随机侦听5-6帧
					{
					random_integer_dist=op_dist_load("uniform_int",5*num_slots,6*num_slots);
					rn=op_dist_outcome(random_integer_dist);
					printf("start_time:%lf\n",rn*slot_length);
					op_intrpt_schedule_self (op_sim_time()+rn*slot_length,11);
					}
				
				
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "tdma3_ [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (7, state7_enter_exec, ;, "default", "", "init", "wait", "tr_95", "tdma3_ [init -> wait : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "tdma3_ [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"tdma3_")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "tdma3_ [idle exit execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [idle exit execs]", state1_exit_exec)
				{
				current_intrpt_type = op_intrpt_type ();
				}
				FSM_PROFILE_SECTION_OUT (state1_exit_exec)


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("tdma3_ [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (FROM_RX)
			FSM_TEST_COND (FROM_SRC)
			FSM_TEST_COND (SLOT )
			FSM_TEST_COND (RE_SEND)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "FROM_RX", "", "idle", "fr_rx", "tr_13", "tdma3_ [idle -> fr_rx : FROM_RX / ]")
				FSM_CASE_TRANSIT (1, 3, state3_enter_exec, ;, "FROM_SRC", "", "idle", "fr_src", "tr_15", "tdma3_ [idle -> fr_src : FROM_SRC / ]")
				FSM_CASE_TRANSIT (2, 4, state4_enter_exec, ;, "SLOT ", "", "idle", "tx", "tr_19", "tdma3_ [idle -> tx : SLOT  / ]")
				FSM_CASE_TRANSIT (3, 6, state6_enter_exec, ;, "RE_SEND", "", "idle", "re_send", "tr_93", "tdma3_ [idle -> re_send : RE_SEND / ]")
				FSM_CASE_TRANSIT (4, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_10", "tdma3_ [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (fr_rx) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "fr_rx", state2_enter_exec, "tdma3_ [fr_rx enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [fr_rx enter execs]", state2_enter_exec)
				{
				int nei_CL;
				double error;
				double temp_time;
				
				pkptr =  op_pk_get (RX_IN_STRM);
				op_pk_nfd_get (pkptr, "TYPE", &type);
				if(type==0x00)//QWZ
					{
					op_pk_nfd_get (pkptr, "Clock_level", &nei_CL);
					QWZ_rcv(pkptr);
				    //计算一下已经经过的时隙
				    current_time = op_sim_time();
					used_slots = (int) floor (current_time / slot_length);
					printf("used_slots%d\n",used_slots);
					printf("my_clock_level:%d  nei_cl:%d\n",my_clock_level,nei_CL);
					//父节点收到包后测量误差
					if(my_clock_level< nei_CL)
						{
						//error=(op_sim_time() -used_slots* slot_length)*1e8;
						if(op_sim_time()-slot_start>0.09) slot_start=slot_start+slot_length;
						
						error=(op_sim_time() - slot_start)<0? -(op_sim_time() - slot_start)*1e8 :(op_sim_time() - slot_start)*1e8;
						printf("error%lf %lf %lf",error,op_sim_time() - slot_start,abs(op_sim_time() - slot_start));
						a_frame_record[nei_count-1][6]=(int)error;
						printf("pk error:%d\n",a_frame_record[nei_count-1][6]);
					    }
					
					
					if(my_clock_level> nei_CL && sync_state==1 && retr!=-1)	
						{     //精同步
					    error=(double)(retr/1e8);
						printf("%lf   %lf   %lf\n",op_sim_time(),slot_start,op_sim_time()-slot_start);
						if(op_sim_time()-slot_start>0.09) slot_start=slot_start+slot_length;
						printf("%lf   %lf   %lf\n",op_sim_time(),slot_start,op_sim_time()-slot_start);
						
						temp_time=op_sim_time()-slot_start;
						/*if((error-temp_time)/(1e8)>0.02)
							{
							op_intrpt_schedule_self(op_sim_time()+slot_length-2*(error-temp_time)/(1e8),0);
							op_ev_cancel(evh2);
							printf("\nzhong%lf,my_address%d,op_sim_time:%lf,slot_start:%lf\n",op_sim_time()+slot_length-error/(2*1e8),my_address,op_sim_time(),slot_start);
							}*/
						printf("当前error:%lf\n",error);
						if((error-temp_time)/2>0.001)
							{
							op_intrpt_schedule_self(op_sim_time()+slot_length-(error-temp_time)/2,0);
							op_ev_cancel(evh2);
							printf("\nzhong%lf,my_address%d,op_sim_time:%lf,slot_start:%lf\n",op_sim_time()+slot_length-(error-temp_time)/2,my_address,op_sim_time(),slot_start);
							}
						
						
				        /*if((error/1e8)-(op_sim_time()-slot_start)>error/(2*1e8))
					      {
						  op_intrpt_schedule_self(op_sim_time()+slot_length-error/(2*1e8),0);
					      op_ev_cancel(evh2);
						  printf("\nzhong%lf,my_address%d,op_sim_time:%lf,slot_start:%lf\n",op_sim_time()+slot_length-error/(2*1e8),my_address,op_sim_time(),slot_start);
						  }*/
						
						/*if((error/1e8)-(op_sim_time()-slot_start)<error/(8*1e8))
					      {op_intrpt_schedule_self(op_sim_time()+slot_length-error/1e8,0);
					      op_ev_cancel(evh2);
						  printf("\nzhong%lf,my_address%d,op_sim_time:%lf,slot_start:%lf\n",op_sim_time()+slot_length-error/(1e8),my_address,op_sim_time(),slot_start);
					      }*/
						
						}
					}
				
				else if(type==0x01)//向交互节点发送的上报帧
					{
					Link_report_mac_rx(pkptr);
					}
				
				else if(type==0x02)
					{
					Data_control_mac_rx(pkptr);
					}
				
				else if(type==0x20)
					{
					Appoint_mac_rcv(pkptr);
					}
				
				else if(type==0x21)
					{
					ACK_rcv(pkptr);
					}
							
				
				else if(type==0x81)
					{
					DATA_rcv(pkptr);
					}
				
				else op_pk_destroy(pkptr);
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (fr_rx) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "fr_rx", "tdma3_ [fr_rx exit execs]")


			/** state (fr_rx) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "fr_rx", "idle", "tr_14", "tdma3_ [fr_rx -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (fr_src) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "fr_src", state3_enter_exec, "tdma3_ [fr_src enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [fr_src enter execs]", state3_enter_exec)
				{
				pkptr =  op_pk_get (SRC_IN_STRM);
				op_pk_nfd_get (pkptr, "TYPE", &type);
				
				if(type==0x01)//向交互节点发送的上报帧
					{
					Link_report_frnet(pkptr);
					}
				
				if(type==0x20)//交互节点第一次收到
					{
					Appoint_frnet(pkptr);
					}
				
				if(type==0x02)
					{
					Data_control_frnet(pkptr);
					}
				
				if(type==0x81)//DATA
					{
					Data_frnet(pkptr);
					}
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (fr_src) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "fr_src", "tdma3_ [fr_src exit execs]")


			/** state (fr_src) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "fr_src", "idle", "tr_80", "tdma3_ [fr_src -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (tx) enter executives **/
			FSM_STATE_ENTER_FORCED (4, "tx", state4_enter_exec, "tdma3_ [tx enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [tx enter execs]", state4_enter_exec)
				{
				int min;
				int rn;
				int base_id;
				int base_net_id;
				int NET_CHANGE_FLAG;
				double x,y,z;
				Distribution* random_integer_dist;
				
				slot_start=op_sim_time();
				sync_state=1;
				//时隙计算
				current_time = op_sim_time();
				printf("%d slot time:%lf\n",my_address,current_time);
				used_slots = (int) floor ((current_time / slot_length) + EPSILON);
				current_offset = used_slots % num_slots;
				if(my_node_state==3) 
					{
					data_slot_now=(data_slot_now+1)%data_slot_num_all;
					if(data_slot_now==0 && my_address==0) printf("new frame begin! time:%lf\n",op_sim_time());
					}
				
				//计时器响应
				if(TTL!=-1 && current_offset==0) TTL=TTL-1;
				if(TTL2!=-1 && current_offset==0) TTL2=TTL2-1;
				if(link_maintain_TTL!=-1 && current_offset==0) link_maintain_TTL=link_maintain_TTL-1;
				if(TTL==0)//新数传控制生效
					{
					TTL=-1;
					printf("%d:TTL is 0,time:%lf\n",my_address,op_sim_time());
					for(i=0;i<240;i++) DATA_frame[i]=0;
					//开始数传阶段；
					if(my_node_state==2 || my_node_state==7) my_node_state=3;
					if(control_continue!=0)////时长限制
						{
						printf("定点传输开始！\n");
						//保存之前时隙表
						for(i=0;i<240;i++) last_data_frame[i]=DATA_frame[i];
						//构建新时隙表
						for(i=0;i<240;i++)
							DATA_frame[i]=new_data_frame[i];
						//continue倒计时
						TTL2=control_continue*data_slot_num_all;
						}
					else
						{
						data_slot_num_all=data_frame_num*data_slot_num+num_slots;
						for(j=0;j<data_frame_num;j++)
							DATA_frame[my_data_slot+data_slot_num*j]=1;
						DATA_frame[data_slot_num*data_frame_num+my_offset]=2;//DATA_frame[],长度：data_slot_num_all
						
						data_slot_now=0;
						}
					}
				if(TTL2==0)//定点传输结束
					{
					printf("定点传输结束！！！\n");
					for(i=0;i<240;i++) DATA_frame[i]=last_data_frame[i];
					data_slot_now=0;
					op_intrpt_schedule_remote(op_sim_time(),REQ_OVER_CODE,my_id+12);
					is_send=0xFF;
					data_next=0xFF;
					}
				if(link_maintain_TTL==0)
					{
					op_subq_pk_insert (0, link_maintain_pk, OPC_QPOS_TAIL);
					link_maintain_TTL=-1;
					op_pk_destroy(link_maintain_pk);
					}
				
				//预约下一个时隙
				evh2=op_intrpt_schedule_self (slot_start+0.1, 0);
				//location get
				op_ima_obj_pos_get(my_node_id,&Latitude,&Longitude,&Height,&x,&y,&z);
				
				//防止冲突
				if(time[current_offset]==1 && my_node_state!=3 && NO_NEI==3)
					{
					time[my_offset]=0;
					random_integer_dist=op_dist_load("uniform_int",0,num_slots);
					rn=op_dist_outcome(random_integer_dist);
					my_offset=(my_offset+rn)%num_slots;
					time[my_offset]=1;
					printf("NO NEI!%d\n",my_address);
					NO_NEI=0;
					}
				
				//自身QW时隙，生成并发送QWZ
				if(time[current_offset]==1 && my_node_state!=3) 
					{
					op_intrpt_schedule_remote(op_sim_time(),ROUTE_CHANGE,my_id+12);
					if(nei_count==0) NO_NEI++;
					
					//检查net_id
					NET_CHANGE_FLAG=0;
					base_net_id=my_net_id;
					base_id=my_address;
					for(i=0;i<nei_count;i++)
						{
						if(a_frame_record[i][3]!=base_net_id && base_net_id!=interactive_id)
							{
							if(a_frame_record[i][3]==interactive_id || (base_net_id>a_frame_record[i][3]))
								{
								base_net_id=a_frame_record[i][3];
								base_id=i;
								NET_CHANGE_FLAG=1;
								}
							}
						}
					if(NET_CHANGE_FLAG)
						{
						for(i=0;i<nei_count;i++)
							if(a_frame_record[i][3]==base_net_id && a_frame_record[i][2]<a_frame_record[base_id][2]) base_id=i;
						my_net_id=base_net_id;
						my_node_state=1;
						my_clock_level=a_frame_record[base_id][2]+1;
						time[my_offset]=0;
						my_offset=my_address-a_frame_record[base_id][0]+a_frame_record[base_id][1];
						if(my_offset<0) my_offset+=num_slots;
						my_offset=my_offset%num_slots;
						time[my_offset]=1;
						}
					//不换网时隙更新
					else
						{
						min=8;
						for(i=0;i<nei_count;i++)
							if(a_frame_record[i][3]==my_net_id && a_frame_record[i][2]<min)
								{
								min=a_frame_record[i][2];
								base_id=i;
								}
						if(min<my_clock_level)
							{
							my_clock_level=a_frame_record[base_id][2]+1;
							time[my_offset]=0;
							my_offset=my_address-a_frame_record[base_id][0]+a_frame_record[base_id][1];
							if(my_offset<0) my_offset+=num_slots;
							my_offset=my_offset%num_slots;
							time[my_offset]=1;
							}
						}
					
					
					is_my_slot=1;
					time_left_in_slot1=0.1-0.012-0.0312;//QWZ(0.0312)
					//建包
					if(my_node_state!=3) QWZ_create();
					//重置部分消息
					for(i=0;i<24;i++)
						{
						for(j=0;j<5;j++)
							{
							a_frame_record_last[i][j]=a_frame_record[i][j];
							a_frame_record[i][j]=0xFF;
							}
						}
					nei_num_last=nei_count;
					nei_count=0;
					for(i=0;i<24;i++)
						if(my_two_nei[i]!=0) my_two_nei[i]--;
					
					}
				//数传阶段时隙
				else if(my_node_state==3)
					{
					if(DATA_frame[data_slot_now]==0) is_my_slot=0;
					else
						{
						time_left_in_slot1=0.1-0.012;
						is_my_slot=1;
						if(DATA_frame[data_slot_now]==2)
							{
							printf("QW,%d,time:%lf\n",my_address,op_sim_time());
							op_intrpt_schedule_remote(op_sim_time(),ROUTE_CHANGE,my_id+12);
							QWZ_create();
							time_left_in_slot1=0.1-0.012-0.0312;//0.0312(QWZ)
							//重置部分消息
							for(i=0;i<24;i++)
								{
								for(j=0;j<5;j++)
									{
									a_frame_record_last[i][j]=a_frame_record[i][j];
									a_frame_record[i][j]=0xFF;
									}
								}
							nei_num_last=nei_count;
							nei_count=0;
							for(i=0;i<24;i++)
								if(my_two_nei[i]!=0) my_two_nei[i]--;
							}
						else printf("shuchuan,%d,time:%lf\n",my_address,op_sim_time());
						}
					}
				else
					{
					is_my_slot=0;
					}
				
				//周期上报
				if(used_slots%600==0 && used_slots>0 && TTL2==-1)
					{
					Link_report_create();
					}
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** state (tx) exit executives **/
			FSM_STATE_EXIT_FORCED (4, "tx", "tdma3_ [tx exit execs]")


			/** state (tx) transition processing **/
			FSM_PROFILE_SECTION_IN ("tdma3_ [tx trans conditions]", state4_trans_conds)
			FSM_INIT_COND ((!is_my_slot) || (NO_DATA && (link_maintain_TTL ==-1) ))
			FSM_TEST_COND (is_my_slot && (!NO_DATA || link_maintain_TTL!=-1))
			FSM_TEST_LOGIC ("tx")
			FSM_PROFILE_SECTION_OUT (state4_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, ;, "(!is_my_slot) || (NO_DATA && (link_maintain_TTL ==-1) )", "", "tx", "idle", "tr_18", "tdma3_ [tx -> idle : (!is_my_slot) || (NO_DATA && (link_maintain_TTL ==-1) ) / ]")
				FSM_CASE_TRANSIT (1, 5, state5_enter_exec, ;, "is_my_slot && (!NO_DATA || link_maintain_TTL!=-1)", "", "tx", "tx_queue", "tr_61", "tdma3_ [tx -> tx_queue : is_my_slot && (!NO_DATA || link_maintain_TTL!=-1) / ]")
				}
				/*---------------------------------------------------------*/



			/** state (tx_queue) enter executives **/
			FSM_STATE_ENTER_FORCED (5, "tx_queue", state5_enter_exec, "tdma3_ [tx_queue enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [tx_queue enter execs]", state5_enter_exec)
				{
				Dequeue();
				}
				FSM_PROFILE_SECTION_OUT (state5_enter_exec)

			/** state (tx_queue) exit executives **/
			FSM_STATE_EXIT_FORCED (5, "tx_queue", "tdma3_ [tx_queue exit execs]")


			/** state (tx_queue) transition processing **/
			FSM_PROFILE_SECTION_IN ("tdma3_ [tx_queue trans conditions]", state5_trans_conds)
			FSM_INIT_COND (!is_my_slot || (NO_DATA &&(link_maintain_TTL ==-1)))
			FSM_TEST_COND (is_my_slot && (!NO_DATA || link_maintain_TTL!=-1))
			FSM_TEST_LOGIC ("tx_queue")
			FSM_PROFILE_SECTION_OUT (state5_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, ;, "!is_my_slot || (NO_DATA &&(link_maintain_TTL ==-1))", "", "tx_queue", "idle", "tr_62", "tdma3_ [tx_queue -> idle : !is_my_slot || (NO_DATA &&(link_maintain_TTL ==-1)) / ]")
				FSM_CASE_TRANSIT (1, 5, state5_enter_exec, ;, "is_my_slot && (!NO_DATA || link_maintain_TTL!=-1)", "", "tx_queue", "tx_queue", "tr_88", "tdma3_ [tx_queue -> tx_queue : is_my_slot && (!NO_DATA || link_maintain_TTL!=-1) / ]")
				}
				/*---------------------------------------------------------*/



			/** state (re_send) enter executives **/
			FSM_STATE_ENTER_FORCED (6, "re_send", state6_enter_exec, "tdma3_ [re_send enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [re_send enter execs]", state6_enter_exec)
				{
				Re_send();
				}
				FSM_PROFILE_SECTION_OUT (state6_enter_exec)

			/** state (re_send) exit executives **/
			FSM_STATE_EXIT_FORCED (6, "re_send", "tdma3_ [re_send exit execs]")


			/** state (re_send) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "re_send", "idle", "tr_94", "tdma3_ [re_send -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (wait) enter executives **/
			FSM_STATE_ENTER_UNFORCED (7, "wait", state7_enter_exec, "tdma3_ [wait enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (15,"tdma3_")


			/** state (wait) exit executives **/
			FSM_STATE_EXIT_UNFORCED (7, "wait", "tdma3_ [wait exit execs]")


			/** state (wait) transition processing **/
			FSM_PROFILE_SECTION_IN ("tdma3_ [wait trans conditions]", state7_trans_conds)
			FSM_INIT_COND (RX_STRM)
			FSM_TEST_COND (NET_BUILD)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("wait")
			FSM_PROFILE_SECTION_OUT (state7_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 8, state8_enter_exec, ;, "RX_STRM", "", "wait", "RX", "tr_96", "tdma3_ [wait -> RX : RX_STRM / ]")
				FSM_CASE_TRANSIT (1, 9, state9_enter_exec, ;, "NET_BUILD", "", "wait", "net_build", "tr_99", "tdma3_ [wait -> net_build : NET_BUILD / ]")
				FSM_CASE_TRANSIT (2, 7, state7_enter_exec, ;, "default", "", "wait", "wait", "tr_98", "tdma3_ [wait -> wait : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (RX) enter executives **/
			FSM_STATE_ENTER_FORCED (8, "RX", state8_enter_exec, "tdma3_ [RX enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [RX enter execs]", state8_enter_exec)
				{
				pkptr=op_pk_get(RX_IN_STRM);
				op_pk_nfd_get(pkptr,"TYPE",&type);
				if(type==0)//QWZ
					{
					QWZ_rcv(pkptr);
					WAIT=1;
					}
				else op_pk_destroy(pkptr);
				}
				FSM_PROFILE_SECTION_OUT (state8_enter_exec)

			/** state (RX) exit executives **/
			FSM_STATE_EXIT_FORCED (8, "RX", "tdma3_ [RX exit execs]")


			/** state (RX) transition processing **/
			FSM_TRANSIT_FORCE (7, state7_enter_exec, ;, "default", "", "RX", "wait", "tr_97", "tdma3_ [RX -> wait : default / ]")
				/*---------------------------------------------------------*/



			/** state (net_build) enter executives **/
			FSM_STATE_ENTER_FORCED (9, "net_build", state9_enter_exec, "tdma3_ [net_build enter execs]")
				FSM_PROFILE_SECTION_IN ("tdma3_ [net_build enter execs]", state9_enter_exec)
				{
				int base_num;
				
				//1.建网
				if(WAIT==0)
					{
					Net_build();
					}
				//2.待入网
				else
					{
					base_num=Net_in();
					}
				//自身时隙中断
				if(WAIT==0 || my_address==interactive_id)
					{
					current_time=op_sim_time();
					used_slots=(int)floor((current_time/slot_length)+EPSILON);
					current_offset=used_slots%num_slots;
					next_offset=my_offset-current_offset;
					if(next_offset<=0)
						next_offset+=num_slots;
					my_next_slot_time=(double)(used_slots+next_offset)*slot_length;
					op_intrpt_schedule_self(my_next_slot_time,0);
					}
				else//粗同步
					{
					sy_base_id=a_frame_record[base_num][0];
					if(my_offset>a_frame_record[base_num][1]){
					//printf("%d,%d,,,,,\n",my_offset);
				    op_intrpt_schedule_self(rcv_time[base_num]+(my_offset-a_frame_record[base_num][1])*slot_length+slot_length*num_slots, 0);
					//sync_state=1;
					printf("1yuyue time:%lf base_num:%d\n",rcv_time[base_num]+(my_offset-a_frame_record[base_num][1])*slot_length+slot_length*num_slots,sy_base_id);
					}
					if(my_offset<a_frame_record[base_num][1]){
					op_intrpt_schedule_self(rcv_time[base_num]-(a_frame_record[base_num][1]-my_offset)*slot_length+slot_length*num_slots*2, 0);
					//sync_state=1;
					printf("yuyue time:%lf base_num:%d\n",rcv_time[base_num]-(a_frame_record[base_num][1]-my_offset)*slot_length+slot_length*num_slots*2,sy_base_id);
					}
					}
				}
				FSM_PROFILE_SECTION_OUT (state9_enter_exec)

			/** state (net_build) exit executives **/
			FSM_STATE_EXIT_FORCED (9, "net_build", "tdma3_ [net_build exit execs]")


			/** state (net_build) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "net_build", "idle", "tr_100", "tdma3_ [net_build -> idle : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"tdma3_")
		}
	}




void
_op_tdma3__diag (OP_SIM_CONTEXT_ARG_OPT)
	{
#if defined (OPD_ALLOW_ODB)
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = __LINE__+1;
#endif

	FIN_MT (_op_tdma3__diag ())

	if (1)
		{
		/* Temporary Variables */
		Packet*		pkptr;
		Objid 		tx_id, comp_id, tx_ch_id; 
		int			used_slots;
		int			current_offset;
		int			next_offset;
		double		current_time;
		//double		time_left_in_slot;
		//double		pk_len;
		//double		pk_time;
		double		my_next_slot_time;
		int			current_intrpt_type;
		int 		i,j;
		
		/* End of Temporary Variables */

		/* Diagnostic Block */

		BINIT
		{
		
		}

		/* End of Diagnostic Block */

		}

	FOUT
#endif /* OPD_ALLOW_ODB */
	}




void
_op_tdma3__terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_tdma3__terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_tdma3__svar function. */
#undef my_offset
#undef slot_length
#undef tx_data_rate
#undef my_node_id
#undef my_id
#undef my_address
#undef type
#undef nei_count
#undef my_two_nei
#undef is_my_slot
#undef interactive_id
#undef WAIT
#undef num_slots
#undef my_node_state
#undef my_clock_level
#undef a_frame_record
#undef a_frame_record_last
#undef nei_num_last
#undef node_num
#undef Longitude
#undef Latitude
#undef Height
#undef time
#undef TTL
#undef remain_pk
#undef evh
#undef remain_num
#undef control_num
#undef control_continue
#undef my_data_slot
#undef data_slot_num
#undef data_frame_num
#undef DATA_frame
#undef data_slot_num_all
#undef data_slot_now
#undef link_maintain_TTL
#undef link_maintain_pk
#undef link_maintain_time
#undef last_data_frame
#undef TTL2
#undef new_data_frame
#undef is_send
#undef data_next
#undef my_net_id
#undef NO_NEI
#undef time_left_in_slot1
#undef FLAG1
#undef n
#undef sync_state
#undef rcv_time
#undef evh2
#undef slot_start
#undef sy_base_id
#undef retr
#undef round_num
#undef round_num_now

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_tdma3__init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_tdma3__init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (tdma3_)",
		sizeof (tdma3__state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_tdma3__alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	tdma3__state * ptr;
	FIN_MT (_op_tdma3__alloc (obtype))

	ptr = (tdma3__state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "tdma3_ [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_tdma3__svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	tdma3__state		*prs_ptr;

	FIN_MT (_op_tdma3__svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (tdma3__state *)gen_ptr;

	if (strcmp ("my_offset" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_offset);
		FOUT
		}
	if (strcmp ("slot_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->slot_length);
		FOUT
		}
	if (strcmp ("tx_data_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->tx_data_rate);
		FOUT
		}
	if (strcmp ("my_node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_node_id);
		FOUT
		}
	if (strcmp ("my_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_id);
		FOUT
		}
	if (strcmp ("my_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_address);
		FOUT
		}
	if (strcmp ("type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->type);
		FOUT
		}
	if (strcmp ("nei_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->nei_count);
		FOUT
		}
	if (strcmp ("my_two_nei" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->my_two_nei);
		FOUT
		}
	if (strcmp ("is_my_slot" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->is_my_slot);
		FOUT
		}
	if (strcmp ("interactive_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->interactive_id);
		FOUT
		}
	if (strcmp ("WAIT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->WAIT);
		FOUT
		}
	if (strcmp ("num_slots" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->num_slots);
		FOUT
		}
	if (strcmp ("my_node_state" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_node_state);
		FOUT
		}
	if (strcmp ("my_clock_level" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_clock_level);
		FOUT
		}
	if (strcmp ("a_frame_record" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->a_frame_record);
		FOUT
		}
	if (strcmp ("a_frame_record_last" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->a_frame_record_last);
		FOUT
		}
	if (strcmp ("nei_num_last" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->nei_num_last);
		FOUT
		}
	if (strcmp ("node_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_num);
		FOUT
		}
	if (strcmp ("Longitude" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Longitude);
		FOUT
		}
	if (strcmp ("Latitude" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Latitude);
		FOUT
		}
	if (strcmp ("Height" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Height);
		FOUT
		}
	if (strcmp ("time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->time);
		FOUT
		}
	if (strcmp ("TTL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL);
		FOUT
		}
	if (strcmp ("remain_pk" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->remain_pk);
		FOUT
		}
	if (strcmp ("evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->evh);
		FOUT
		}
	if (strcmp ("remain_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->remain_num);
		FOUT
		}
	if (strcmp ("control_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->control_num);
		FOUT
		}
	if (strcmp ("control_continue" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->control_continue);
		FOUT
		}
	if (strcmp ("my_data_slot" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_data_slot);
		FOUT
		}
	if (strcmp ("data_slot_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_slot_num);
		FOUT
		}
	if (strcmp ("data_frame_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_frame_num);
		FOUT
		}
	if (strcmp ("DATA_frame" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->DATA_frame);
		FOUT
		}
	if (strcmp ("data_slot_num_all" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_slot_num_all);
		FOUT
		}
	if (strcmp ("data_slot_now" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_slot_now);
		FOUT
		}
	if (strcmp ("link_maintain_TTL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->link_maintain_TTL);
		FOUT
		}
	if (strcmp ("link_maintain_pk" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->link_maintain_pk);
		FOUT
		}
	if (strcmp ("link_maintain_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->link_maintain_time);
		FOUT
		}
	if (strcmp ("last_data_frame" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->last_data_frame);
		FOUT
		}
	if (strcmp ("TTL2" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL2);
		FOUT
		}
	if (strcmp ("new_data_frame" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->new_data_frame);
		FOUT
		}
	if (strcmp ("is_send" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->is_send);
		FOUT
		}
	if (strcmp ("data_next" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_next);
		FOUT
		}
	if (strcmp ("my_net_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_net_id);
		FOUT
		}
	if (strcmp ("NO_NEI" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NO_NEI);
		FOUT
		}
	if (strcmp ("time_left_in_slot1" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->time_left_in_slot1);
		FOUT
		}
	if (strcmp ("FLAG1" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->FLAG1);
		FOUT
		}
	if (strcmp ("n" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->n);
		FOUT
		}
	if (strcmp ("sync_state" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->sync_state);
		FOUT
		}
	if (strcmp ("rcv_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->rcv_time);
		FOUT
		}
	if (strcmp ("evh2" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->evh2);
		FOUT
		}
	if (strcmp ("slot_start" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->slot_start);
		FOUT
		}
	if (strcmp ("sy_base_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->sy_base_id);
		FOUT
		}
	if (strcmp ("retr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->retr);
		FOUT
		}
	if (strcmp ("round_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->round_num);
		FOUT
		}
	if (strcmp ("round_num_now" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->round_num_now);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

