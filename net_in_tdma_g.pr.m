MIL_3_Tfile_Hdr_ 145A 140A modeler 9 66CC11E0 675FB2D8 4B ray-laptop 28918 0 0 none none 0 0 none CDF96257 328A 0 0 0 0 0 0 1bcc 1                                                                                                                                                                                                                                                                                                                                                                                              Ф═gЅ      @   D   к  ј  њ  #  &  &  *М  0m  0y  0ѓ  0є            Slot Length                seconds       ?­                                                         ЦZ             	   begsim intrpt         
          
   doc file            	nd_module      endsim intrpt                       failure intrpts            disabled      intrpt interval         н▓IГ%ћ├}          priority                        recovery intrpts            disabled      subqueue                     count          
          
      list   	      
          
      super priority                              Objid	\my_node_id;       Objid	\my_id;       int	\my_address;       
int	\type;       Evhandle	\evh[5];       Packet *	\remain_pk[5];       int	\remain_num[5];       int	\interactive_id;          Packet*	pkptr;   int i;       int current_intrpt_type;   2   #include <math.h>       /* Constant Definitions */   #define RX_IN_STRM		(1)   #define SRC_IN_STRM		(0)   #define TX_OUT_STRM		(1)   #define SINK_OUT_STRM	(0)       7#define EPSILON  		(1e-10)  /* rounding error factor */   #define TDMA_COMPLETE	(-10)   #define FRAME_BEGIN		(2000)   //#define WAIT			(2222)   #define TS				(3333)   #define TIME_OUT_1		(2501)////   #define TIME_OUT_2		(2502)   #define TIME_OUT_3		(2503)   #define TIME_OUT_4		(2504)   #define TIME_OUT_5		(2505)       "/* Transition Condition Macros */    _#define FROM_RX			(current_intrpt_type == OPC_INTRPT_STRM) && (op_intrpt_strm () == RX_IN_STRM)   b#define FROM_SRC 		(current_intrpt_type == OPC_INTRPT_STRM) && (op_intrpt_strm () == SRC_IN_STRM)    5#define TRANSMITTING	(op_stat_local_read (0) == 1.0)    T#define SLOT 			(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == 0)   Y#define MY_SLOT 		(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == 3000)   8#define END  			(current_intrpt_type == OPC_INTRPT_STAT)       X#define NET_IN			(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == 4444)   \#define SEND_NET_IN		(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == 5555)   ^//#define WAIT_BEGIN 		(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == WAIT)   )#define DATA_ENQ 		(!(op_subq_empty (0)))   W#define TIME_END		(current_intrpt_type == OPC_INTRPT_SELF) && (op_intrpt_code () == TS)   a#define RE_SEND_1		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_1)////   a#define RE_SEND_2		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_2)////   a#define RE_SEND_3		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_3)////   a#define RE_SEND_4		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_4)////   a#define RE_SEND_5		(op_intrpt_type() == OPC_INTRPT_SELF) && (op_intrpt_code () == TIME_OUT_5)////   G#define RE_SEND			RE_SEND_1||RE_SEND_2||RE_SEND_3||RE_SEND_4||RE_SEND_5       ,#define	SELF_INTRPT_SCHLD	(intrpt_flag == 1)       /* Global Variables */   int		tdma_pk_sent;   int		tdma_pk_rcvd;   int		tdma_bits_sent;   int		tdma_bits_rcvd;   int		tdma_setup;   int		tdma_id;   int		num_slots;       x   (static void Appoint_mac2_rcv(Packet *pk)   	{   	Packet *pkptr;   
	int send;   	int pk_id;   	   "	FIN(Appoint_mac2_rcv(Packet *pk))    	op_pk_nfd_get(pk,"SEND",&send);   #	op_pk_nfd_get(pk,"Pk_num",&pk_id);   	op_pk_send(pk,SINK_OUT_STRM);	   	pkptr=op_pk_create_fmt("ACK");   (	op_pk_nfd_set(pkptr,"SEND",my_address);   "	op_pk_nfd_set(pkptr,"TYPE",0x21);   "	op_pk_nfd_set(pkptr,"DEST",send);   %	op_pk_nfd_set(pkptr,"PK_NUM",pk_id);   	op_pk_send(pkptr,TX_OUT_STRM);   	FOUT   	}       $static void ACK_mac2_rcv(Packet *pk)   	{   
	int dest;   
	int send;   	int pk_num;   	int FF;   	int i;   	   	FIN(ACK_mac2_rcv(Packet *pk))    	op_pk_nfd_get(pk,"SEND",&dest);    	op_pk_nfd_get(pk,"DEST",&send);   $	op_pk_nfd_get(pk,"PK_NUM",&pk_num);   (	if(my_address!=send) op_pk_destroy(pk);   	else   		{   		FF=0;   		for(i=0;i<5;i++)   			{   			if(pk_num==remain_num[i])   				{   					FF=1;   O				printf("%d  ACK get form %d !!! time:%lf\n",my_address,dest,op_sim_time());   				op_ev_cancel(evh[i]);    				op_pk_destroy(remain_pk[i]);   				remain_num[i]=-1;   				}   			}   		if(FF==0) op_pk_destroy(pk);   		}   	FOUT   	}       static void Re_send_mac2(void)   	{   	Packet *pkptr;   	   	FIN(Re_send_mac2(void))   !	if(op_intrpt_type ()==RE_SEND_1)   		{   =		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());   "		pkptr=op_pk_copy (remain_pk[0]);    		op_pk_send(pkptr,TX_OUT_STRM);   		}   !	if(op_intrpt_type ()==RE_SEND_2)   		{   =		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());   "		pkptr=op_pk_copy (remain_pk[1]);    		op_pk_send(pkptr,TX_OUT_STRM);   		}   !	if(op_intrpt_type ()==RE_SEND_3)   		{   =		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());   "		pkptr=op_pk_copy (remain_pk[2]);    		op_pk_send(pkptr,TX_OUT_STRM);   		}   !	if(op_intrpt_type ()==RE_SEND_4)   		{   =		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());   "		pkptr=op_pk_copy (remain_pk[3]);    		op_pk_send(pkptr,TX_OUT_STRM);   		}   !	if(op_intrpt_type ()==RE_SEND_5)   		{   =		printf("%d  re_send pk at %lf\n",my_address,op_sim_time());   "		pkptr=op_pk_copy (remain_pk[4]);    		op_pk_send(pkptr,TX_OUT_STRM);   		}       	FOUT   	}       /static void Link_maintain_frnet2(Packet *pkptr)   	{   )	FIN(Link_maintain_frnet2(Packet *pkptr))   (	op_pk_nfd_set(pkptr,"SEND",my_address);   A	op_pk_nfd_set(pkptr,"FL",(double) op_pk_total_size_get (pkptr));   	op_pk_send(pkptr,TX_OUT_STRM);   	FOUT   	}       )static void Appoint_frnet2(Packet *pkptr)   	{   	int i;   	   #	FIN(Appoint_frnet2(Packet *pkptr))   	for(i=0;i<5;i++)   		{   		if(remain_num[i]==-1)   				break;   		}   !	remain_pk[i]=op_pk_copy (pkptr);   .	op_pk_nfd_get(pkptr,"Pk_num",&remain_num[i]);   	op_pk_send(pkptr,TX_OUT_STRM);   E	if(i==0) evh[0]=op_intrpt_schedule_self(op_sim_time()+5,TIME_OUT_1);   E	if(i==1) evh[1]=op_intrpt_schedule_self(op_sim_time()+5,TIME_OUT_2);   E	if(i==2) evh[2]=op_intrpt_schedule_self(op_sim_time()+5,TIME_OUT_3);   E	if(i==3) evh[3]=op_intrpt_schedule_self(op_sim_time()+5,TIME_OUT_4);   E	if(i==4) evh[4]=op_intrpt_schedule_self(op_sim_time()+5,TIME_OUT_5);       	FOUT   	}                                       Z   м          
   init   
       
      my_id = op_id_self();   $my_node_id = op_topo_parent (my_id);   6op_ima_obj_attr_get(my_node_id,"Address",&my_address);   "for(i=0;i<5;i++) remain_num[i]=-1;   Iop_ima_sim_attr_get (OPC_IMA_INTEGER, "interactive_id", &interactive_id);   
       
       
       
          
          pr_state      	     м          
   idle   
       
       
       
      (current_intrpt_type = op_intrpt_type ();       
       
           
          pr_state           Z          
   fr_rx   
       J       pkptr =  op_pk_get (RX_IN_STRM);   "op_pk_nfd_get(pkptr,"TYPE",&type);       2if(my_address==interactive_id || my_address==0xFE)   	{   	if(type==0x10)   		{   ,		if(my_address!=0xFE) op_pk_destroy(pkptr);   		else   			{   #			op_pk_send(pkptr,SINK_OUT_STRM);   			}   		}       	if(type==0x20)   		{   		Appoint_mac2_rcv(pkptr);   		}   	if(type==0x21)   		{   		ACK_mac2_rcv(pkptr);   		}       	if(type==0x81)   "		op_pk_send(pkptr,SINK_OUT_STRM);       	if(type==0x11)   "		op_pk_send(pkptr,SINK_OUT_STRM);       	}   else op_pk_destroy(pkptr);   J       
       
       
          
          pr_state        ┬   м          
   fr_src   
       J      !pkptr =  op_pk_get (SRC_IN_STRM);   %op_pk_nfd_get (pkptr, "TYPE", &type);        if(type==0x10)//й╗╗Цй┌хсх─╔¤▒еоА   	{   	Link_maintain_frnet2(pkptr);   	}       if(type==0x20)   	{   	Appoint_frnet2(pkptr);   	}       if(type==0x81)//DATA   	{    	op_pk_send(pkptr,TX_OUT_STRM);	   	}           if(type==0x11)   	{   	op_pk_send(pkptr,TX_OUT_STRM);   	}   J       
       
       
          
          pr_state        J  J          
   re_send   
       J      Re_send_mac2();   J                     
          
          pr_state               	   	   І   Ю      з   К   Д   Ќ   е   Ђ   З   ┼          
   tr_10   
       
   default   
       
       
       
           
       
          
                    pr_transition            	   ќ   ¤      v   ¤   Ы   ¤          
   tr_12   
       
       
       
       
       
           
       
          
                    pr_transition         	        Ю      Ч   ╗   Ч   o          
   tr_13   
       
   FROM_RX   
       
       
       
           
       
          
                    pr_transition            	  H   ї     "   m  "   й          
   tr_14   
       
       
       
       
       
           
       
          
                    pr_transition      #   	     m   и     %   ┴  Ф   ┴          
   tr_15   
       
   FROM_SRC   
       
       
       
           
       
          
                    pr_transition      '      	  m   п     Е   я  '   я          
   tr_16   
       
       
       
       
       
           
       
          
                    pr_transition      J   	     9          я  Y  :          
   tr_74   
       
   RE_SEND   
                     
           
                                     pr_transition      K      	  (       ?  ;     Я          
   tr_75   
                                   
           
                                     pr_transition         L          Load (bits)          (Number of bits broadcast by this node.          TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   Load (bits/sec)          'Number of bits per second broadcast by    this node.      TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   Load (packets)          $Number of packets broadcast by this    node.      TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   Load (packets/sec)          'Number of packets per second broadcast    by this node.      TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   Traffic Received (bits)          'Number of bits received by this node.     TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   Traffic Received (bits/sec)          &Number of bits per second received by    this node.      TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   Traffic Received (packets)          #Number of packets received by this    node.    TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   Traffic Received (packets/sec)          &Number of packets per second received    by this node.       TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}      TDMA Load (bits)          &Total number of bits broadcast by all    tdma capable nodes.     TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   TDMA Load (bits/sec)           Total number of bits per second    &broadcast by all tdma capable nodes.     TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   TDMA Load (packets)          #Total number of packets per second    %broadcast by all tdma capable nodes.    TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   TDMA Load (packets/sec)          #Total number of packets per second    %broadcast by all tdma capable nodes.    TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   TDMA Traffic Received (bits)           Total number of bits per second    $received by all tdma capable nodes.    TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}    TDMA Traffic Received (bits/sec)           Total number of bits per second    $received by all tdma capable nodes.    TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}   TDMA Traffic Received (packets)          (Total number of packets received by all    tdma capable nodes.    TDMA   bucket/default total/sum   linear        н▓IГ%ћ├}   #TDMA Traffic Received (packets/sec)          #Total number of packets per second    $received by all tdma capable nodes.    TDMA   bucket/default total/sum_time   linear        н▓IГ%ћ├}                           