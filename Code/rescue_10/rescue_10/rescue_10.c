/**************************************************************************************************************************************
***************************************************************************************************************************************
 * Team ID:     eYRCPlus-SR#935
 * Author List:	Sanjay A C, Vikas H C  
 * Filename:	Rescue_10.c
 * Theme:		Search and Rescue
 * Functions:	dijsktra,init_devices, setp, setPlot, set_adj_Plots, set_adj_node, updatePlot, get_dir, turn, detect_block, plot_scan,
				go, update, goto_n, move, uart0_init, SIGNAL, USART0_TX, send, init_arena,port_config, init_devices.
 * Global Variables: path,				(array of size 26 x 26 to track the traversed path),
					crt_node			(current node pointer),
					nxt_node			(Next node pointer), 
					prv_node			(Previous node pointer),
					crrt_dir,			(4 values indicating the direction of robot),
					n_node,				(An array of size 26 to store the node numbers to be traversed by the robot),
					data,				(Variable to store the incoming bytes from the uart0, value ranges from 0 - 255),
					cost,				(array of size 26 x 26 to store the cost of path for Dijikstra Algorithm),
					flag,				(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
					flag1				(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
					search_complete_flag,(To choose the incoming bytes from uart0 into appropriate variable in the ISR), 
 *						
 *
 **************************************************************************************************************************************
***************************************************************************************************************************************
 */

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "sensor.h"
#include "motion.h"
#include "buzzer.h"
#include "lcd.h"
#include "servo.h"
#include "colour.h"
#define IN 99
#define N 26

int	sp = 0;
int store[100];
volatile int f=0,pos[2];
int path_length,path_ptr;
volatile int flag=0,flag1=0;
int v[26], visit[26][26], path[26][26]={{0}};
int crt_node=15,nxt_node=0, prv_node=0, dst_node=0, itm_count=0;
int crrt_dir=3;
int c_plot=12,d_plot=0;
int n_node[N];
int count=0;
volatile int data,buffer[10],ptr=0;
volatile int survivor_ptr=0;
int search_complete_flag=0;
int block_node[2];
//int store[50]={{0}},sp=0;
volatile int survivor_plot[10], survivor_colour[10], survivor_status[10];
volatile int stop_flag=0;
volatile int node_ok=0,node_check=0;
volatile int ACK=0;
volatile int vikas_flag=0,midpoint_flag=0;

int cost[N][N]=
{
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0}
};

/******************************************************************************************************************************
* Function Name: dijsktra
* Input:		 source(current node), target(Destination node)
* Output:		 n_node(an array storing the shortest path between source and target node) 
* Logic:		 Dijkstra algorithm- It is an algorithm used in finding shortest path between two nodes in a graph or a network.
				 Here, this function provides a shortest obstacle free path between given source and target node.
* Example call:	 dijkstra(23,15)
*******************************************************************************************************************************/

int dijsktra(int source,int target)
{
  int dist[N],prev[N],selected[N]={0},i,m,min,start,d,j;
  int path[N];
  for(i=1;i< N;i++)
    for(j=1;j< N;j++)
      if(cost[i][j] == 0)
         cost[i][j]=IN;

  for(i=1;i< N;i++)
  {
    dist[i] = IN;
    prev[i] = -1;
  }

  start = source;
  selected[start]=1;
  dist[start] = 0;

  while(selected[target] == 0)
  {
    min = IN;
    m = 0;
    for(i=1;i< N;i++)
    {
      d = dist[start] +cost[start][i];

      if(d< dist[i] && selected[i] == 0)
      {
        dist[i] = d;
        prev[i] = start;
      }
      if(min>dist[i] && selected[i]==0)
      {
        min = dist[i];
        m = i;
      }
    }
    start = m;
    selected[start] = 1;
  }
  start = target;
  j = 0;

  while(start != -1)
  {
    path[j++] = start;
    start = prev[start];
  }

    path[j]=0;
    int k=0;
    for(i=0;i<N;i++)
        n_node[i]=0;

    for(i=j-1;i>=0;i--)
        n_node[k++]=path[i];

    return j;
}

struct plot
{
	int st[5];
	int ed[5];
	int pstatus;
	int adj[5];
};

struct node
{
	int adjn[5];
	int blk[5];
	int nstatus;
};

struct survivors
{
	int plot;
	int colour;
	int status;		
};

struct node n[26];
struct plot b[17];
struct survivors s[10];

/******************************************************************************************************************************
* Function Name: init_survivors
* Input:		 none
* Output:		 none 
* Logic:		 This function is used to initialize the status of the plots (type of survivor found) after receiving 
				 information from the Search robot.
* Example call:	 init_survivors()
*******************************************************************************************************************************/

void init_survivors(void)
{
	for (int i=0;i<10;i++)
	{
		survivor_status[i] = 2;
	}
}

/******************************************************************************************************************************
* Function Name: setp
* Input:		 start(plot number + column number), id(plot number)
* Output:		 none 
* Logic:		 This function is used to initialize all four paths in a plot. 
* Example Call:  setp(i,i)
*******************************************************************************************************************************/

void setp(int start,int id)		// To initialize all 4 paths in a plot
{
	b[id].st[1]=start;
	b[id].ed[1]=start+1;
	b[id].st[2]=start+5;
	b[id].ed[2]=start+6;
	b[id].st[3]=start;
	b[id].ed[3]=start+5;
	b[id].st[4]=start+1;
	b[id].ed[4]=start+6;
	b[id].adj[1]=start;
	b[id].adj[2]=start+1;
	b[id].adj[3]=start+6;
	b[id].adj[4]=start+5;
}

/******************************************************************************************************************************
* Function Name: setPlot
* Input:		 none
* Output:		 none 
* Logic:		 This function is used to initialize all plots with their respective paths
* Example call:	 setPlot()
*******************************************************************************************************************************/

void setPlot(void)				// To initalize all plots with their respective paths
{
    int i;
	for(i=1;i<=16;i++)
	{
		if(i<=4)
		{
			setp(i,i);
		}
		else if(i<=8)
		{
			setp(i+1,i);
		}
		else if(i<=12)
		{
			setp(i+2,i);
		}
		else if(i<=16)
		{
			setp(i+3,i);
		}
	}
}

/******************************************************************************************************************************
* Function Name: set_adj_Plots
* Input:		 none
* Output:		 none 
* Logic:		 This function is used to initialize adjacent plots for each node.
* Example call:	 set_adj_Plots()
*******************************************************************************************************************************/

void set_adj_Plots(void)		// To initialize adjacent plots of each node
{
	int i;
	for(i=1;i<=25;i++)
	{
		if(i<=5)
		{
			if(i%5==1)
			{
				n[i].blk[2]=i;
			}
			else if(i%5==0)
			{
				n[i].blk[1]=i-1;
			}
			else
			{
				n[i].blk[1] = i-1;
				n[i].blk[2] = i;
			}
		}
		else if(i<=10)
		{
			if(i%5==1)
			{
				n[i].blk[2]=i-1;
				n[i].blk[4]=i-5;
			}
			else if(i%5==0)
			{
				n[i].blk[1]=i-2;
				n[i].blk[3]=i-6;
			}
			else
			{
				n[i].blk[1]=i-2;
				n[i].blk[3]=i-6;
				n[i].blk[2]=i-1;
				n[i].blk[4]=i-5;
			}
		}
		else if(i<=15)
		{
			if(i%5==1)
			{
				n[i].blk[4]=i-6;
				n[i].blk[2]=i-2;
			}
			else if(i%5==0)
			{
				n[i].blk[3]=i-7;
				n[i].blk[1]=i-3;
			}
			else
			{
				n[i].blk[4]=i-6;
				n[i].blk[2]=i-2;
				n[i].blk[3]=i-7;
				n[i].blk[1]=i-3;
			}
		}
		else if(i<=20)
		{
			if(i%5==1)
			{
				n[i].blk[4]=i-7;
				n[i].blk[2]=i-3;
			}
			else if(i%5==0)
			{
				n[i].blk[3]=i-8;
				n[i].blk[1]=i-4;
			}
			else
			{
				n[i].blk[4]=i-7;
				n[i].blk[2]=i-3;
				n[i].blk[3]=i-8;
				n[i].blk[1]=i-4;
			}
		}
		else if(i<=25)
		{
			if(i%5==1)
			{
				n[i].blk[4]=i-8;
			}
			else if(i%5==0)
			{
				n[i].blk[3]=i-9;
			}
			else
			{
				n[i].blk[4]=i-8;
				n[i].blk[3]=i-9;
			}
		}
	}
}

/******************************************************************************************************************************
* Function Name: set_adj_node
* Input:		 none
* Output:		 none 
* Logic:		 This function is used to initialize adjacent nodes for each node.
* Example call:	 set_adj_node()
*******************************************************************************************************************************/

void set_adj_node(void)			// To initialize adjacent nodes of each node
{
    int i;
	
	for(i=1;i<=25;i++)
	{
		if(i>5)
		{
			n[i].adjn[1]=i-5;
		}
		if(i<=20)
		{
			n[i].adjn[2]=i+5;
		}
		if((i%5 != 1) && ((i-1) > 0))
		{
			n[i].adjn[3]=i-1;
		}
		if((i%5 != 0) && ((i+1) <= 25))
		{
			n[i].adjn[4]=i+1;
		}
		v[i]=0;
	}
}

/******************************************************************************************************************************
* Function Name: get_dir
* Input:		 cnode(current node), nnode(Next node)
* Output:		 direction  
* Logic:		 This function is used to get the direction of the next node from the current node. 
* Example call:	 get_dir(3,4)
*******************************************************************************************************************************/

int get_dir(int cnode,int nnode) // To get the direction of nnode fron cnode
{
	switch(crrt_dir)
	{
		case 1:
		switch(cnode-nnode)
		{
			case 5:return 0;
			break;
			case -5:crrt_dir = 2;return 180;
			break;
			case 1:crrt_dir = 3;return 270;
			break;
			case -1:crrt_dir = 4;return 90;
			break;
			default:return 0;
		}
		break;
		case 2:
		switch(cnode-nnode)
		{
			case 5:crrt_dir = 1;return 180;
			break;
			case -5:return 0;
			break;
			case 1:crrt_dir = 3;return 90;
			break;
			case -1:crrt_dir = 4;return 270;
			break;
			default:return 0;
		}
		break;
		case 3:
		switch(cnode-nnode)
		{
			case 5:crrt_dir = 1;return 90;
			break;
			case -5:crrt_dir = 2;return 270;
			break;
			case 1:return 0;
			break;
			case -1:crrt_dir = 4;return 180;
			break;
			default:return 0;
		}
		break;
		case 4:
		switch(cnode-nnode)
		{
			case 5:crrt_dir = 1;return 270;
			break;
			case -5:crrt_dir = 2;return 90;
			break;
			case 1:crrt_dir = 3;return 180;
			break;
			case -1:return 0;
			break;
			default:return 0;
		}
		break;
		default:
		break;
	}
	
	return 0;
}

/******************************************************************************************************************************
* Function Name: turn
* Input:		 crtnode(current node), nxtnodenode(Next node)
* Output:		 none 
* Logic:		 This function is used to rotate the robot accordingly to reach next node from the current node. 
* Example call:	 turn(15,20)
*******************************************************************************************************************************/

void turn(int crtnode,int nxtnode)		//to turn the bot towards nxtnode from crtnode
{
	int deg = get_dir(crtnode,nxtnode);

	switch(deg)
	{
		case 0:
		break;
		case 90:
		rotate_right(90);
		break;
		case 180:
		rotate_right(180);
		break;
		case 270:
		rotate_left(90);
		break;
	}
}

/******************************************************************************************************************************
* Function Name: detect_block
* Input:		 c_node, n_node_node
* Output:		 status of obstucle in the path between c_node & d_node
* Logic:		 This function is used to detect black debris and update about it in the cost matrix.
* Example call:	 detect_block(7,8)
*******************************************************************************************************************************/

int detect_block(int c_node,int n_node)		// To detect black debris and updating the midpoint(ob)
{
	int retv=0;
	_delay_ms(100);
	int temp = detect_obstacle(80,220);
	
	if(temp)
	{
		cost[c_node][n_node]=IN;
		cost[n_node][c_node]=IN;
		retv=1;
	}
	
	return retv;
}

/******************************************************************************************************************************
* Function Name: plot_scan
* Input:		 crtid, nxtid
* Output:		 none
* Logic:		 This function is used inorder to service the detected survivors 
* Example call:	 plot_scan(1,2,2)
*******************************************************************************************************************************/

int plot_scan(int crtid,int nxtid, int survivor_index)		//To scan the plots for white debris
{
	int l = 0, r = 0;
	int retval = 0;

	switch(crtid - nxtid)
	{
		case 1:
		l=n[crtid].blk[1];
		r=n[crtid].blk[3];
		break;
		
		case -1:
		l=n[crtid].blk[4];	//l=n[crtid].blk[4];
		r=n[crtid].blk[2];	//r=n[crtid].blk[2];
		break;
		
		case 5:
		l=n[crtid].blk[3];
		r=n[crtid].blk[4];
		break;
		
		case -5:
		l=n[crtid].blk[2];
		r=n[crtid].blk[1];
		break;
		
		default: break;
	}
	
	//turn(crtid,nxtid);
	
	if(survivor_status[survivor_index] == 0)
	if(survivor_plot[survivor_index] == l )
	{
		//left_degrees(86);
		
		
		survivor_status[survivor_index]=1;
		
		if (survivor_colour[survivor_index] == 0)
		{
			left_degrees(86);
			buzzer(1);
			rotate_right(85);
		}
		else
		{
			path_mm(60);
			left_degrees(86);
			first_aid_kit_deposit();
			display_colour(3);
			buzzer(1);
			rotate_right(85);
		}
		
		
		retval = 1;
	}
	else if(survivor_plot[survivor_index] == r)
	{
		//right_degrees(86);
		survivor_status[survivor_index]=1;
		
		if (survivor_colour[survivor_index] == 0)
		{
			right_degrees(86);
			buzzer(1);
			rotate_left(85);
		}
		else
		{
			path_mm(60);
			right_degrees(86);
			first_aid_kit_deposit();
			display_colour(3);
			buzzer(1);
			rotate_left(85);
		}
		
		retval = 1;
	}
	
	if(retval==1)
	b[survivor_plot[survivor_index]].pstatus=1;
	
	return retval;
}

void check_collision(int crtnode, int nxtnode, int index)
{
	if (nxtnode == pos[1])
	{
		while(nxtnode == pos[1]);	////while(nxtnode == pos[1]);
		_delay_ms(1);
		//while(nxtnode == pos[1]);	////while(nxtnode == pos[1]);		
	}
	
	if ((pos[0] == nxtnode) && !((pos[0] == nxtnode) && (pos[1] == crtnode)))
	{
		ACK=0;
		USART0_TX(85);
		//if (search_complete_flag==0)
		{
			//_delay_ms(10);
			while(ACK == 0)
			{
				USART0_TX(85);
				_delay_ms(1);
			}
			ACK = 0;
			
		}
		//_delay_ms(500);
		while(midpoint_flag == 0)
		{
			USART0_TX(85);
			if (pos[0] != nxtnode)
			{
				break;
			}
			if((pos[0] == nxtnode) && (pos[1] == crtnode))
			{
				break;
			}
			_delay_ms(10);
		}
		
		_delay_ms(1000);
		
		midpoint_flag = 0;
	}
	else if ((pos[0] == nxtnode))
	{
		_delay_ms(2000);
	}
	
	if((pos[0] == nxtnode) && (pos[1] == crtnode))
	{
		int move_flag=0;
		int t_node = crtnode;
		int v_node;
		
		for (int j=1;j<=4;j++)
		{
			if(n[crtnode].adjn[j] == 0)
			{
				move_flag = 1;
			}
			else if ( n[crtnode].adjn[j] == nxtnode )
			{
				move_flag = 1;
				//continue;
			}
			else if ( n[crtnode].adjn[j] == store[sp])
			{
				move_flag = 1;
				//continue;
			}
			else if (cost[crtnode][n[crtnode].adjn[j]] == IN)
			{
				move_flag = 1;
				//continue;
			}
			else
			{
				turn(crtnode,n[crtnode].adjn[j]);
				move_flag =1;
				int temp = detect_block(crtnode,n[crtnode].adjn[j]);
				
				if(temp == 0)
				{				
					node_check = 0;
					send1(90,n[crtnode].adjn[j]);
					
					while(node_check == 0);
					_delay_ms(200);
					if (node_ok)
					{
						move_flag = 0;
						go1(crtnode,n[crtnode].adjn[j]);
						
						break;				///
					}
					else
					{
						move_flag =1;
					}
				}
			}
		}
		
		if (move_flag == 1)
		{
			go_back();
		}
		
		goto_n(crt_node,t_node,index);
		
		turn(crtnode,nxtnode);
	}
}

void check_collision_midpoint(int crtnode,int nxtnode, int index)
{
	int umpire=0;
	
	if (nxtnode == pos[1])
	{
		back();
		linear_mm(240);
		umpire = 1;
		while(nxtnode != pos[0]);
		_delay_ms(1);
		//_delay_ms(3000);
	}
	
	if ((pos[0] == nxtnode) && !((pos[0] == nxtnode) && (pos[1] == crtnode)))
	{
		if(!umpire)
		{
			back();
			linear_mm(240);
			umpire = 1;
		}
		ACK=0;
		USART0_TX(85);
		//	if (search_complete_flag==0)
		{
			_delay_ms(10);
			while(ACK == 0)
			{
				USART0_TX(85);
				_delay_ms(1);
			}
			ACK = 0;
		}
		
		while(midpoint_flag == 0)
		{
			USART0_TX(85);
			if (pos[0] != nxtnode)
			{
				break;
			}
			if((pos[0] == nxtnode) && (pos[1] == crtnode))
			{
				break;
			}
			_delay_ms(100);
		}
		midpoint_flag = 0;
	}
	
	if((pos[0] == nxtnode) && (pos[1] == crtnode))
	{
		if(!umpire)
		{
			back();
			linear_mm(240);
			umpire = 1;
		}
		
		int move_flag=0;
		
		int t_node = crt_node;
		
		for (int j=1;j<=4;j++)
		{
			if(n[crtnode].adjn[j] == 0)
			{
				move_flag = 1;
			}
			else if ( n[crtnode].adjn[j] == nxtnode )
			{
				move_flag = 1;
				//continue;
			}
			else if ( n[crtnode].adjn[j] == store[sp])
			{
				move_flag = 1;
				//continue;
			}
			else if (cost[crtnode][n[crtnode].adjn[j]] == IN)
			{
				move_flag = 1;
				//continue;
			}
			else
			{
				turn(crtnode,n[crtnode].adjn[j]);
				move_flag =1;
				int temp=detect_block(crtnode,n[crtnode].adjn[j]);
				
				if( temp == 0)
				{
					node_check = 0;
					send1(90,n[crt_node].adjn[j]);
					while(node_check == 0);
					if (node_ok)
					{
						move_flag = 0;
						go1(crtnode,n[crt_node].adjn[j]);
					}
					else
					{
						move_flag =1;
					}
				}
			}
		}
		
		if (move_flag == 1)
		{
			//goto_n(crt_node,prv_node,index);
			go_back();
		}
		
		//while(nxtnode == pos[1] );
		
		//move(crt_node,t_node,index);
		goto_n(crt_node,t_node,index);
		
		turn(crtnode,nxtnode);
	}
	
	if (umpire)
	{
		path_node(1);
		umpire = 0;
	}
}

/******************************************************************************************************************************
* Function Name: go
* Input:		 crtnode,nxtnode,index
* Output:		 status of the survivor scan of specific index
* Logic:		 This function is used to move the robot from current node to next node. 
* Example call:  go(4,5)
*******************************************************************************************************************************/

int go(int crtnode, int nxtnode, int index)	// To move the bot from crtnode to nxtnode
{			
	//lcd_print(2,1,crtnode,2);
	//lcd_print(2,4,nxtnode,2);
	for (int i=0;i<10;i++)
	{
		check_collision(crtnode,nxtnode,index);
	}
	
	
	path_node(1);

	for (int i=0;i<10;i++)
		check_collision_midpoint(crtnode,nxtnode,index);
	
	int retval = plot_scan(crtnode,nxtnode,index);
	
	for (int i=0;i<10;i++)
		check_collision_midpoint(crtnode,nxtnode,index);
	
	if (nxtnode == pos[0])
	{
		_delay_ms(2000);
	}
	
	path_node(1);
	
	update(crtnode,nxtnode);
	
	return retval;
}

/******************************************************************************************************************************
* Function Name: update
* Input:		 an(node1 of current path), bn(node2 of current path)
* Output:		 none 
* Logic:		 This function is used to update the paths (i.e., midpoints) about the presence or absence of the black debris.
* Example call:	 update(16,17)
*******************************************************************************************************************************/

void update(int an,int bn)		// Update the paths (midpoints) for visiting
{
	prv_node=an;
	push(an);
	crt_node=bn;
	path[an][bn]=path[bn][an]=1;
}

void go1(int crtnode, int nxtnode)	// To move the bot from crtnode to nxtnode
{		
	send(50,crtnode,nxtnode);
	
	turn(crtnode,nxtnode);	
	
	path_node(1);
	path_node(1);
	
	crt_node = nxtnode;
	
}

void push(int x)
{
	sp++;
	store[sp]=x;
}

int pop(void)
{
	if(sp>=0)
	{
		int temp = store[sp];
		sp--;
		return temp;	
	}
	else
		return -1;
}

void clear_stack(void)
{
	for(int i=0;i<100;i++)
	{
		store[i]=0;
	}
	sp=0;
}

void go_back(void)
{
	go1(crt_node,pop());
}

/******************************************************************************************************************************
* Function Name: goto_n
* Input:		 c_node,n_node,index
* Output:		 status of the survivor scan of specific index and obstucle information
* Logic:		 This function is used to check for the obstacle. If absent, calls go function. If present, updates the 
				 presence of the block in the cost matrix.
* Example call:  goto_n(4,5)
*******************************************************************************************************************************/

int goto_n(int c_node,int d_node, int index)
{
	ACK=0;
	send(50,c_node,d_node);		// position flag
	_delay_ms(10);
	if (ACK != 1)
	{
		while(ACK !=1)
		{
			send(50,c_node,d_node);
			_delay_ms(1);
		}
	}
	ACK = 0;
				
	turn(c_node,d_node);
	
	lcd_print(1,6,c_node,2);
	lcd_print(1,10,d_node,2);
	int retval=0;
	 	
	if(cost[c_node][d_node]==IN)
	{
		retval=-1;
	}
	else if(path[c_node][d_node]==1)
	{
		count++;
		retval = go(c_node,d_node,index);
	}	
	else
	{
		int abs = detect_block(c_node,d_node);
	
		if(abs)
		{
			send(30,c_node,d_node);
			retval=-1;
		}
		else
		{
			count++;
			retval = go(c_node,d_node,index);
		}
	}
	
	return retval;
}

/******************************************************************************************************************************
* Function Name: move
* Input:		 c_node (current node),d_node (destination node),index (index of survivor to be scanned)
* Output:		 movement of the robot to the required destination node.
* Logic:		 This function is used to move to the adjacent nodes of an unscanned plot.
* Example call:  move(4,5,4)
*******************************************************************************************************************************/

int move(int c_node,int d_node, int index)
{
	int i,a=0,retval=0,j=1;
	int path_length = dijsktra(c_node,d_node),flag=0;
	int t_node,move_flag=0;
	
	for (i=1;i<path_length;i++)
	{
		path_ptr = i;
		a = goto_n(crt_node,n_node[i],index);
				
		if(a == 1)
		{
			return 1;
			//break;
		}			
		else if(a == -1)
		{
			flag=1;
			break;
		}			
	}
	if(flag)
	{
		move(crt_node,d_node,index);
	}

	return retval;
}

int rescue(int index)
{
	int i;
	
	int p = survivor_plot[index];
	lcd_print(2,1,p,2);
	
	int flag=1,flag420=0;

	for(i=1;i<=4;i++)
	{
		if(b[p].adj[i] == crt_node)
		{
			flag=0;
			break;
		}
	}

	for(i=1;i<=4;i++)
	{
		if(b[p].adj[i] != 0)
		{
			dst_node = b[p].adj[i];
	
			if (flag)
			{
				int a = move(crt_node, dst_node,index);		////
				if(a == 1)
				{	//return 1;
					break;
				}					
				if(b[p].pstatus==1)
				//break;
					return 1;
				flag=0;
			}
			else
			{
				int a=abs(dst_node - crt_node);

				if (a==1 || a==5)
				{
					int bp = goto_n(crt_node,dst_node,index);
	
					if(bp == -1)
					{
						continue;
					}
					else if(bp == 1)
					{
						flag420=0;
						return 1;
						//break;
					}
					if(b[p].pstatus==1)
						return 1;
						//break;					
				}
				else
				{
					flag420=1;
					if(b[p].pstatus==1)
						return 1;
						//break;
				}
			}
		}
	}
	if(flag420)
	for(i=1;i<=4;i++)
	{
		if(b[p].adj[i]!=0)
		{
			dst_node = b[p].adj[i];
	
			int a = move(crt_node, dst_node,index);
			
			if(a == 1)
				return 1;
				//break;
				
			if(b[p].pstatus==1)
				return 1;
				//break;
		}
	}
	
	return 0;
}

/******************************************************************************************************************************
* Function Name: uart0_init
* Input:		 none
* Output:		 none
* Logic:		 This function is used to 
* Example call:  uart0_init()
*******************************************************************************************************************************/

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

volatile int survivor_flag=0,survivor_flag1=0,survivor_flag2=0;
volatile int clear_flag=0,clear_flag1=0;
volatile int position_flag=0,position_flag1=0;

/******************************************************************************************************************************
* Function Name: Interrupt service Routine - SIGNAL
* Input:		 SIG_USART0_RECV interrupts
* Output:		 none
* Logic:		 This is Interrupt Service Routine to receive complete interrupt.
* Example call:  
*******************************************************************************************************************************/

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
		
	if (data==30 || data==40 || data==50 || data==60 || data==80 || data==100 || data==140 || data==150 || data==160)
	{
		switch(data)
		{
			case 30:flag = 1;
			survivor_flag = 0;
			//			clear_flag=0;
			break;
			case 40:survivor_flag = 1;
			flag = 0;
			//		clear_flag=0;
			break;
			case 50:survivor_flag2 =1;
			break;
			case 60:search_complete_flag=1;
					USART0_TX(100);
			break;
			case 80:position_flag=1;
			flag=0;
			survivor_flag=0;
			break;
			case 100:	ACK =1;
			break;
			case 140:	node_check=1;
			node_ok=0;
			break;
			case 150:
			node_check=1;
			node_ok=1;
			break;
			case 160:midpoint_flag = 1;
			position_flag=0;
			flag=0;
			survivor_flag=0;
			USART0_TX(100);
			break;
			default: break;
		}
	}
	else
	{
		if(flag)
		{
			block_node[0]=data;
			flag1=1;
			flag=0;
		}
		else if(flag1)
		{
			block_node[1]=data;
			cost[block_node[0]][block_node[1]]=IN;
			cost[block_node[1]][block_node[0]]=IN;
			flag1=0;
			USART0_TX(100);		// sending ACK
		}
		
		if(position_flag)
		{
			pos[0]=data;
			position_flag=0;
			position_flag1=1;
			lcd_print(2,7,pos[0],2);
		}
		else if(position_flag1)
		{
			pos[1]=data;
			position_flag1=0;
			lcd_print(2,10,pos[1],2);
			USART0_TX(100);
		}
		
		if(survivor_flag)
		{
			survivor_plot[survivor_ptr] = data;
			survivor_flag=0;
			survivor_flag1=1;
		}
		else if(survivor_flag1)
		{
			survivor_colour[survivor_ptr] = data;
			survivor_flag1 = 0;
			survivor_status[survivor_ptr] = 0;
			//survivor_ptr = survivor_ptr + 1;
			USART0_TX(100);
		}
		
		if (survivor_flag2)
		{
			survivor_ptr = data;
			survivor_flag2 = 0;
			USART0_TX(100);
		}
	}		
}

/******************************************************************************************************************************
* Function Name: USART0_TX
* Input:		 data
* Output:		 none
* Logic:		 This function is used to transmit data to rescue robot via ZigBee communication.
* Example call:  USART0_TX(5)
*******************************************************************************************************************************/

void USART0_TX(int data)
{
	while(!(UCSR0A & (1<<UDRE0)))
		;
	UDR0 = data;
}

/******************************************************************************************************************************
* Function Name: send
* Input:		 a(data 1),b(data 2),c(data 3)
* Output:		 none
* Logic:		 This function is used to send 3 bytes of data to search robot through Zigbee(USART0).
* Example call:  send(10,12,0)
*******************************************************************************************************************************/

void send(int a,int b,int c)
{
	USART0_TX(a);
	_delay_us(100);
	USART0_TX(b);
	_delay_us(100);
	USART0_TX(c);
	_delay_ms(100);
	if (search_complete_flag == 1)
	{
		ACK = 1;
	}
}

/******************************************************************************************************************************
* Function Name: send1
* Input:		 a(data 1),b(data 2)
* Output:		 none
* Logic:		 This function is used to send 2 bytes of data to search robot through Zigbee(USART0).
* Example call:  send(10,12)
*******************************************************************************************************************************/

void send1(int a,int b)
{
	USART0_TX(a);
	_delay_us(100);
	USART0_TX(b);
	_delay_us(100);
}

/******************************************************************************************************************************
* Function Name: init_arena
* Input:		 none
* Output:		 none
* Logic:		 This function is used to initialize the arena about plot, corresponding adjacent nodes and adjacent plots
* Example call:  init_arena()
*******************************************************************************************************************************/

void init_arena(void)
{
	setPlot();
	set_adj_node();
	set_adj_Plots();
	init_survivors();
}

/******************************************************************************************************************************
* Function Name: port_config
* Input:		 none
* Output:		 none
* Logic:		 This function is used to configure various device interfaced to FireBird V robot. 
* Example call:  port_config
*******************************************************************************************************************************/

void port_config(void)			// To configure various device interfaced to FireBird V robot
{
	lcd_port_config();
	buzzer_pin_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
}

/******************************************************************************************************************************
* Function Name: init_devices
* Input:		 none
* Output:		 none
* Logic:		 This function is used to initialize the devices. 
* Example call:  init_devices
*******************************************************************************************************************************/

void init_devices(void)
{
	cli();			//Clears the global interrupt
	port_config();
	lcd_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	uart0_init();
	servo_init();
	colour_init();
	sei();			//Enable the global interrupt
}

/******************************************************************************************************************************
*
* Function Name: main
*
*******************************************************************************************************************************/

int main(void)
{
	init_arena();
	init_devices();
	int i=0;
	int t=0,t1=0;
	
	while (1)
	{	
		int index = -1, priority_flag = 1;
	
		for (i=0; i < survivor_ptr; i++)
		{
			if(survivor_status[i] == 0)
			{
				if (survivor_colour[i] == 0)
				{
					index = i;
					break;	
				}
				else if(survivor_colour[i] == 1 && priority_flag)
				{
					index = i;
					priority_flag = 0;
				}					
			}
		}
				
		if(index != -1)
		{
			display_colour(survivor_colour[index]);

			if(t==0)
			{
				if(pos[0]==15 || pos[1]==15)
				{
					while(pos[0]==15 || pos[1]==15);
				}
				forward();
				linear_mm(70);
				path_node(1);
				t=1;
			}	
			
			rescue(index);
			
			if(survivor_colour[index] == 0)
			{
				move(crt_node,15,9);
				turn(15,10);
				rotate_right(85);
				switch_flag = 1;
				path_node(1);
				switch_flag = 0;
				rotate_right(170);
				back();
				linear_mm(100);
				display_colour(3);
				buzzer(1);
				crt_node = 15;
				crrt_dir=3;
				t=0;
				
				clear_stack();
				
				ACK =0;
				send(50,0,0);
				_delay_us(100);
				while(ACK == 0)
				{
					send(50,0,0);
					_delay_ms(1);
				}
			}
			
		}	
		
		if((search_complete_flag == 1) && (index == -1))
			break;
	}
	
	buzzer(5);
	
	return 0;
}
