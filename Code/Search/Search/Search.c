/***************************************************************************************************************************************
****************************************************************************************************************************************
 * Team ID:     B34
 * Author List:	Keerthana A B
 * Filename:	Search.c
 * Theme:		Search and Rescue
 * Functions:	dijsktra, setp, setPlot, set_adj_Plots, set_adj_node, updatePlot, get_row, get_col, get_dir, turn,
				get_next_unscanned_plot, detect_block, plot_scan, go, goto_n, move, search, SIGNAL, USART0_TX, send, init_arena,
				port_config, init_devices.
 * Global Variables:   	path,			(array of size 26 x 26 to track the traversed path),
						crt_node		(current node pointer),
						nxt_node		(Next node pointer),
						prv_node		(Previous node pointer),
						crrt_dir,		(4 values indicating the direction of robot),
						n_node,			(An array of size 26 to store the node numbers to be traversed by the robot),
						data,			(Variable to store the incoming bytes from the uart0, value ranges from 0 - 255),
						cost,			(array of size 26 x 26 to store the cost of path for Dijikstra Algorithm),
						position_flag,	(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
						position_flag1,	(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
						flag,			(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
						flag1			(To choose the incoming bytes from uart0 into appropriate variable in the ISR),
						block_node,		(An array of size 2 to store the black debris position information),
						temp_1			(An array of size 2 to store position of rescue robot)

 *
 *
 ***************************************************************************************************************************************
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


int path_length,path_ptr=0;
int v[26], visit[26][26], path[26][26]={{0}};
int crt_node=23,nxt_node=0, prv_node=0, dst_node=0, itm_count=0;
int crrt_dir=1;
int c_plot=14,d_plot=0;
int n_node[N];
volatile int data,buffer[10],ptr=0;
int count=0;
volatile int pos[2];

volatile int position_flag=0,position_flag1=0;
volatile int block_node[2],flag=0,flag1=0;
volatile int ACK=0;
int survivor_cnt=0;
volatile int san_flag = 0;

volatile int cost[N][N]=
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

struct node n[26];
struct plot b[17];

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
* Logic:		 This function is used to initialize adjacent plots of each node.
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
				n[i].blk[1]=i-1;
				n[i].blk[2]=i;
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
* Function Name: updatePlot
* Input:		 none
* Output:		 none
* Logic:		 This function is used to update the status of plot as to whether it is visited yet or not.
* Example call:	 updatePlot()
*******************************************************************************************************************************/

void updatePlot(void)			// To update the status of plot, whether it is visited or not
{
	int ar,br,res,i,j;
	
	for(i=1;i<=16;i++)
	{
	    res=0;
		for(j=1;j<=4;j++)
		{
			ar=b[i].st[j];
			br=b[i].ed[j];

			if(ar != 0 && br != 0)
			{
				res += path[ar][br];
			}
		}
		if(res >= 1)
		{
			b[i].pstatus=1;
			//plot[j]=1;
		}
	}
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
	crt_node=bn;
	path[an][bn]=path[bn][an]=1;
	updatePlot();
}

/******************************************************************************************************************************
* Function Name: get_row
* Input:		 plot
* Output:		 row number of that plot
* Logic:		 This function is used give the row number of that particular plot
* Example call:	 get_row(12)
*******************************************************************************************************************************/

int get_row(int plot)
{
	if (plot % 4 == 0 )
		return (plot / 4);
	else
		return ((plot / 4) + 1);
}

/******************************************************************************************************************************
* Function Name: get_col
* Input:		 plot
* Output:		 column number of that plot
* Logic:		 This function is used give the column number of that particular plot
* Example call:	 get_col(12)
*******************************************************************************************************************************/

int get_col(int plot)
{
	unsigned char col = (plot % 4);
	if(col == 0)
		col = 4;

	return col;
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
* Function Name: get_next_unscanned_plot
* Input:		 none
* Output:		 next unscanned plot
* Logic:		 This function is used to get the next unscanned plot number.
* Example call:	 get_next_unscanned_plot()
*******************************************************************************************************************************/

int get_n_u_plot(void)
{
	int res=0,i;

	for(i=1;i<=16;i++)
		res += b[i].pstatus;

	if(res >= 16)
		return -1;

	for(i=1;i<=4;i++)
		if(n[crt_node].blk[i] != 0)
		{
			c_plot = n[crt_node].blk[i];			
			break;
		}

	int d[17],min = IN;
	int r = get_row(c_plot),c = get_col(c_plot);

	for(i=1;i<17;i++)
	{
		if(b[i].pstatus == 0)
		{
			int r1=get_row(i),c1=get_col(i);
			d[i] = abs(r1-r)+abs(c1-c);
		}
	}
	for(i=1;i<17;i++)
	{
		if(b[i].pstatus==0)
		{
			if(d[i] < min)
				min=d[i];
		}
	}
	
	for(i=1;i<17;i++)
		if((d[i] == min) && b[i].pstatus == 0)
			break;
	return i;
}

/******************************************************************************************************************************
* Function Name: detect_block
* Input:		 c_node, n_node
* Output:		 status of obstucle in the path between c_node & d_node
* Logic:		 This function is used to detect black debris and update about it in the cost matrix.
* Example call:	 detect_block(7,8)
*******************************************************************************************************************************/

int detect_block(int c_node,int n_node)		// To detect black debris and updating the midpoint(ob)
{
	int retv=0;
	int temp = detect_obstacle(80,200);
	
	if(temp)
	{
		cost[c_node][n_node]=IN;
		cost[n_node][c_node]=IN;
		ACK=0;
		send(30,c_node,n_node);
		if (ACK != 1)
		{
			while(ACK !=1)
			{
				send(30,c_node,n_node);
				_delay_ms(1);
			}
		}
		ACK = 0;
			
		retv=1;
	}
	
	return retv;
}

/******************************************************************************************************************************
* Function Name: plot_scan
* Input:		 crtid, nxtid
* Output:		 none
* Logic:		 This function is used to scan the plot for the presence of white debris
* Example call:	 plot_scan(1,2)
*******************************************************************************************************************************/

void plot_scan(int crtid,int nxtid)		//To scan the plots for white debris
{
	int l=0,r=0,c=3;
	
	switch(crtid - nxtid)
	{
		case 1:
		l=n[crtid].blk[1];
		r=n[crtid].blk[3];
		break;
		
		case -1:
		l=n[crtid].blk[4];
		r=n[crtid].blk[2];
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
	
	if(r!=0 && b[r].pstatus!=1)
	{
		//_delay_ms(50);
		int a = detect_white(80,400,13);
		
		//lcd_print(2,1,a,2);
		
		if(a == 1)
		{
			//buzzer(4);
			
			{
				servo_1(0);
				servo_2(180);
				_delay_ms(1000);
				servo_2(78);
				_delay_ms(2000);
				servo(30);
				_delay_ms(500);
				servo(20);
				_delay_ms(100);
				
				// Detect
				c= get_colour();
				servo_2(180);
			}
			
			display_colour(c);
			_delay_ms(2000);
			display_colour(3);
			
			ACK = 0;
			send(40,r,c);
			_delay_ms(10);
			if (ACK != 1)
			{
				while(ACK !=1)
				{
					send(40,r,c);
					_delay_ms(1);
				}
			}
			ACK = 0;
			
			survivor_cnt++;
			ACK = 0;
			USART0_TX(50);
			USART0_TX(survivor_cnt);
			_delay_ms(10);
			if (ACK != 1)
			{
				while(ACK !=1)
				{
					USART0_TX(50);
					USART0_TX(survivor_cnt);
					_delay_ms(1);
				}
			}
			ACK = 0;
		}
		else
		{
			display_colour(2);
			buzzer(2);
			display_colour(3);
		}
	}
	
	if(l!=0 && b[l].pstatus!=1)
	{
		int a = detect_white(80,400,9);

		if(a == 1)
		{
			{
				servo_1(150);
				c_degrees = 150;
				_delay_ms(1000);
				servo_2(78);
				servo(210);
				//_delay_ms(500);
				servo(197);
				_delay_ms(1);
				
				c = get_colour();
				_delay_ms(1);
				servo_2(180);
				_delay_ms(300);				
				servo_1(0);
				c_degrees = 0;
			}
			
			display_colour(c);			
			_delay_ms(2000);
			display_colour(3);
			
			ACK = 0;
			send(40,l,c);
			_delay_ms(10);		
			if (ACK != 1)
			{
				while(ACK !=1)
				{
					send(40,l,c);
					_delay_ms(1);
				}
			}	
			ACK = 0;	
					
			survivor_cnt++;
			ACK = 0;
			USART0_TX(50);
			USART0_TX(survivor_cnt);
			_delay_ms(10);
			if (ACK != 1)
			{
				while(ACK !=1)
				{
					USART0_TX(50);
					USART0_TX(survivor_cnt);
					_delay_ms(1);
				}
			}
			ACK = 0;
		}
		else
		{
			display_colour(2);
			buzzer(2);
			display_colour(3);
		}		
	}
	
	update(crtid,nxtid);
}

/******************************************************************************************************************************
* Function Name: go
* Input:		 crtnode,nxtnode
* Output:		 none
* Logic:		 This function is used to move the robot from current node to next node.
* Example call:  go(4,5)
*******************************************************************************************************************************/

int go(int crtnode,int nxtnode)	// To move the bot from crtnode to nxtnode
{
	if( nxtnode == pos[0] || crtnode == pos[1] )
	{		
		while(nxtnode == pos[0]);
		if(nxtnode == pos[0])
			while(nxtnode == pos[0]);
		_delay_ms(1000);
	}
	
	path_node1(1);
	
	if (san_flag)
	{
		_delay_ms(10);
		ACK =0;
		USART0_TX(160);
		_delay_ms(1);
		if(ACK !=1)
		while(ACK != 1)
		{
			USART0_TX(160);
			_delay_ms(1);
		}
		ACK = 0;
		
		san_flag = 0;
	}
			
	if( nxtnode == pos[0])
	{
		back();
		linear_mm(210);
		
		while(nxtnode == pos[0]);
		
		_delay_ms(1500);
		path_node1(1);
	}
	
	if(path[crtnode][nxtnode]==0)
		plot_scan(crtnode,nxtnode);	
	
	if (san_flag)
	{
		_delay_ms(10);
		ACK = 0;
		USART0_TX(160);
		_delay_ms(1);
		while(ACK == 0)
		{
			USART0_TX(160);
			_delay_ms(1);
		}
		ACK = 0;
		san_flag = 0;
	}
	
	if( nxtnode == pos[0])
	{
		back();
		linear_mm(210);
		
		while(nxtnode == pos[0]);
		
		_delay_ms(1500);
		path_node(1);
	}
	
	//path_mm(150);
	path_node(1);
	
	if (san_flag)
	{
		_delay_ms(10);
		ACK = 0;
		USART0_TX(160);
		_delay_ms(1);
		while(ACK == 0)
		{
			USART0_TX(160);
			_delay_ms(1);
		}
		ACK = 0;
		san_flag = 0;
	}
	
	update(crtnode,nxtnode);//update(crt_node,nxtnode);
	
	return 0;
}

/******************************************************************************************************************************
* Function Name: goto_n
* Input:		 c_node,n_node
* Output:		 none
* Logic:		 This function is used to check for the obstacle. 
				 If absent, calls go function. If present, updates the
				 presence of the block in the cost matrix.
* Example call:  goto_n(4,5)
*******************************************************************************************************************************/

int goto_n(int c_node,int d_node)
{
	ACK = 0;
	send(80,c_node,d_node);
	_delay_ms(10);
	if (ACK != 1)
	{
		while(ACK !=1)
		{
			send(80,c_node,d_node);
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
		retval = go(c_node,d_node);
	}		
	else
	{
		_delay_ms(50);
		int abs = detect_block(c_node,d_node);

		if(abs)
		{
			retval=-1;
		}
		else
		{
			count++;
			retval = go(c_node,d_node);
		}
	}
	return retval;
}

/******************************************************************************************************************************
* Function Name: move
* Input:		 c_node (current node),d_node (destination node),d_plot (destination plot)
* Output:		 movement of the robot to the required destination node.
* Logic:		 This function is used to move to the adjacent nodes of an unscanned plot.
* Example call:  move(4,5,4)
*******************************************************************************************************************************/

int move(int c_node,int d_node, int d_plot)////int dst_plot
{
	int i,a,retval=0,j=1;
	path_length = dijsktra(c_node,d_node),flag=0;
	
	for (i = 1; i < path_length; i++)
	{
		path_ptr = i;
		
		a = goto_n(crt_node,n_node[i]);
				
		if(b[d_plot].pstatus == 1)
			break;
		
		if(a == -1)
		{			
			flag=1;
			break;
		}			
	}
	if(flag)
	{
		move(crt_node,d_node,d_plot);
	}
	
	return retval;
}

/******************************************************************************************************************************
* Function Name: goto_home
* Input:		 none
* Output:		 none
* Logic:		 This function is used to move to start position.
* Example call:  goto_home();
*******************************************************************************************************************************/

int goto_home(void)
{
	int i,a,retval=0,j=1;
	int n = dijsktra(crt_node,23),flag=0;
	
	for (i=1;i<n;i++)
	{
		a = goto_n(crt_node,n_node[i]);
				
		if(a == -1)
		{
			flag=1;
			break;
		}
	}
	if(flag)
		goto_home();
	
	return retval;
}

/******************************************************************************************************************************
* Function Name: search
* Input:		 none
* Output:		 status about completion of scaning all the plots
* Logic:		 This function searches the unscanned plot, gets its adjacent nodes, moves to the node and scans the plot.
				 If path is blocked, it takes an alternate path in order to scan that plot.
* Example call:  search()
*******************************************************************************************************************************/

int search(void)
{
	int retv=0,i;
	int p = get_n_u_plot();
	
	if(p == -1)
	{
		retv=-1;
		return retv;
	}

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
				move(crt_node, dst_node,p);
				if(b[p].pstatus==1)
				//break;
					return 1;
				flag=0;
			}
			else
			{
				int a = abs(dst_node - crt_node);

				if (a==1 || a==5)
				{
					int bp = goto_n(crt_node,dst_node);
					
					if(b[p].pstatus==1)
					//break;
					return 1;
					if(bp == -1)
					{
						continue;
					}
					else
					{
						flag420=0;
					break;
					}
				}
				else
				{
					flag420=1;
					if(b[p].pstatus==1)
						return 1;
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
	
				move(crt_node, dst_node,p);
				if(b[p].pstatus==1)
					return 1;
			}
		}

	return retv;
}

/******************************************************************************************************************************
* Function Name: uart0_init
* Input:		 none
* Output:		 none
* Logic:		 This function initialise the uart0 port and sets the baud rate.
* Example call:  uart0_init();
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

volatile int clear_flag=0,clear_flag1=0;
volatile int node_flag=0,node_search,detect_flag=0;
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
	
	//lcd_print(1,1,data,3);
		
	if (data == 30 || data == 50 || data == 70 || data == 85 || data == 90 || data == 100)
	{
		switch(data)
		{
			case 30:flag = 1;
			position_flag=0;
			break;
			case 50:position_flag=1;
			flag = 0;
			break;
			case 70:clear_flag=1;
			flag = 0;
			position_flag=0;
			break;
			case 85: san_flag = 1;
			USART0_TX(100);
			break;
			case 90:node_flag=1;
			clear_flag=0;
			flag = 0;
			position_flag=0;
			break;
			
			case 100:	ACK=1;
			break;
			default:
			break;
		}
	}
	else
	{	
		if (node_flag)
		{
			node_search = data;
			for (int i=path_ptr-1; i<=path_length; i++)
			{
				if (node_search == n_node[i])
				{
					detect_flag = 1;
					break;
				}
				else
				detect_flag = 0;
			}
			if (detect_flag)
			{
				USART0_TX(140);
			}
			else
			{
				USART0_TX(150);
			}
			
			node_flag = 0;
		}
		
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
		}
		
		if(position_flag)
		{
			pos[0] = data;
			position_flag = 0;
			position_flag1 = 1;
			lcd_print(2,7,pos[0],2);
		}
		else if(position_flag1)
		{
			pos[1] = data;
			position_flag1 = 0;
			lcd_print(2,10,pos[1],2);
			USART0_TX(100);
		}	
	}	
}

/******************************************************************************************************************************
* Function Name: USART0_TX
* Input:		 data
* Output:		 none
* Logic:		 This function is used to transmit data to rescue robot via ZigBee communication.
* Example call:  USART0_TX(10)
*******************************************************************************************************************************/

void USART0_TX(unsigned char data)
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
}

/******************************************************************************************************************************
* Function Name: port_config
* Input:		 none
* Output:		 none
* Logic:		 This function is used to configure various device interfaced to FireBird V robot.
* Example call:  port_config()
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
* Example call:  init_devices()
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

int main()
{
	crt_node = 23;
	init_arena();
	init_devices();
	
	servo_1(0);
	servo_2(180);
	
	path_mm(70);	
	path_node(1);
	
	while (1)
	{
		int n = search();
		if(n == -1)
			break;		
	}

	ACK =0;
	USART0_TX(60);		//Send Search Complete Flag
	while(ACK == 0)
	{
		USART0_TX(60);
		_delay_ms(1);
	}

	goto_home();	
	turn(23,24);
	rotate_right(85);
	path_node(1);
	path_mm(40);
	rotate_right(175);	
		
	for (int i=0; i<10; i++)
	{
		n_node[i] = 0;
	}
	
	ACK=0;
	send(80,0,0);
	_delay_us(100);
	while(ACK == 0)
	{
		send(80,0,0);
		_delay_ms(1);
	}
	
	while(1);
	return 0;
}
