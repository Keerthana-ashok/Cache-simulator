#Author: Keerthana Ashok
#Input: PC address
#Output:Gives the result of current branch based on previous instructions

import sys
import math
line=[]
x=[]
pred=[]

d={}
s=[]
i=0

#Reading the file
f = open(sys.argv[1],"r")
contents = f.readlines()
m=int(sys.argv[2])
n=int(sys.argv[3])

p=2**m

l_contents = len(contents)
for i in range(0,l_contents):
 line.append(contents[i].strip('\n'))
z=len(line)

for i in  range (0,z):
   split=line[i].split()
   x.append(split[0])
   pred.append(split[1])
len_x=len(x)
len_y=len(pred)
compare=[]

#Converting to decimal
for i in range (0,len_x):
   value1= int(x[i],16)
   mask=value1 & 63
   compare.append(mask)

pc=compare
len_pc=len(pc)

for i in range (0,len_y):
    if(pred[i]=='T'):
      pred[i]=1
    else:
      pred[i]=0

for i in range (0,p):
   d[i]={}
for i in range (0,p):
   for j in range (0,(2**6)):
     d[i][j]=0
count=0

#1 bit predictor
s=0
if(n==1):
   for i in range (0,len_pc):
     if(pred[i]!=d[s][pc[i]]):
       d[s][pc[i]]=pred[i]
       count=count+1
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
   mispredictionrate= float(count*100)/(len_pc)
   print mispredictionrate


#2 bit predictor
elif(n==2):
  for i in range (0,len_pc):
   if (pred[i]==1) and (d[s][pc[i]]==0):
     d[s][pc[i]]=1
     count=count+1
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     
   elif (pred[i]==1 and d[s][pc[i]]==2):
     d[s][pc[i]]=3
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
         
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     
   elif (pred[i]==0 and d[s][pc[i]]==0):
     d[s][pc[i]]=0
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
         
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     
   elif (pred[i]==0 and d[s][pc[i]]==1):
     d[s][pc[i]]=0
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
   elif (pred[i]==0 and d[s][pc[i]]==3):
     d[s][pc[i]]=2
     count=count+1
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
   elif (pred[i]==0 and d[s][pc[i]]==2):
     d[s][pc[i]]= 0
     count=count+1
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
   elif (pred[i]==1 and d[s][pc[i]]==3):
     d[s][pc[i]]= 3
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     
   elif (pred[i]==1 and d[s][pc[i]]==1):
     d[s][pc[i]]=3
     count=count+1
     if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1    
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1  
     else:
       if(m>=1):
          if(pred[i]==0):
             s=s<<1
             s=s & 2**m-1
         
          elif(pred[i]==1):
             s=s<<1
             s=s & 2**m-1
             s=s+1    

  mispredictionrate= float(count*100)/(len_pc)
  print ('misprediction rate',mispredictionrate)
