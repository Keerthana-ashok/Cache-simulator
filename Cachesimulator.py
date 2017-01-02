import sys
import math

line=[]
address=[]
decimal=[]
tag=[]
index=[]
setv=[]

file_1 = open(sys.argv[1],"r")
contents = file_1.readlines()
cache_size=int(sys.argv[2])
block_size=int(sys.argv[3])
number_ways=int(sys.argv[4])

l_contents = len(contents)
for i in range(0,l_contents):
  line.append(contents[i].strip('\n'))
len_line=len(line)

for i in  range (0,len_line):
   split=line[i].split()
   address.append(split[2])

mem_size=len(address)

for i in range(0,mem_size):
 dec_value= int(address[i],16)
 mask=dec_value & 4294967295
 decimal.append(mask)
   
len_decimal=len(decimal)

partial_index=(cache_size/block_size)
set_length1=partial_index/number_ways
set_length=int(math.log(set_length1,2))
print 'number of address bit of set:',set_length

block_length=int(math.log(block_size,2))
print 'number of address bit of block:',block_length


tag_length=32-set_length-block_length
print 'number of address bit of tag:',tag_length

for i in range (0,len_decimal):
 a=(2**(set_length)-1)<<(block_length)
 b=decimal[i] & a
 set_1=b>>(block_length)
 setv.append(set_1)
 len_set=len(setv)
 #print setv

#print len_set

for i in range (0,len_decimal):
 a1=(2**(tag_length)-1)<<(block_length+set_length)
 b1= decimal[i] & a1
 tag_1=b1>>(block_length+set_length)
 tag.append(tag_1)


hit_counter=0
miss_counter=0
flag=0
dict2={}
lru={}

for i in range (0,set_length1):
   lru[i]={}
for i in range (0,set_length1):
   for j in range (0,number_ways):
     lru[i][j]=j

for i in range (0,set_length1):
   dict2[i]={}
for i in range (0,set_length1):
   for j in range (0,number_ways):
     dict2[i][j]=0

if (number_ways==1):
   dict1={}
   for i in range (0,set_length1):
    dict1[i]=0
   for i in range (0,len_decimal):
    if(dict1[setv[i]]==tag[i]):
      hit_counter=hit_counter+1
    else:
      dict1[setv[i]]=tag[i]
      miss_counter=miss_counter+1
      print "hit rate",float(hit_counter)/(100)
      print "miss rate",float(miss_counter)/(100) 

else:
    for i in range (0,len_decimal):
     for j in range (0,number_ways):
       if (dict2[setv[i]][j]==tag[i]):
            for k in range (0,number_ways):
              if(lru[setv[i]][k]<lru[setv[i]][j]):
                  update=lru[setv[i]][k]
                  update=update+1
                  lru[setv[i]][k]=update
                  lru[setv[i]][j]=0
                  #print "dict2",dict2
            #print "lru", lru
            hit_counter=hit_counter+1
            flag=1
            break
       else:
           if (j==number_ways):
              break
           else: 
              continue
     if(flag==1):
        flag=0
        continue 
     flag=0      
     for j in range (0,number_ways):
        if(dict2[setv[i]][j]==0):
         if(lru[setv[i]][j]==(number_ways-1)):
           dict2[setv[i]][j]=tag[i]
           #print "dict2",dict2
           for k in range (0,number_ways):
             if(lru[setv[i]][k]<lru[setv[i]][j]):
                update2=lru[setv[i]][k]
                update2=update2+1
                lru[setv[i]][k]=update2
           lru[setv[i]][j]=0
           #print "lru",lru
           miss_counter=miss_counter+1  
           flag=1
           break
         else:
             if (j==number_ways):
                break
             else: 
                continue
 #print i
     if(flag==1):
       flag=0
       continue  
     flag=0 
     for j in range (0,number_ways):
             if(lru[setv[i]][j]==(number_ways-1)):
                dict2[setv[i]][j]=tag[i]
                #print "dict2",dict2
                for k in range (0,number_ways):
                  if(lru[setv[i]][k]<lru[setv[i]][j]):
                    update3=lru[setv[i]][k]
                    update3=update3+1
                    lru[setv[i]][k]=update3
                lru[setv[i]][j]=0
                #print "lru",lru
                miss_counter=miss_counter+1
                flag=1
                break
     if(flag==1):
       flag=0
       continue 
     #print i  
print "hit rate",float(hit_counter)/(100)
print "miss rate",float (miss_counter)/(100)



       
 






  
  
