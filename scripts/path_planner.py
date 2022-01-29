#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float64
from path_nav.msg import points
import math
import random
rospy.init_node('path_planner', anonymous=True)
pub=rospy.Publisher('/path',points,queue_size=10)
rate=rospy.Rate(10)


queuem=[]
mapopenlist=[]
mapcloselist=[]
frontieropenlist=[]
frontiercloselist=[]
#sub=rospy.Subscriber('/odom',Odometry,callback)
class complex: #class for implementing WFD algorithm
  class Node:
    def __init__(self,x,y):
      self.x=x
      self.y=y
      self.path=points()
      self.parent=None
  

  def __init__(self,start,goal):
    self.start=self.Node(start[0],start[1])
    self.goal=self.Node(goal[0],goal[1])
    
    self.indication=None
    self.isFrontierPoint==True

pose=complex(self.start,mapopenlist)
queuem.append(pose)
# p = queuem.pop(0)
#ob1.indication = MOCL


neighbour=[[-1,0],[1,0],[0,-1],[0,1]]
while(len(queuem>0)):
  p=queuem.pop(0)
  if p.indication==mapcloselist:
    continue
  if p.isFrontierPoint==True:
    queuef=[]
    NewFrontier=[]
    queuef.append(p)
    p.indication==frontieropenlist
    while(len(queuef>0)):
      q=queuef.pop(0)
      if q.indication==mapcloselist or q.indication==frontiercloselist:
        continue
      if q.isFrontierPoint==True:
        NewFrontier.append(q)
        for i in range(3):
          offset=neighbour[i]
          new_position=[q.x+offset[0],q.y+offset[1]]
          if new_position.indication!=frontieropenlist and new_position.indication!=frontiercloselist and new_position.indication!=mapcloselist:
            queuef.append(new_position)
            new_position.indication==frontieropenlist
      q.indication==frontiercloselist
      savedata=NewFrontier
      for point in NewFrontier:
        point.indication==mapcloselist

for i in range(3):
  offset=neighbour[i]
  v=[p.x+offset[0],p.y+offset[1]]
  flag=False
  for i in range(3):
    offset=neighbour[i]
    adjacent=[v.x+offset[0],v.y+offset[1]]
    if adjacent.indication==mapopenlist:
      flag=True
      break
  if v.indication!=mapopenlist and v.indication!=mapcloselist and flag==True:
    
    queuem.append(v)
    v.indication==mapopenlist
p.indication==mapcloselist

def main(goalx = 10.0, goaly = 10.0):       #Goal point is (10.0, 10.0) Start point is origin
    wfd=complex(start=[0, 0], goal=[goalx,goaly])
    path=wfd.
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        while not rospy.is_shutdown():
            pub.publish(path)
            rate.sleep()

        rospy.spin()      

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass            
          
        


  
  
    
                  



                           
            








