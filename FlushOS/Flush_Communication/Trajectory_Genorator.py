from math import sqrt,atan2,degrees,sin,cos



def Targectory_Gen(x1,y1,x2,y2):
    P1 = (0,0)
    P2 = (400,400)
    delta_x = P2[0]-P1[0]
    delta_y = P2[1]-P1[1]
    r = sqrt(pow(delta_x,2)+pow(delta_y,2))
    
    Theta = atan2(delta_y,delta_x)
    return int(r),int(degrees(Theta)+180), int(r*cos(Theta)) , int(r*sin(Theta))

P1 = (0,0)
P2 = (400,400)
r,Theta,X,Y = Targectory_Gen(*P1,*P2)
time = int(r*154/11)

print("r =" + str(r))
print("X =" + str(X))
print("Y =" + str(Y))
print("time ="+str(time))
print("theta =" + str(Theta))
