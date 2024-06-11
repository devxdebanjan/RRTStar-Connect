#parameters
max_iteration = 1000
Step = 40
search_radius = 30
safety = 10
import cv2 
import math
import random
import matplotlib.pyplot as plt
import goto 

#give the desired image path
img = cv2.imread("Image1.png")
l,m,_= img.shape
print(l,m)
white = [255,255,255]
red = [0,0,255]
green = [0,255,0]
black=[0,0,0]

#converting image to traversable pixels using opencv and then converting image to suitable pixels
for i in range(l):
    for j in range(m):
        if img[i][j][0] > 150 and img[i][j][1] > 150 and img[i][j][2] > 150:  #white
            img[i][j][0] = 255         
            img[i][j][1] = 255
            img[i][j][2] = 255
        elif img[i][j][0] < 80 and img[i][j][1] > 150 and img[i][j][2] < 80:  #green
            img[i][j][0] = 0
            img[i][j][1] = 255
            img[i][j][2] = 0
        elif img[i][j][0] < 50 and img[i][j][1] < 50 and img[i][j][2] > 150:    #red
            img[i][j][0] = 0
            img[i][j][1] = 0
            img[i][j][2] = 255
        else:                                                                  #black
            img[i][j][0] = 0
            img[i][j][1] = 0
            img[i][j][2] = 0

start=(70,550)         #for Image1.png
end=(525,50)           #for Image1.png                          
# start=(545,535)            #for Image2.png
# end=(55,55)            #for Image2.png   

######################################################################################

class Node():

    def __init__(self, index, cost= math.inf, parent = None):

        self.index = index

        self.cost = cost

        self.parent = parent



#sampling a random point

def random_point():

    x = int(random.random() * l)

    y = int(random.random() * m)

    return (x,y)    



#finding distance between two points

def dist(p1,p2):

    x1,y1 = p1

    x2,y2 = p2

    return (math.sqrt((x2-x1)**2 + (y2-y1)**2))



#checks if point is out of bounds for image

def is_valid(point):

    if (point[0]>=0 and point[0]<l) and (point[1]>=0 and point[1]<m):

        return True

    return False



#func to find if image is far from obstacles or not in obstacle

def obstacle_free(x):

    if is_valid(x)==False:

        return False

    for i in range(-safety,safety):

        for j in range(-safety,safety):

            point = (x[0] + i, x[1] + j)

            if is_valid(point):     #point should be valid

                point=list(point)

                point.reverse()

                point=tuple(point)

                if (img[point][0] == black[0] and img[point][1] == black[1] and img[point][2] == black[2]):   

                    return False

    return True



#func to find nearest vertex of tree from x_rand

def nearest(vertex, x_rand):

    min = math.inf

    x_nearest = (-1, -1)

    for ver in vertex:

        s = dist(ver,x_rand)

        if s < min:

            min = s

            x_nearest = ver

    return x_nearest



#connecting two trees by finding which node of both tress are at least distance

def treeconnect(vertex1, vertex2):

    min = math.inf

    tree1ver=start

    tree2ver=end

    for i in vertex1:

        for j in vertex2:

            if dist(i,j)<min:

                min=dist(i,j)

                tree1ver=i

                tree2ver=j

    return tree1ver,tree2ver,min



#defining if goal is found

def isgoal(vertex1,vertex2):

    a,b,c=treeconnect(vertex1,vertex2)

    if (c<=search_radius):

        return a,b,True

    else:

        return a,b,False



#func to find new vertex at step dist from x_nearest in the dirn of x_rand

def steer(x_nearest, x_rand):

    x_n, y_n = x_nearest

    x, y = x_rand

    global Step



    if dist(x_nearest,x_rand) < Step:

        return x_rand



    if x_n != x:

        theta = math.atan( (y_n-y) / (x_n-x) )

    else:

        theta = math.pi

    

    a = int(x_n + Step * math.cos(theta))

    b = int(y_n + Step * math.sin(theta))

    return (a,b)



#func to determine if the path to new vertex is obstacle free

def obst_free_path(x_nearest, x_new):

    if obstacle_free(x_nearest) and obstacle_free(x_new):

        return True

    return False



#func to return the neighbour nodes of x_new

def near(vertex, x_new):

    x,y = x_new

    x_near = []

    for vert in vertex:

        if dist(vert,x_new) <=  search_radius and (vert!=x_new):

            if obst_free_path(vert,x_new):                          

                x_near.append(vert)

    return x_near



#func to choose parent resulting into lowesr cost

def choose_parent(tree,vertex, x_near, x_nearest, x_new):

    min_cost = tree[vertex.index(x_new)].cost

    x_parent = x_nearest

    for neighbour in x_near:

        c = tree[vertex.index(neighbour)].cost + dist(neighbour, x_new)

        if c < min_cost:

            min_cost = c

            x_parent = neighbour

    return x_parent,min_cost





#func to rewire the tree.. lowering the costs of nodes

def rewire(tree,vertex, x_near, x_parent, x_new):

    for node in x_near:

        c1 = tree[vertex.index(node)].cost      #direct cost

        c2 = tree[vertex.index(x_new)].cost + dist(x_new,node)    #cost via x_new

        if c1 > c2:    #needs some update

            tree[vertex.index(node)].cost = c2

            tree[vertex.index(node)].parent = tree[vertex.index(x_new)]



#func to extend the tree

def extend(tree,vertex, x_rand):



    x_nearest = nearest(vertex, x_rand)

    x_new = steer(x_nearest, x_rand)   #good is to ensure x_new is valid & path obstacle free



    if obst_free_path(x_nearest, x_new):

        new_node = Node(x_new, cost= (tree[vertex.index(x_nearest)].cost + dist(x_nearest,x_new)), parent= tree[vertex.index(x_nearest)])   #x_nearest is not node but vertex

        tree.append(new_node)

        vertex.append(x_new)



        #searching for better parent

        x_near = near(vertex, x_new)   #returns a list containing neighbours of x_new



        x_parent = choose_parent(tree,vertex, x_near, x_nearest, x_new)       #node having least cost to go through to x_new

        new_node.parent = tree[vertex.index(x_parent[0])] 

        new_node.cost = x_parent[1]



        rewire(tree,vertex, x_near, x_parent[0], x_new)

        ################################################################ 

        return x_new, True      



    return x_new, False   





def append_path(tree,vertex,current_node,final_node, img, flag= 1):

    path = []

    t = 0

    while current_node != final_node:

        

        x1,y1 = current_node.index

        if flag==1:

            path.insert(0,(x1,y1))

        else:

            path.append((x1,y1))

        

        current_node = current_node.parent

    if flag==1:

        path.insert(0,(current_node.index))

    else :

        path.append((current_node.index))

    return path



def connect(treea,vertexa,pta,treeb,vertexb,ptb,img):

    path1=append_path(treea,vertexa, treea[vertexa.index(pta)],treea[0], img,1)

    path2=append_path(treeb,vertexb, treeb[vertexb.index(ptb)],treeb[0], img,0)

    path1.extend(path2)

    return path1



def RRTStar_Connect(start, end, img):



    #from start

    tree1 = []

    vertex1 = []

    #from end

    tree2 = []

    vertex2 = []



    #initialize

    start_node = Node(start, 0)

    tree1.append(start_node)

    vertex1.append(start)



    end_node = Node(end, 0)

    tree2.append(end_node)

    vertex2.append(end)



    for n in range(max_iteration):

        print("n=",n)

        x_rand = random_point()



        #checks for validity of x_rand (wrt vertex1)

        while x_rand in vertex1 or not obstacle_free(x_rand):

            # print("Figuring out...")

            x_rand = random_point()

        

        x_newa, msg1 = extend(tree1,vertex1, x_rand)

        # print("X_newa:",x_newa," mssg:",msg1)

        if msg1 == True:

            y,x = x_newa

            img[x][y][0] = 250

            img[x][y][1] = 0

            img[x][y][2] = 0

            plt.pause(0.001)

            plt.plot(start[0],start[1],'gd')

            plt.plot(end[0],end[1],'rd')

            plt.plot(x_newa[0],x_newa[1],'bo')

        

        x_rand = random_point()



        #checks for validity of x_rand (wrt vertex1)

        while x_rand in vertex1 or not obstacle_free(x_rand):

            x_rand = random_point()



        x_newb,msg2 = extend(tree2,vertex2, x_rand)

        if msg2 == True:

            plt.plot(start[0],start[1],'gd')

            plt.plot(end[0],end[1],'rd')

            plt.plot(x_newb[0],x_newb[1],'mo')

            y,x = x_newb

            img[x][y][0] = 45

            img[x][y][1] = 86

            img[x][y][2] = 154

        

        # plt.pause(0.0001)

        # plt.plot(start[0],start[1],'go')

        # plt.plot(end[0],end[1],'ro')

        # plt.plot(x_newa[0],x_newa[1],'bo')

        # plt.plot(x_newb[0],x_newb[1],'mo')



        a,b,mssg=isgoal(vertex1,vertex2)

        if mssg:

            print("GOAL FOUND!!!")

            path=connect(tree1,vertex1,a,tree2,vertex2,b,img)

            return path,img

        

        

    print("Max iteration reached...")

    a,b,_=treeconnect(vertex1,vertex2)

    path=connect(tree1,vertex1,a,tree2,vertex2,b,img)

    print("Not enough iterations... Path can be bad")

    #returning till whatever tree1 has grown.



    return path,img     



path,img = RRTStar_Connect(start, end,img)

print("Final Path Coordinates: ",path)



for i in range(len(path)-1):

    cv2.line(img, path[i], path[i+1], (250,0,0), 2)



cv2.imshow("result",img)

cv2.waitKey(0)

##################################################___turle

goto.spawning(path)

x=goto.TurtleBot()

x.move2goal(path,'Image1.png')

  