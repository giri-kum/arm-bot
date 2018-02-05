from numpy import *
def pyramid(nlayers,spacing): #returns numpy array.
	coords=zeros((3,nlayers*int((nlayers+1)/2)))
	count=0
	for j in range(0, nlayers):
		for i in range(0 , nlayers-j):
			print(nlayers-j-1)
			coords[0][count]=0+spacing*(i-((nlayers-j)/2))
			coords[1][count]=-8.95
			coords[2][count]=offset+(blockheight*j) #define blockheight and offset 
			count=count+1
	return coords
