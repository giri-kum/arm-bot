def pyramid(nlayers,spacing) #returns numpy array.
	coords=zeros(3,nlayers*(nlayers+1)/2)
	count=0
	for j in 0 to nlayers-1
		for i in 0 to nlayers-j-1
			x[count]=0+spacing*(i-((n-j)/2))
			y[count]=-8.95
			z[count]=offset+(blockheight*j) #define blockheight and offset 
			count=count+1
	return coords
