#(448.442491, 462.282784) (457.556044, 462.282784) (460.404396, 456.014652) (460.404396, 445.570696)
#p1x = 448.442491
#p1y = 462.282784
#p2x = 457.556044
#p2y = 462.282784
#p3x = 460.404396
#p3y = 456.014652
#p4x = 460.404396
#p4y = 445.570696
#mu = 0.01

def getpoint(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, mu):
	n1 = 0
	n13 = 0
	mu3 = 0
	p = 0

	n1 = 1 - mu
	n13 = n1 * n1 * n1
	mu3 = mu * mu * mu
	px = n13*p1x + 3*mu*n1*n1*p2x + 3*mu*mu*n1*p3x + mu3*p4x;
	py = n13*p1y + 3*mu*n1*n1*p2y + 3*mu*mu*n1*p3y + mu3*p4y;

	return ([px,py])

def getlines(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, x):
	z = 0
	c = 1.00 / x
	z = c
	points = []
	print z
	while (z < 1):
		points.append(getpoint(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, z))
		z = z + c
	print points

getlines(448.442491, 462.282784, 457.556044, 462.282784,
 460.404396, 456.014652, 460.404396, 445.570696, 100)
