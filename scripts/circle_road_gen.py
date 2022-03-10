from __future__ import print_function
import numpy as np

def main():

	radius = float(input("Radius in metres: "))
	angle = float(input("Angle in radians: "))

	print("\nRadius: {}\nAngle {}: ".format(radius, angle))

	theta = np.arange(0, 2*np.pi, angle)
	x = radius * np.cos(theta)
	y = radius * np.sin(theta)
	X = np.concatenate((X, X[0]))
	Y = np.concatenate((Y, Y[0]))

	generate_road_structure(X, Y)

def generate_road_structure(X, Y):

	for x, y in zip(X, Y):
		print("<point> {} {} 0 </point>".format(x, y))

if __name__ == "__main__":
    main()