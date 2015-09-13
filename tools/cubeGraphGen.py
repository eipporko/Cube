#!/usr/bin/python

import sys, getopt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.mlab import griddata
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np
import dataset


def openReadFile():
	global inputfile

	try:
		inputfile = open(inputfilename, 'r')
	except IOError:
		print 'cannot open ', inputfilename
		sys.exit(2)


def createDatabase():
	global db
	global data
	db = dataset.connect('sqlite:///:memory:')
	data = db['data']	


def processTime(line):
	global resolution
	global renderName
	global blendingName
	global fxaa
	global numOfPoints
	global numOfLights

	fps = 1./float(line)
	data.insert(dict(resolution=resolution, render=renderName, blending=blendingName, fxaa=fxaa, points=numOfPoints, lights=numOfLights, fps=fps))


def processHeader7(line):
	global resolution
	global renderName
	global blendingName
	global fxaa
	global numOfPoints 
	global numOfLights

	split = line.split(" | ")
	resolution = split[6]
	renderName = split[1]
	blendingName = split[2]
	fxaa = False
	numOfPoints = int(split[4].split( )[0])
	numOfLights = int(split[5].split( )[0])


def processHeader8(line):
	global resolution
	global renderName
	global blendingName
	global fxaa
	global numOfPoints
	global numOfLights

	split = line.split(" | ")
	resolution = split[7]
	renderName = split[1]
	blendingName = split[2]
	fxaa = True
	numOfPoints = int(split[5].split( )[0])
	numOfLights = int(split[6].split( )[0])


def doDefault(line):
	return



def drawRasterizationComparative():
	global db
	global verbose

	if (verbose == True):
		result = db.query('SELECT resolution, render, blending, points, fxaa, lights, AVG(fps) as avrg FROM data WHERE blending = \'Flat Shading\' AND lights = 0 AND fxaa = 0 GROUP BY resolution, render, points')
		print 'RASTERIZATION COMPARATIVE  (FXAA=FALSE, LIGHTS=0 BLEND=FLAT)'
		for row in result:
			print(row['resolution'], row['render'], row['blending'], row['fxaa'], row['points'], row['lights'], row['avrg'] )
		print '\n'


	resolutionList = db.query('SELECT resolution FROM data WHERE blending = \'Flat Shading\' AND lights = 0 AND fxaa = 0 GROUP BY resolution')
	for resolution in resolutionList:
		#do plot by resolution
		renderList = db.query('SELECT render FROM data WHERE blending = \'Flat Shading\' AND lights = 0 AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' GROUP BY render')
		for render in renderList:
			pointsArray = []
			avrgArray = []
			pointList = db.query('SELECT points FROM data WHERE blending = \'Flat Shading\' AND lights = 0 AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' AND render=\''+ render['render'] +'\'GROUP BY points')
			for points in pointList:
				pointsArray.append(points['points'])
				fpsList = db.query('SELECT AVG(fps) as avrg FROM data WHERE blending = \'Flat Shading\' AND lights = 0 AND fxaa = 0 AND resolution =\'' + resolution['resolution'] + '\' AND render=\''+ render['render'] +'\' AND points = ' + str(points['points']))
				for fps in fpsList:
					avrgArray.append(fps['avrg'])

			plt.plot(pointsArray, avrgArray, 'o-', label=render['render'], linewidth=2)
		plt.legend()
		plt.ylabel('FPS')
		plt.xlabel('Number of Points')
		plt.title('Rasterization Comparative ('+resolution['resolution']+')')
		plt.grid(True)
		plt.show()

def drawBlendingComparative():
	global db
	global verbose
	
	if (verbose == True):
		result = db.query('SELECT resolution, render, blending, points, fxaa, lights, AVG(fps) as avrg FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = 0 AND fxaa = 0 GROUP BY resolution, blending, points')
		print 'BLENDING COMPARATIVE  (FXAA=FALSE, LIGHTS=0 RASTER=PERSPECTIVE CORRECT)'
		for row in result:
			print(row['resolution'], row['render'], row['blending'], row['fxaa'], row['points'], row['lights'], row['avrg'] )
		print '\n'


	resolutionList = db.query('SELECT resolution FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = 0 AND fxaa = 0 GROUP BY resolution')
	for resolution in resolutionList:
		#do plot by resolution
		blendingList = db.query('SELECT blending FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = 0 AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' GROUP BY blending')
		for blend in blendingList:
			pointsArray = []
			avrgArray = []
			pointList = db.query('SELECT points FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = 0 AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' AND blending=\''+ blend['blending'] +'\'GROUP BY points')
			for points in pointList:
				pointsArray.append(points['points'])
				fpsList = db.query('SELECT AVG(fps) as avrg FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = 0 AND fxaa = 0 AND resolution =\'' + resolution['resolution'] + '\' AND blending=\''+ blend['blending'] +'\' AND points = ' + str(points['points']))
				for fps in fpsList:
					avrgArray.append(fps['avrg'])

			plt.plot(pointsArray, avrgArray, 'o-', label=blend['blending'], linewidth=2)

		plt.legend()
		plt.ylabel('FPS')
		plt.xlabel('Number of Points')
		plt.title('Blending Comparative ('+resolution['resolution']+')')
		plt.grid(True)
		plt.show()



def drawLightningComparative():
	global db
	global verbose
	
	if (verbose == True):
		result = db.query('SELECT resolution, render, blending, points, fxaa, lights, AVG(fps) as avrg FROM data WHERE render = \'Perspective Correct Rasterization\' AND fxaa = 0 GROUP BY resolution, blending, lights, points')
		print 'LIGHTNING COMPARATIVE  (FXAA=FALSE, RASTER=PERSPECTIVE CORRECT)'
		for row in result:
			print(row['resolution'], row['render'], row['blending'], row['fxaa'], row['points'], row['lights'], row['avrg'] )
		print '\n'

	resolutionList = db.query('SELECT resolution FROM data WHERE render = \'Perspective Correct Rasterization\' AND fxaa = 0 GROUP BY resolution')
	for resolution in resolutionList:
		blendingList = db.query('SELECT blending FROM data WHERE render = \'Perspective Correct Rasterization\' AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' GROUP BY blending')
		for blend in blendingList:
			#do plot by blending
			fig = plt.figure()
			ax = fig.gca(projection='3d')
			pointsArray = []
			lightsArray = []
			avrgArray = []
			pointList = db.query('SELECT points FROM data WHERE render = \'Perspective Correct Rasterization\'AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' AND blending=\''+ blend['blending'] +'\'GROUP BY points')
			for points in pointList:
				lightsList = db.query('SELECT lights FROM data WHERE render = \'Perspective Correct Rasterization\'AND fxaa = 0 AND resolution =\''+resolution['resolution']+'\' AND blending=\''+ blend['blending'] +'\' AND points=' + str(points['points']) + ' GROUP BY lights')
				for lights in lightsList:
					pointsArray.append(points['points'])
					lightsArray.append(lights['lights'])
					fpsList = db.query('SELECT AVG(fps) as avrg FROM data WHERE render = \'Perspective Correct Rasterization\' AND lights = ' + str(lights['lights'])+ ' AND fxaa = 0 AND resolution =\'' + resolution['resolution'] + '\' AND blending=\''+ blend['blending'] +'\' AND points = ' + str(points['points']))
					for fps in fpsList:
						avrgArray.append(fps['avrg'])
			
			if ( min(pointsArray) != max(pointsArray) and min(lightsArray) != max(lightsArray) ):
				AD = np.linspace(min(pointsArray), max(pointsArray))
				MD = np.linspace(min(lightsArray), max(lightsArray))
				X,Y = np.meshgrid(AD, MD)
				Z = griddata(pointsArray,lightsArray,avrgArray,AD,MD,'linear')
				surf = ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)
				ax.set_xlabel('Number of Points')
				ax.set_ylabel('Number of Lights')
				ax.set_zlabel('FPS') 
				plt.title('Lightning Comparative in ' + blend['blending'] + ' ('+resolution['resolution']+')')
				plt.grid(True)
				fig.colorbar(surf, shrink=0.5, aspect=10)
				plt.show()
			else: 
				print "ERROR: Input is less than 3-dimensional (same numOfPoints or numOfLights)"





def drawPlots():
	drawRasterizationComparative()
	drawBlendingComparative()
	drawLightningComparative()



def getData():
	global data

	openReadFile()
	createDatabase()

	allLines = inputfile.readlines()
	for line in allLines:
		line = line.strip()			#remove \n
		numOfitems = len(line.split(" | "))
		switcher = {
			1 : processTime,
			7 : processHeader7,
			8 : processHeader8
		}
		func = switcher.get(numOfitems, doDefault)
		func(line)

	inputfile.close()
	drawPlots()

def processInput():
	getData()


#print help usage
def printHelp():
	print 'CUBE GRAPH GENERATOR'
	print ''
	print 'USAGE: cubeGraphGen.py [options] logFile'
	print ''
	print 'OPTIONS:'
	print '    -h        Display available options'
	print '    --help    Display available options'
	print '    -v        Run in verbose mode'
	print 'May the Force be with you :)'


def main(argv):

	#define global variables
	global inputfilename
	global db
	global data
	global resolution
	global renderName
	global blendingName
	global fxaa
	global numOfPoints
	global numOfLights
	global verbose
	
	#init global variables
	inputfilename = ''
	resolution = ''
	renderName = ''
	blendingName = ''
	fxaa = False
	numOfPoints = 0
	numOfLights = 0
	verbose = False

	#parse argv
	try:
		opts, args = getopt.getopt(argv, "h:v", ["help"])
	except getopt.GetoptError:
		printHelp()
		sys.exit(2)

	for o, a in opts:
		if o == "-v":
			verbose = True
		elif o in ("-h", "--help"):
			printHelp()
			sys.exit()
		else:
			assert False, "unhandled option"

	#get input file name from args
	if len(args) == 0:
		printHelp()
		sys.exit(2)
	else:
		inputfilename = args[0]

	processInput()

if __name__ == "__main__":
	main(sys.argv[1:])