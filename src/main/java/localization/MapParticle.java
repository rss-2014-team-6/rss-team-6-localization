package localization;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.text.ParseException;
import java.util.Random;
import java.lang.Math;
import org.apache.commons.math3.distribution.NormalDistribution; // will we have access to this?

import map.PolygonMap;

public class MapParticle implements Cloneable{

    // TODO: bumpSensorUpdate, fiducialSensorUpdate
    
    // we store weights as the negative log in order to get extra precision for low weight particles
    private double weight;
    private PolygonMap map;
    private double x;
    private double y;
    private double theta; // theta ranges 0 to 2pi
    private Random rand;
    private int id;

    // variance values -- should determine from experimentation
    // numbers are pulled out of a hat for now
    private final double SONAR_VARIANCE = .2;
    private final double FIDUCIAL_VARIANCE = .2;
    private final double X_VARIANCE = .001;
    private final double Y_VARIANCE = .001;
    private final double THETA_VARIANCE = .03;
    private final double MOTION_THRESHOLD = .001;

    private final double PROBABILITY_OF_BUMP_IF_IN_POSITION = .9;
    private final double PROBABILITY_OF_BUMP_IF_NOT_IN_POSITION = .02;
    private final double PROBABILITY_OF_FALSE_SONAR = .2; //pulled out of a hat!

    private final double SONAR_MAX_DIST = 1.2; //check for real value
    private final double SONAR_MIN_DIST = .02;

    private final double OUT_OF_BOUND_PENALTY = .001; 


    // constructor
    // takes in starting map file and total number of particles
    public MapParticle(String startMapFile, int numParticles, int id) {
	try{
	    this.map = new PolygonMap(startMapFile);
	} catch (IOException e){
	    System.out.println("Error reading in map file!!");
	} catch (ParseException p) {
	    System.out.println("Error reading in map file!");
	}
	this.rand = new Random();
	double tx = rand.nextDouble() * map.worldRect.getWidth() + map.worldRect.getX();
	double ty = rand.nextDouble() * map.worldRect.getHeight() + map.worldRect.getY();
	while(!map.isValid(tx, ty)){
	    tx = rand.nextDouble() * map.worldRect.getWidth() + map.worldRect.getX();
	    ty = rand.nextDouble() * map.worldRect.getHeight() + map.worldRect.getY();
	}
	this.x = tx;
	this.y = ty;
	this.theta = rand.nextDouble() * Math.PI * 2;
	// all particles start off with the same weight
	this.weight = -1 * Math.log( 1.0 / numParticles);
	this.id = id;
	System.out.println("MAP PARTICLE " + id + ": x: " + this.x + ", y: " + this.y);
   }

    public MapParticle(MapParticle mp, int numParticles, int id){
	this.weight = -1 * Math.log( 1.0 / numParticles);
	this.x = mp.getX();
	this.y = mp.getY();
	this.theta = mp.getTheta();
	rand = new Random();
	this.id = id;
	this.map = new PolygonMap(mp.getMap());
    }

    // performs a sensor update for this particle for bump sensors
    // bumpLoc is offset from local coordinates of robot
    // returns nothing, but particle weight changes
    public synchronized void bumpSensorUpdate(int bumpID){
	//reasoning behind bump update:
	// most of the time, we don't expect to get a bump sensor reading
	// there's a fairly high variability when exactly we get a bump update based on angle/etc
	// we only perform bump updates if we're actually bumping something
	// Localization passes in a location where there's a bump
	
	if(map.withinBumpThreshold(x, y, theta, bumpID)){
	    weight += -1 * Math.log(PROBABILITY_OF_BUMP_IF_IN_POSITION);
	}
	else{
	    weight += -1 * Math.log(PROBABILITY_OF_BUMP_IF_NOT_IN_POSITION);
	}
	
    }

    // performs a sensor update for this particle for sonars
    // sonarMeasurements are ranges
    // returns nothing, but particle weight changes
    // eventually, here we'll think about adding new obstacles
    public synchronized void sonarSensorUpdate(double[] sonarMeasurements){
	double[] predicted = map.predictSonars(x, y, theta);
	//System.out.println("Predicted vals: " + predicted[0] + ", " + predicted[1]);
	double logprob = 0;
	for(int i=0; i<predicted.length; i++){
	    if(sonarMeasurements[i] > SONAR_MIN_DIST && sonarMeasurements[i] < SONAR_MAX_DIST){
		if(predicted[i] != -1)
		    logprob += likelihood(sonarMeasurements[i], predicted[i], SONAR_VARIANCE);
		else
		    logprob += -1 * Math.log(PROBABILITY_OF_FALSE_SONAR);
	    }
	}
	weight = weight + logprob;
	//System.out.println("\t Particle " + id + "weight: " + weight);
    }

    // performs a motion update for this particle
    // takes in delta values from odometry message
    // as well as a reference theta to offset the
    // deltas to match internal theta estimate
    // returns nothing, but particle position changes
    public synchronized void motionUpdate(double deltaX, double deltaY, double deltaTheta, double deltaTime, double odoRefTheta) {
	// Rotate deltaX and deltaY by (theta - odoRefTheta)
	if(deltaX > MOTION_THRESHOLD || deltaY > MOTION_THRESHOLD || deltaTheta > MOTION_THRESHOLD){
	    double rotTheta = theta - odoRefTheta;
	    double realDeltaX = deltaX*Math.cos(rotTheta) - deltaY*Math.sin(rotTheta);
	    double realDeltaY = deltaX*Math.sin(rotTheta) + deltaY*Math.cos(rotTheta);
	    double dist = Math.sqrt(Math.pow(realDeltaX, 2) + Math.pow(realDeltaY, 2));
	    x += sample(realDeltaX, X_VARIANCE*dist);
	    y += sample(realDeltaY, Y_VARIANCE*dist);
	    theta += sample(deltaTheta, THETA_VARIANCE*dist);
	}

	if(!map.isValid(x,y))
	    weight += -1 * Math.log(OUT_OF_BOUND_PENALTY);
    }

    // returns the weight
    public double getWeight(){
	return weight;
    }

    // set the weight -- used for normalization
    public synchronized void setWeight(double w){
	weight = w;
    }

    // get the current map estimate for this particle
    public PolygonMap getMap(){
        return map;
    }

    // get the x coord
    public double getX() {
        return x;
    }

    // get the y coord
    public double getY() {
        return y;
    }

    // get the theta coord
    public double getTheta() {
        return theta;
    }

    public Point2D.Double getPosition(){
	return new Point2D.Double(x, y);
    }

    public double getID(){
	return id;
    }

    // performs a Gaussian sample
    // takes in a mean value and a variance
    // returns a random sample
    private double sample(double mean, double variance){
	return mean + Math.sqrt(variance) * rand.nextGaussian(); // stdv is sqrt of var
    }

    // finds the likelihood of a Gaussian sample
    // takes in value, mean, variance
    // returns a negative log probability
    private double likelihood(double value, double mean, double variance){
	// if this is too slow or something we can switch to having a standard normal distribution and
	// making the necessary compensations
	NormalDistribution normal = new NormalDistribution(mean, Math.sqrt(variance)); // stdv is sqrt of var
	if(value >  mean)
	    value = mean - (value - mean); // move value to equally far away but smaller than mean
	double p = normal.cumulativeProbability(value);
	return -1 * Math.log(p);
    }


}
