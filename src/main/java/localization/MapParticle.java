package localization;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.text.ParseException;
import java.util.Random;
import java.lang.Math;
import org.apache.commons.math3.distribution.NormalDistribution; // will we have access to this?

import map.PolygonMap;

/*
 * @author - bhomberg
 */

public class MapParticle {

    // TODO: bumpSensorUpdate, fiducialSensorUpdate
    
    // we store weights as the negative log in order to get extra precision for low weight particles
    private double weight;
    private PolygonMap map;
    private double x;
    private double y;
    private double theta; // theta ranges 0 to 2pi
    private Random rand;

    // variance values -- should determine from experimentation
    // numbers are pulled out of a hat for now
    private final double SONAR_VARIANCE = .05;
    private final double FIDUCIAL_VARIANCE = .2;
    private final double X_VARIANCE = .1;
    private final double Y_VARIANCE = .1;
    private final double THETA_VARIANCE = .3;

    // constructor
    // takes in starting map file and total number of particles
    public MapParticle(String startMapFile, int numParticles) {
	try{
	    this.map = new PolygonMap(startMapFile);
	} catch (IOException e){
	    System.out.println("Error reading in map file!!");
	} catch (ParseException p) {
	    System.out.println("Error reading in map file!");
	}
	this.rand = new Random();
	this.x = rand.nextDouble() * map.worldRect.getWidth();
	this.y = rand.nextDouble() * map.worldRect.getHeight();
	this.theta = rand.nextDouble() * Math.PI * 2;
	// all particles start off with the same weight
	this.weight = -1 * Math.log( 1.0 / numParticles);
    }

    // performs a sensor update for this particle for bump sensors
    // bumpLoc is offset from local coordinates of robot
    // returns nothing, but particle weight changes
    public synchronized void bumpSensorUpdate(Point2D.Double bumpLoc){
    }

    // performs a sensor update for this particle for sonars
    // sonarMeasurements are ranges
    // returns nothing, but particle weight changes
    // eventually, here we'll think about adding new obstacles
    public synchronized void sonarSensorUpdate(double[] sonarMeasurements){
	double[] predicted = map.predictSonars(x, y, theta);
	double logprob = 0;
	for(int i=0; i<predicted.length; i++){
	    logprob += likelihood(sonarMeasurements[i], predicted[i], SONAR_VARIANCE);
	}
	weight = weight + logprob;
    }

    // performs a motion update for this particle
    // takes in delta values from odometry message
    // returns nothing, but particle position changes
    public synchronized void motionUpdate(double deltaX, double deltaY, double deltaTheta, double deltaTime){
	x += sample(deltaX, X_VARIANCE)*deltaTime;
	y += sample(deltaY, Y_VARIANCE)*deltaTime;
	theta += sample(deltaTheta, THETA_VARIANCE)*deltaTime;
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
