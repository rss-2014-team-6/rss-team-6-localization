package localization;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.buffer.ChannelBuffer;
import java.nio.ByteOrder;

import map.PolygonMap;
import gui_msgs.GUIParticleCloudMsg;
import gui_msgs.PointDataMsg;
import rss_msgs.BumpMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.SonarMsg;
import rss_msgs.MapMsg;
import rss_msgs.PositionMsg;
import rss_msgs.FiducialMsg;
import rss_msgs.InitializedMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

import java.lang.Runtime;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicInteger;

public class Localization implements NodeMain{

    // Conversion between ms and secs
    private static final double MILLIS_TO_SECS = 0.001;

    // TODO: bump and fiducial updates
    // finish renormalizing and resampling

    // Subscribers
    protected Subscriber<BumpMsg> bumpSub;
    protected Subscriber<OdometryMsg> odoSub;
    protected Subscriber<SonarMsg> sonSub;
    protected Subscriber<FiducialMsg> fidSub;
    protected Subscriber<InitializedMsg> initSub;

    // Publishers
    protected Publisher<MapMsg> mapPub;
    protected Publisher<PositionMsg> posPub;
    protected Publisher<GUIParticleCloudMsg> guiCloudPub;
    protected Publisher<PointDataMsg> guiPointDataPub;

    protected ArrayList<MapParticle> mapParticleList = new ArrayList<MapParticle>();

    protected static final int MAX_PARTICLES = 1000;
    protected static final double CONFIDENCE_THRESH = 0.30;

    protected boolean RESAMPLING = true;
    // Heuristic to track roughly how much variance in particle 
    // position has been introduced by motion.
    protected double resamplingCount = 0.0;
    protected double RESAMPLING_FREQUENCY = 5; 
    /**
     * How many of the resampled particles are chosen based on
     * existing weights (the rest are resampled new).
     */
    protected double RESAMPLING_FRACTION = .5;
    /**
     * Particles above this probability and kept with their original
     * weight. Rest are resampled.
     */
    protected double RESAMPLING_KEEP_PROB_THRESH = 0.01;
    /**
     * Maximum possible noise to add to each coord for resampled particles.
     */
    protected double RESAMPLING_NOISE = 0.1;

    protected MapParticle prevBestParticle;

    protected double start_x;
    protected double start_y;
    protected double start_theta;
    protected long start_time;
    protected double curr_x;
    protected double curr_y;
    protected double curr_theta;
    protected long curr_time;

    protected boolean motion_initialized = false;
    protected boolean state_initialized = false;

    private Random rand;

    private AtomicInteger counter = new AtomicInteger(0);

    private String globalMapFile;

    ExecutorService threadpool;

    @Override
    public synchronized void onStart(ConnectedNode node) {
     
	// Publishers
        mapPub = node.newPublisher("/loc/Map", "rss_msgs/MapMsg");
        posPub = node.newPublisher("/loc/Position", "rss_msgs/PositionMsg");
        guiCloudPub = node.newPublisher("/gui/ParticleCloud", "gui_msgs/GUIParticleCloudMsg");
        // Dummy publisher so we can create new messages of this type (yayyy rosjava)
        guiPointDataPub = node.newPublisher("/dummy/PointData", "gui_msgs/PointDataMsg");


	// Subscribers
	bumpSub = node.newSubscriber("/sense/Bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg msg) {
                if(state_initialized)
		    bumpSensorUpdate(msg);
            }
        });


	sonSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
	sonSub.addMessageListener(new MessageListener<SonarMsg>() {
            @Override
            public void onNewMessage(SonarMsg msg) {
                System.out.println("Sonar msg, init: " + state_initialized);
                if(state_initialized)
                    sonarSensorUpdate(msg);
            }
        });


	fidSub = node.newSubscriber("/vision/FiducialLocation", "rss_msgs/FiducialMsg");
	fidSub.addMessageListener(new MessageListener<FiducialMsg>() {
            @Override
            public void onNewMessage(FiducialMsg msg) {
                if(state_initialized)
                    fiducialSensorUpdate(msg);
            }
        });


        odoSub = node.newSubscriber("/odo/Odometry", "rss_msgs/OdometryMsg");
	odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
            @Override
            public void onNewMessage(OdometryMsg msg) {
                if(state_initialized)
		    currPosition(msg);
            }
        });

	initSub = node.newSubscriber("/state/Initialized", "rss_msgs/InitializedMsg");
	initSub.addMessageListener(new MessageListener<InitializedMsg>() {
            @Override
            public void onNewMessage(InitializedMsg msg) {
                state_initialized = msg.getInitialized();
            }
        });

	rand = new Random();

	//initialize threadpool
	threadpool = Executors.newCachedThreadPool();


	// initialize particles
        ParameterTree paramTree = node.getParameterTree();
        final String mapFile = paramTree.getString(node.resolveName("/loc/mapFileName"));
	globalMapFile = mapFile;

        final double particleWeight = -1 * Math.log(1.0 / MAX_PARTICLES);
        for(int i=0; i<MAX_PARTICLES; i++){
            mapParticleList.add(new MapParticle(mapFile, particleWeight, i));
        }
    }
	
    // performs sensor updates based on bump sensor values for all particles
    // updates the particle list, doesn't return anything
    public synchronized void bumpSensorUpdate(BumpMsg msg) {
	    
	//motionUpdate();
	
	//System.out.println("bumpSensorUpdate: " + (curr_x - start_x) + ", " + (curr_y - start_y) + ", " + (curr_theta - start_theta) + ", " + (curr_time - start_time) * MILLIS_TO_SECS);
	
	
	//insert actual bump update here
	
	//resample();
	
	//publishMap();
    }

    private void drawParticleCloud() {
        double[] weights = new double[mapParticleList.size()];
        List<PointDataMsg> points = new ArrayList<PointDataMsg>();
        for (int i = 0; i < mapParticleList.size(); i++) {
            MapParticle particle = mapParticleList.get(i);
            PointDataMsg pointData = guiPointDataPub.newMessage();
            pointData.setX(particle.getX());
            pointData.setY(particle.getY());
            points.add(pointData);
            weights[i] = particle.getWeight();
        }
        GUIParticleCloudMsg cloudMsg = guiCloudPub.newMessage();
        cloudMsg.setPoints(points);
        cloudMsg.setWeights(weights);
        guiCloudPub.publish(cloudMsg);
    }

    private void publishMap(){
	// Best weight is the minimum, since we're using negative log
        double minWeight = Double.POSITIVE_INFINITY;
        MapParticle bestParticle = null;
        for (MapParticle particle : mapParticleList) {
            double weight = particle.getWeight();
	    //if(particle.getID() == 100)
	    //	System.out.println("ID: " + particle.getID() + " wt: " + weight);
            if (weight < minWeight) {
                minWeight = particle.getWeight();
                bestParticle = particle;
            }
        }
        double totalWeight = 0;
        for (MapParticle particle : mapParticleList) {
            totalWeight += Math.exp(-1 * (particle.getWeight() - minWeight));
        }
        double confidence = 1.0 / totalWeight;
        // No particles stand out enough (probably after resampling)
        if (confidence < CONFIDENCE_THRESH && prevBestParticle != null) {
            // Go with our previous most confident particle, if there is one
            bestParticle = prevBestParticle;
            //System.out.println("Selecting prev best particle");
        }
        else if (confidence > CONFIDENCE_THRESH) {
            prevBestParticle = new MapParticle(bestParticle, minWeight, bestParticle.getID());
            //System.out.println("Particle is good conf");
        }
        // TEST


        double[] predicted = bestParticle.getMap().predictSonars(
            bestParticle.getX(), bestParticle.getY(), bestParticle.getTheta());
        //System.out.println("Predicted vals: " + predicted[0] + ", " + predicted[1] + ", " + predicted[2] + ", " + predicted[3]);
	System.out.println("Particle ID: " + bestParticle.getID() + ", weight: " + minWeight + ", prob: " + Math.exp(-1*minWeight) + 
                           "\n totalweight: " + totalWeight + ", conf: " + confidence
			   + "\n pos: " + bestParticle.getPosition());
	/*if(((Double)totalWeight).isNaN()){
	    // for(MapParticle p : mapParticleList)
	    //System.out.println("ID: " + p.getID() + " wt: " + p.getWeight());
	}
        if(totalWeight == 1)
	    for(MapParticle p : mapParticleList)
	    System.out.println("ID: " + p.getID() + " wt: " + p.getWeight());*/
	// Serialize and publish the map
        PolygonMap map = bestParticle.getMap();
        ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
        try {
            ObjectOutputStream stream = new ObjectOutputStream(byteStream);
            stream.writeObject(map);
        }
        catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("IOException on serializing map");
        }
        MapMsg mapMsg = mapPub.newMessage();
	byte[] ba = byteStream.toByteArray();
	// ChannelBuffer MUST be little endian
	ChannelBuffer buffer = ChannelBuffers.buffer(ByteOrder.LITTLE_ENDIAN, ba.length);
	buffer.writeBytes(ba);
        mapMsg.setSerializedMap(buffer);
        mapPub.publish(mapMsg);
        // Publish the position estimate

        PositionMsg posMsg = posPub.newMessage();
        posMsg.setX(bestParticle.getX());
        posMsg.setY(bestParticle.getY());
        posMsg.setTheta(bestParticle.getTheta());
        posPub.publish(posMsg);
    }

    // performs sensor updates based on sonar values for all particles
    // updates the particle list, doesn't return anything
    public synchronized void sonarSensorUpdate(SonarMsg msg) {
	//System.out.println("Sonar timestamp: " + msg.getTime() + " Odometry timestamp: " + curr_time);
	//System.out.println("DIFFFFF: " + (msg.getTime() - curr_time));

	if(Math.abs(msg.getTime() - curr_time) < 50){
	    counter.incrementAndGet(); // add one before doing updates as a chunk

	    motionUpdate();

	    final double[] vals = msg.getSonarValues();

	    if(motion_initialized) {
		for(int i=0; i<mapParticleList.size(); i++){
		    counter.incrementAndGet();
		    final MapParticle particle = mapParticleList.get(i);
		    threadpool.execute(
				       new Runnable() {
					   @Override public void run() {
					       synchronized(particle) {
						   particle.sonarSensorUpdate(vals);
						   counter.decrementAndGet();
					       }
					   }
				       });
		}
	    }

	    counter.decrementAndGet(); // decrement after finishing all updates
	}else{
	    //System.out.println("T\nT\nT\nDIFF TOO BIG! IGNORING SONAR UPDATE!!\nT\nT\nT");
	}

        resample();

        publishMap();
        drawParticleCloud();
    }

    // performs sensor updates based on fiducial observation
    // updates the particle list, doesn't return anything
    public synchronized void fiducialSensorUpdate(FiducialMsg msg){
	if(Math.abs(msg.getTime() - curr_time) < 50){
	    counter.incrementAndGet(); // add one before doing updates as a chunk

	    motionUpdate();

	    final double range = msg.getRange();
	    final double bearing = msg.getBearing();
	    final int top = (int)msg.getTop();
	    final int bottom = (int)msg.getBottom();

	    if(motion_initialized) {
		for(int i=0; i<mapParticleList.size(); i++){
		    counter.incrementAndGet();
		    final MapParticle particle = mapParticleList.get(i);
		    threadpool.execute(
				       new Runnable() {
					   @Override public void run() {
					       synchronized(particle) {
						   particle.fiducialSensorUpdate(range, bearing, top, bottom);
						   counter.decrementAndGet();
					       }
					   }
				       });
		}
	    }

	    counter.decrementAndGet(); // decrement after finishing all updates
	}else{
	    //System.out.println("T\nT\nT\nDIFF TOO BIG! IGNORING SONAR UPDATE!!\nT\nT\nT");
	}

        resample();

        publishMap();
        drawParticleCloud();
    }

    // performs motion updates based on odometry for all particles
    // updates the particle list, doesn't return anything
    public synchronized void motionUpdate() {
        //System.out.println("motionUpdate: queued -- " + counter.get());
	if(motion_initialized) {
            //System.out.println("queueing");
	    for(int i=0; i<mapParticleList.size(); i++){
		counter.incrementAndGet();
                final MapParticle particle = mapParticleList.get(i);
		threadpool.execute(
                    new Runnable() {
			@Override public void run() {
                            synchronized(particle) {
                                particle.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, (curr_time - start_time) * MILLIS_TO_SECS, start_theta);
				counter.decrementAndGet();
			    }
			}
		    });
            }
            // Also update our prev best particle, in case we need to fall back on it
            if (prevBestParticle != null) {
                final MapParticle prevBestParticleFinal = prevBestParticle;
                threadpool.execute(
                    new Runnable() {
                        @Override public void run() {
                            synchronized(prevBestParticleFinal) {
                                prevBestParticleFinal.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, (curr_time - start_time) * MILLIS_TO_SECS, start_theta);
                            }
                        }
                    });
            }
        }
	
        updateResamplingCount();

	start_x = curr_x;
	start_y = curr_y;
	start_theta = curr_theta;
	start_time = curr_time;
    }

    public synchronized void currPosition(OdometryMsg msg) {
	curr_x = msg.getX();
	curr_y = msg.getY();
	curr_theta = msg.getTheta();
	curr_time = (long) msg.getTime(); // seems to build fine on the eeepc but complains on my comp without this

	if(!motion_initialized){
	    start_x = msg.getX();
	    start_y = msg.getY();
	    start_theta = msg.getTheta();
	    start_time = (long) msg.getTime();
	    motion_initialized = true;
	}
    }    

    /**
     * Update our resampling count to account for variance added by motion,
     * plus a baseline delta time component.
     * Expects start_x, start_y, start_theta, start_time, curr_x, curr_y,
     * curr_theta, and curr_time to be set.
     */
    private void updateResamplingCount() {
        final double THETA_COEFF = 5.0;
        final double DIST_COEFF = 1.0;
        final double TIME_COEFF = 0.001;
        double deltaX = curr_x - start_x;
        double deltaY = curr_y - start_y;
        double deltaTheta = curr_theta - start_theta;
        double deltaTime = curr_time - start_time;
        deltaTheta = deltaTheta % (Math.PI*2);
        resamplingCount +=
            DIST_COEFF * Math.sqrt(Math.pow(curr_x-start_x, 2)
                                   + Math.pow(curr_y-start_y, 2)) +
            THETA_COEFF * Math.abs(deltaTheta) +
            TIME_COEFF * deltaTime;
    }
    
    // "renormalize" particles, such that best particle has prob 1
    // this induces error -- since we need to represent weights as actual probabilities -- so
    // we avoid calling this unless we need them to resample
    private synchronized void renormalize(){
	double sum = 0;

	while(counter.get() > 0);

        // First just scale probabilities such that largest is 1
        // (in terms of log, this is just subtracting the minweight)
        double minWeight = Double.POSITIVE_INFINITY;
        for(MapParticle p : mapParticleList) {
            if (p.getWeight() < minWeight) {
                minWeight = p.getWeight();
            }
        }
	//if(minWeight == Double.POSITIVE_INFINITY)
	//    System.out.println("X\nX\nX\n POSITIVE INFINITY CURSE YOU TEJ \nX\nX\nX");

	for(MapParticle p : mapParticleList) {
            double w = p.getWeight();
            p.setWeight(w - minWeight);
	    sum += Math.exp(-1*p.getWeight());
        }

	for(int i=0; i<mapParticleList.size(); i++){
	    double w = mapParticleList.get(i).getWeight();
	    System.out.println("ID: " + mapParticleList.get(i).getID() + " sum: " + sum + " new wt: " + w + " prob: " + (w + -1 * Math.log( 1 / sum )));

	    //the following two lines should be equivalent...
	    //mapParticleList.get(i).setWeight(-1 * Math.log( Math.exp(-1*w) / sum ));
	    mapParticleList.get(i).setWeight(w + -1 * Math.log( 1 / sum ));
	}
    }

    // resample particles
    private synchronized void resample(){
	if(RESAMPLING && resamplingCount >= RESAMPLING_FREQUENCY){
            System.out.println("Resampling!");
	    renormalize();
            
	    resamplingCount = 0.0;

	    ArrayList<MapParticle> newParticleList = new ArrayList<MapParticle>();

            int index = 0;
            double totalProb = 0;
            for(MapParticle p : mapParticleList) {
                double weight = p.getWeight();
                if (Math.exp(-1*weight) >= RESAMPLING_KEEP_PROB_THRESH) {
                    newParticleList.add(new MapParticle(p, weight, index));
                    index++;
                    totalProb += Math.exp(-1*weight);
                }
            }
            int kept = index+1;
            System.out.println(kept + " total particles kept.");
	    // some fraction of the particles are resampled, others are draw new
	    for(int i = 0; i < (MAX_PARTICLES-kept); i++) {
                double val = rand.nextDouble();
                double temp=0;
                int j;
                // Cycle through particles until we pass the randomly selected
                // val -- more probability of landing on higher weight particles.
                for(j=0; j<mapParticleList.size(); j++){
                    temp += Math.exp(-1*mapParticleList.get(j).getWeight());
                    if(temp > val)
                        break;
                }
                if (j >= mapParticleList.size()) {
                    j = mapParticleList.size()-1;
                }
                // Duplicate the chosen particle at index with noise.
                double newWeight = -1 * Math.log((1-totalProb) / MAX_PARTICLES);
                newParticleList.add(
				    // the below statement used to have RESAMPLING_NOISE
                    new MapParticle(mapParticleList.get(j), newWeight, index, 0));
                index++;
                if (index >= MAX_PARTICLES * RESAMPLING_FRACTION) break;
	    }

	    final String mapFile = globalMapFile;

	    // the rest of the particles are made new
	    for(int i = index; i<MAX_PARTICLES; i++){
                double newWeight = -1 * Math.log((1-totalProb) / MAX_PARTICLES);
		newParticleList.add(new MapParticle(mapFile, newWeight, i));
	    }

            // Replace the old map particle list
            mapParticleList = newParticleList;
	}
    }

    @Override
    public void onShutdown(Node node) {
        if (node != null) {
            node.shutdown();
        }
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/localization");
    }

    @Override
    public void onError(Node node, Throwable error) {
    }
}
