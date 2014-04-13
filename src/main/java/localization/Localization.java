package localization;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.buffer.ChannelBuffer;
import java.nio.ByteOrder;

import map.PolygonMap;
import rss_msgs.BumpMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.SonarMsg;
import rss_msgs.MapMsg;
import rss_msgs.PositionMsg;
import rss_msgs.FiducialMsg;
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
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;

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

    // Publishers
    protected Publisher<MapMsg> mapPub;
    protected Publisher<PositionMsg> posPub;

    protected final ArrayList<MapParticle> mapParticleList = new ArrayList<MapParticle>();

    protected final int MAX_PARTICLES = 200;

    protected boolean RESAMPLING = false;
    protected int RESAMPLING_FREQUENCY = 1000; // we should calibrate this -- my guess is we want to resample
                                               // about once a minute
    protected int RESAMPLING_COUNT = 0;

    protected double start_x;
    protected double start_y;
    protected double start_theta;
    protected long start_time;
    protected double curr_x;
    protected double curr_y;
    protected double curr_theta;
    protected long curr_time;

    protected boolean initialized;

    ExecutorService threadpool;

    @Override
    public synchronized void onStart(ConnectedNode node) {
     
	// Publishers
        mapPub = node.newPublisher("/loc/Map", "rss_msgs/MapMsg");
        posPub = node.newPublisher("/loc/Position", "rss_msgs/PositionMsg");


	// Subscribers
	bumpSub = node.newSubscriber("/sense/Bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg msg) {
                bumpSensorUpdate(msg);
            }
        });


	sonSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
	sonSub.addMessageListener(new MessageListener<SonarMsg>() {
            @Override
            public void onNewMessage(SonarMsg msg) {
                sonarSensorUpdate(msg);
            }
        });


	fidSub = node.newSubscriber("/vis/Fiducial", "rss_msgs/FiducialMsg");
	fidSub.addMessageListener(new MessageListener<FiducialMsg>() {
            @Override
            public void onNewMessage(FiducialMsg msg) {
                fiducialSensorUpdate(msg);
            }
        });


        odoSub = node.newSubscriber("/odo/Odometry", "rss_msgs/OdometryMsg");
	odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
            @Override
            public void onNewMessage(OdometryMsg msg) {
                currPosition(msg);
            }
        });

	//have not initialized motion yet
	initialized = false;

	//initialize threadpool
	threadpool = Executors.newCachedThreadPool();


	// initialize particles
        ParameterTree paramTree = node.getParameterTree();
        final String mapFile = paramTree.getString(node.resolveName("/loc/mapFileName"));

        for(int i=0; i<MAX_PARTICLES; i++){
            mapParticleList.add(new MapParticle(mapFile, MAX_PARTICLES));
        }
    }
	
    // performs sensor updates based on bump sensor values for all particles
    // updates the particle list, doesn't return anything
    public synchronized void bumpSensorUpdate(BumpMsg msg) {
        // TODO: sensor update
	// vaguely -- only send when true?

	motionUpdate();
	
	System.out.println("bumpSensorUpdate: " + (curr_x - start_x) + ", " + (curr_y - start_y) + ", " + (curr_theta - start_theta) + ", " + (curr_time - start_time) * MILLIS_TO_SECS);
	

	//insert actual bump update here

	resample();

	publishMap();
    }

    public void publishMap(){
	// Best weight is the minimum, since we're using negative log
        double minWeight = Double.POSITIVE_INFINITY;
        MapParticle bestParticle = null;
        for (MapParticle particle : mapParticleList) {
            if (particle.getWeight() < minWeight) {
                minWeight = particle.getWeight();
                bestParticle = particle;
            }
        }
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
	motionUpdate();

	/*for(MapParticle p : mapParticleList){
	    p.sonarSensorUpdate(msg.getSonarValues());
	    }*/

	resample();
	publishMap();
    }

    // performs sensor updates based on fiducial observation
    // updates the particle list, doesn't return anything
    public synchronized void fiducialSensorUpdate(FiducialMsg msg){
	// TODO: sensor update
	// can do similarly to sonar update
    }

    // performs motion updates based on odometry for all particles
    // updates the particle list, doesn't return anything
    public synchronized void motionUpdate() {
	if(initialized) {
	    for(int i=0; i<mapParticleList.size(); i++){
                final MapParticle particle = mapParticleList.get(i);
		threadpool.execute(
                    new Runnable() {
			@Override public void run() {
                            synchronized(particle) {
                                particle.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, (curr_time - start_time) * MILLIS_TO_SECS);
                            }
			}
		    });
            }
        }
	
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

	if(!initialized){
	    start_x = msg.getX();
	    start_y = msg.getY();
	    start_theta = msg.getTheta();
	    start_time = (long) msg.getTime();
	    initialized = true;
	}
    }    
    
    // renormalize particles
    // this induces error -- since we need to represent weights as actual probabilities -- so
    // we avoid calling this unless we need them to resample
    public synchronized void renormalize(){
	double sum = 0;
	
    }

    // resample particles
    public synchronized void resample(){
	if(RESAMPLING)
	    RESAMPLING_COUNT++;
	if(RESAMPLING_COUNT >= RESAMPLING_FREQUENCY){
	    renormalize();
	    RESAMPLING_COUNT = 0;
	    
	    //insert renormalizing code
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
