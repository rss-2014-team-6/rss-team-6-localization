package localization;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import org.jboss.netty.buffer.ChannelBuffers;

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

/*
 * @author - bhomberg 
 */

public class Localization implements NodeMain{

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

    protected ArrayList<MapParticle> mapParticleList;

    protected final int MAX_PARTICLES = 200;

    protected boolean RESAMPLING = false;
    protected int RESAMPLING_FREQUENCY = 1000; // we should calibrate this -- my guess is we want to resample
                                               // about once a minute
    protected int RESAMPLING_COUNT = 0;

    protected double start_x;
    protected double start_y;
    protected double start_theta;
    protected double start_time;
    protected double curr_x;
    protected double curr_y;
    protected double curr_theta;
    protected double curr_time;

    protected boolean initialized;

    @Override
    public void onStart(ConnectedNode node) {
     
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
                motionUpdate(msg);
            }
        });


	// initialize particles
        ParameterTree paramTree = node.getParameterTree();
        String mapFile = paramTree.getString(node.resolveName("~/mapFileName"));
	mapParticleList = new ArrayList<MapParticle>();
	for(int i=0; i<MAX_PARTICLES; i++){
	    mapParticleList.add(new MapParticle(mapFile, MAX_PARTICLES));
	}

	initialized = false;
    }
    
    // performs sensor updates based on bump sensor values for all particles
    // updates the particle list, doesn't return anything
    public void bumpSensorUpdate(BumpMsg msg) {
        // TODO: sensor update
	// vaguely -- only send when true?

	if(initialized) {
	    for(MapParticle p : mapParticleList){
		p.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, curr_time - start_time);
	    }
        }

	
	start_x = curr_x;
	start_y = curr_y;
	start_theta = curr_theta;
	start_time = curr_time;
	
	// coordinate resampling based off of motion updates
	if(RESAMPLING)
	    RESAMPLING_COUNT++;
	if(RESAMPLING_COUNT >= RESAMPLING_FREQUENCY)
	    resample();

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
        mapMsg.setSerializedMap(ChannelBuffers.wrappedBuffer(byteStream.toByteArray()));
        mapPub.publish(mapMsg);
    }

    // performs sensor updates based on sonar values for all particles
    // updates the particle list, doesn't return anything
    public void sonarSensorUpdate(SonarMsg msg) {
	//update odometry before updating sensors
	if(!initialized)
	    for(MapParticle p : mapParticleList){
		p.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, curr_time - start_time);
	    }
	

	start_x = curr_x;
	start_y = curr_y;
	start_theta = curr_theta;
	start_time = curr_time;
	
	for(MapParticle p : mapParticleList){
	    p.sonarSensorUpdate(msg.getSonarValues());
	}


	// coordinate resampling based off of motion updates
	if(RESAMPLING)
	    RESAMPLING_COUNT++;
	if(RESAMPLING_COUNT >= RESAMPLING_FREQUENCY)
	    resample();	

	
    }

    // performs sensor updates based on fiducial observation
    // updates the particle list, doesn't return anything
    public void fiducialSensorUpdate(FiducialMsg msg){
	// TODO: sensor update
	// can do similarly to sonar update
    }

    // performs motion updates based on odometry for all particles
    // updates the particle list, doesn't return anything
    public void motionUpdate(OdometryMsg msg) {
	curr_x = msg.getX();
	curr_y = msg.getY();
	curr_theta = msg.getTheta();
	curr_time = msg.getTime();

	if(!initialized){
	    start_x = msg.getX();
	    start_y = msg.getY();
	    start_theta = msg.getTheta();
	    start_time = msg.getTime();
	    initialized = true;
	}
    }    
    
    // renormalize particles
    // this induces error -- since we need to represent weights as actual probabilities -- so
    // we avoid calling this unless we need them to resample
    public void renormalize(){
	double sum = 0;
	
    }

    // resample particles
    public void resample(){
	renormalize();
	RESAMPLING_COUNT = 0;
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
