package localization;

import java.util.ArrayList;

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
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

/*
 * @author - bhomberg 
 */

public class Localization implements NodeMain{

    // TODO: bump and fiducial updates
    // finish renormalizing and resampling

    String mapFile = "~//rss-team-6//src//rosjava_pkg//localization//src//main//java//localization//global-nav-maze-2011-basic.map";

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
	// TODO: figure how we get the map
	mapParticleList = new ArrayList<MapParticle>();
	for(int i=0; i<MAX_PARTICLES; i++){
	    mapParticleList.add(new MapParticle(mapFile, MAX_PARTICLES));
	}

	start_x = null;
	start_y = null;
	start_theta = null;
	start_time = null;
	curr_x = null;
	curr_y = null;
	curr_theta = null;
	curr_time = null;
    }
    
    // performs sensor updates based on bump sensor values for all particles
    // updates the particle list, doesn't return anything
    public void bumpSensorUpdate(BumpMsg msg) {
        // TODO: sensor update
	// vaguely -- only send when true?

	if(start_x != null)
	    for(MapParticle p : mapParticleList){
		p.motionUpdate(curr_x - start_x, curr_y - start_y, curr_theta - start_theta, curr_time - start_time);
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

    }

    // performs sensor updates based on sonar values for all particles
    // updates the particle list, doesn't return anything
    public void sonarSensorUpdate(SonarMsg msg) {
	//update odometry before updating sensors
	if(start_x != null)
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

	if(start_x == null){
	    start_x = msg.getX();
	    start_y = msg.getY();
	    start_theta = msg.getTheta();
	    start_time = msg.getTime();
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
