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

    protected boolean RESAMPLING = true;
    protected int RESAMPLING_FREQUENCY = 1000; // we should calibrate this -- my guess is we want to resample
                                               // about once a minute
    protected int RESAMPLING_COUNT = 0;

    @Override
    public void onStart(ConnectedNode node) {
     
	// Publishers
        mapPub = node.newPublisher("/loc/map", "rss_msgs/MapMsg");
        posPub = node.newPublisher("/loc/position", "rss_msgs/PositionMsg");


	// Subscribers
	bumpSub = node.newSubscriber("/uorc/bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
            @Override
            public void onNewMessage(BumpMsg msg) {
                bumpSensorUpdate(msg);
            }
        });


	sonSub = node.newSubscriber("/uorc/sonar", "rss_msgs/SonarMsg");
	sonSub.addMessageListener(new MessageListener<SonarMsg>() {
            @Override
            public void onNewMessage(SonarMsg msg) {
                sonarSensorUpdate(msg);
            }
        });


	fidSub = node.newSubscriber("/uorc/fiducial", "rss_msgs/FiducialMsg");
	fidSub.addMessageListener(new MessageListener<FiducialMsg>() {
            @Override
            public void onNewMessage(FiducialMsg msg) {
                fiducialSensorUpdate(msg);
            }
        });


        odoSub = node.newSubscriber("/odo/odometry", "rss_msgs/OdometryMsg");
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
    }
    
    // performs sensor updates based on bump sensor values for all particles
    // updates the particle list, doesn't return anything
    public void bumpSensorUpdate(BumpMsg msg) {
        // TODO: sensor update
	// vaguely -- only send when true?
    }

    // performs sensor updates based on sonar values for all particles
    // updates the particle list, doesn't return anything
    public void sonarSensorUpdate(SonarMsg msg) {
        for(MapParticle p : mapParticleList){
	    p.sonarSensorUpdate(msg.getSonarValues());
	}
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
	for(MapParticle p : mapParticleList){
	    p.motionUpdate(msg.getX(), msg.getY(), msg.getTheta(), msg.getTime());
	}

	// coordinate resampling based off of motion updates
	if(RESAMPLING)
	    RESAMPLING_COUNT++;
	if(RESAMPLING_COUNT >= RESAMPLING_FREQUENCY)
	    resample();
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
