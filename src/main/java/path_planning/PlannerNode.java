package path_planning;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import rss_msgs.PositionMsg;
import rss_msgs.PositionTargetMsg;

public class PlannerNode extends AbstractNodeMain {

    /* Publishers and subscribers */
    private Publisher<PositionTargetMsg> positionPub;
    private Subscriber<PositionMsg> targetPub;

    /* Our current pose state */
    private float x;
    private float y;
    private float theta;

    private void handlePositionMsg(PositionMsg msg) {
        // Save our pose
        x = msg.getX();
        y = msg.getY();
        theta = msg.getTheta();
        
        // Update our path plan, and publish to targetPub
        
    }

    @Override
    public void onStart(ConnectedNode node) {
        positionSub = node.newSubscriber("/loc/position", "rss_msgs/PositonMsg");
        positionSub.addMessageListener(new MessageListener<PositionMsg>() {
            @Override
            public void onNewMessage(PositionMsg msg) {
                handlePositionMsg(msg);
            }
        });
        targetPub = node.newSubscriber("/path/target", "rss_msgs/PositionTargetMsg");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("path_planner");
    }
}