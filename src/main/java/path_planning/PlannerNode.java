package path_planning;

import java.awt.geom.Point2D;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.text.ParseException;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import rss_msgs.MapMsg;
import rss_msgs.PositionMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
import map.CSpace;
import map.PolygonMap;

public class PlannerNode extends AbstractNodeMain {

    // TODO: Fill me in
    private static final double ROBOT_RADIUS = 0.3;
    private static final int RRT_MAX_POINTS = 300;

    /* Publishers and subscribers */
    private Publisher<WaypointMsg> targetPub;
    private Subscriber<PositionTargetMsg> goalSub;
    private Subscriber<PositionMsg> positionSub;
    private Subscriber<MapMsg> mapSub;

    /* Our current pose state */
    private double x;
    private double y;
    private double theta;

    /* Our current map state */
    private PolygonMap map;

    private void handlePositionMsg(PositionMsg msg) {
        // Save our pose
        x = msg.getX();
        y = msg.getY();
        theta = msg.getTheta();
    }

    private void handleGoalMsg(PositionTargetMsg msg) {
        // Update our path plan, and publish to targetPub
        CSpace cSpace = new CSpace(map.getObstacles(), ROBOT_RADIUS);
        Point2D.Double start = new Point2D.Double(x, y);
        Point2D.Double goal = new Point2D.Double(msg.getX(), msg.getY());
        RRTStar rrt_graph = new RRTStar(start, goal, map.getWorldRect(), cSpace, RRT_MAX_POINTS);
        List<Point2D.Double> path = rrt_graph.computeShortestPath(start, goal);

        // TODO: Draw full path to gui
        // Maybe also computed tree? (might be slow)
        Point2D.Double nextWaypoint = path.get(0);
        WaypointMsg waypointMsg = targetPub.newMessage();
        waypointMsg.setX(nextWaypoint.x);
        waypointMsg.setY(nextWaypoint.y);
        // TODO: Theta?
        targetPub.publish(waypointMsg);
    }        

    private void handleMapMsg(MapMsg msg) {
        try {
            ObjectInputStream stream = new ObjectInputStream(
                new ByteArrayInputStream(msg.getSerializedMap().array()));
            map = (PolygonMap) stream.readObject();
            stream.close();
        }
        catch (IOException e) {
            e.printStackTrace();
            return;
        }
        catch (ClassNotFoundException e) {
            e.printStackTrace();
            return;
        }
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
        goalSub = node.newSubscriber("/command/Goal", "rss_msgs/PositionTargetMsg");
        goalSub.addMessageListener(new MessageListener<PositionTargetMsg>() {
            @Override
            public void onNewMessage(PositionTargetMsg msg) {
                handleGoalMsg(msg);
            }
        });
        mapSub = node.newSubscriber("/loc/map", "rss_msgs/MapMsg");
        mapSub.addMessageListener(new MessageListener<MapMsg>() {
            @Override
            public void onNewMessage(MapMsg msg) {
                handleMapMsg(msg);
            }
        });
        targetPub = node.newPublisher("/path/target", "rss_msgs/WaypointMsg");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("path_planner");
    }
}
