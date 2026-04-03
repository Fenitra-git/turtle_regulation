import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
 
 
# (1) Créer un nœud set_way_point.py dans turtle_regulation
class SetWayPoint(Node):
    def __init__(self):
        super().__init__("set_way_point")
 
        # (2) Créer un attribut pour stocker la pose de la tortue
        self.pose_tortue = None
 
        # (3) Définir un attribut waypoint avec les coordonnées (7,7)
        self.waypoint_x = 7.0
        self.waypoint_y = 7.0
 
        # (5) Définir la constante Kp pour la commande proportionnelle
        self.kp = 0.5
 
        # (2) Souscrire au topic /turtle1/pose de type Pose
        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10
        )
 
        # (5) Créer un publisher pour publier sur /turtle1/cmd_vel
        self.publisher_cmd = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )
 
      # (5) Créer un timer pour exécuter la régulation régulièrement
        self.timer = self.create_timer(0.1, self.control_loop)
 
         # (2) Quand on reçoit un message pose, on met à jour l’attribut pose_tortue
    def pose_callback(self, msg):
        self.pose_tortue = msg
 
        # (4) Calculer l’angle désiré entre la tortue et le waypoint
    def calcul_angle_desire(self):
        if self.pose_tortue is None:
            return None
 
        xA = self.pose_tortue.x
        yA = self.pose_tortue.y
 
        xB = self.waypoint_x
        yB = self.waypoint_y
 
        theta_desired = math.atan2(yB - yA, xB - xA)
        return theta_desired
 
        # (5) Calculer l’erreur, la commande u, puis publier cmd_vel
    def control_loop(self):
        if self.pose_tortue is None:
            return
 
        theta_desired = self.calcul_angle_desire()
        theta = self.pose_tortue.theta
 
        e = 2 * math.atan(math.tan((theta_desired - theta) / 2))
        u = self.kp * e
 
        cmd = Twist()
        cmd.angular.z = u
        self.publisher_cmd.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SetWayPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()