import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool
from turtle_interfaces.srv import SetWayPoint as SetWayPointSrv
 
# (1) Crée un nœud set_way_point.py dans turtle_regulation
class SetWayPoint(Node):
    def __init__(self):
        super().__init__("set_way_point")
 
        # (2) Crée un attribut pour stocker la pose de la tortue
        self.pose_tortue = None
 
        # (3) Définir un attribut waypoint avec les coordonnées (7,7)
        self.waypoint_x = 7.0
        self.waypoint_y = 7.0
 
        # (5) Définir la constante Kp pour la commande proportionnelle
        self.kp = 2.0
 
        # (2) Souscrire au topic /turtle1/pose de type Pose
        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10
        )
 
        # (5) Crée un publisher pour publier sur /turtle1/cmd_vel
        self.publisher_cmd = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )
 
      # (5) Crée un timer pour exécuter la régulation régulièrement
        self.timer = self.create_timer(0.1, self.control_loop)

        # Partie 2 - Q2: gain pour la vitesse linéaire
        self.kpl = 3.0

        # Partie 2 - Q3: seuil de distance pour arrêter le mouvement
        self.distance_tolerance = 0.1

        # Partie 2 - Q4: publisher booléen sur is_moving
        self.publisher_is_moving = self.create_publisher(Bool, "/is_moving", 10)

        self.set_waypoint_srv = self.create_service(
            SetWayPointSrv,
            "set_waypoint_service",
            self.set_waypoint_callback
        )

 
         # (2) Quand on reçoit un message pose, on met à jour l’attribut pose_tortue
    def pose_callback(self, msg):
        self.pose_tortue = msg
 
        # (4) Calcul de l’angle désiré entre la tortue et le waypoint
    def calcul_angle_desire(self):
        if self.pose_tortue is None:
            return None
 
        xA = self.pose_tortue.x
        yA = self.pose_tortue.y
 
        xB = self.waypoint_x
        yB = self.waypoint_y
 
        theta_desired = math.atan2(yB - yA, xB - xA)
        return theta_desired
 
        # (5) Calcul de l’erreur, la commande u, puis publier cmd_vel
    def control_loop(self):
        if self.pose_tortue is None:
            return

        # Partie 2 - Q1: distance entre tortue et waypoint
        dx = self.waypoint_x - self.pose_tortue.x
        dy = self.waypoint_y - self.pose_tortue.y
        distance = math.sqrt(dx**2 + dy**2)

        theta_desired = self.calcul_angle_desire()
        theta = self.pose_tortue.theta

        e = 2 * math.atan(math.tan((theta_desired - theta) / 2))
        u = self.kp * e

        # Partie 2 - Q3: si on est assez proche du waypoint, on n'envoie plus de mouvement
        if distance < self.distance_tolerance:
            cmd = Twist()
            self.publisher_cmd.publish(cmd)

            # Partie 2 - Q4: la tortue ne bouge plus
            is_moving_msg = Bool()
            is_moving_msg.data = False
            self.publisher_is_moving.publish(is_moving_msg)
            return
        
        # Partie 2 - Q4: la tortue est en mouvement
        is_moving_msg = Bool()
        is_moving_msg.data = True
        self.publisher_is_moving.publish(is_moving_msg)

        # Partie 2 - Q2: v = Kpl * el
        el = distance
        v = self.kpl * el

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = u
        self.publisher_cmd.publish(cmd)
 
    def set_waypoint_callback(self, request, response):
        self.waypoint_x = request.x
        self.waypoint_y = request.y
        response.res = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SetWayPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
