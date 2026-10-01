import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Lab constants
THETA_DEG = 60
LOOKAHEAD = 0.6  # m
DESIRED_DISTANCE_FROM_WALL = 0.8  # m
INTEGRAL_WINDOW_SIZE = 10
KP = 0.0    # 0 à 2.5
KI = 0.0    # 0 à 0.5, facultatif, crée un effet de lag sur le contrôle de l'erreur
KD = 0.0    # 0.01 à 0.1


def angle_to_distance(
    theta_rad: float,
    lidar_array: list[float],
    angle_min: float,
    angle_increment: float,
) -> float:
    """Retourne la distance LiDAR mesurée pour un angle donné dans le plan du capteur.

    La fonction convertit un angle absolu du laser en index dans le tableau de
    mesures, puis renvoie la distance correspondante. Retourne math.inf si l'angle est hors de portée du scan LiDAR.
    """
    # TODO: Convertir l'angle radian en index dans le tableau lidar_array
    pass


def is_valid_lidar_scan(scan: float) -> bool:
    """Vérifie si une mesure LiDAR est exploitable ou si elle est invalide (infinie)."""
    return not math.isinf(scan) and not math.isnan(scan)


class WallFollowNode(Node):
    """Nœud ROS qui suit un mur à partir des données LiDAR.

    Le nœud publie des commandes Ackermann sur le topic ``drive`` afin de
    maintenir la distance souhaitée à un mur. Il estime l'erreur de distance à
    partir de deux points de mesure du mur, puis corrige cette erreur via un
    contrôle PID.
    """

    def __init__(self):
        """Initialise le subscriber LiDAR, le publisher de commande, et les états PID."""
        super().__init__("wall_follow_node")

        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.last_time = None
        self.last_steering = 0.0
        self.last_errors_window = np.array([])

    def lidar_callback(self, scan: LaserScan):
        """Traite un nouveau scan LiDAR et en déduit la commande de pilotage.

        Cette méthode:
        - récupère les distances à deux angles de détection du mur,
        - calcule la distance effective au mur et l'erreur relative à la consigne,
        - applique un correcteur PID,
        - publie la commande de vitesse et de direction.
        """
        steering = 0.0  # rad
        throttle = 2.2  # m/s

        lidar_range_array: list[float] = scan.ranges  # type: ignore
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # 1. TODO: Récupérer les distances LiDAR à deux angles de détection du mur
        
        # 2. TODO: Calculer la distance effective au mur et l'erreur relative à la distance désirée
        
        # 3. TODO: Calcul de PID
        
        # 4. TODO: Mettre à jour les variables d'état
        
        # 5. TODO: Ajuster la vitesse en fonction de l'angle de braquage (steering) pour éviter les collisions

        self.send_control_command(throttle, steering)

    def send_control_command(self, throttle: float, steering: float):
        """Publie une commande de vitesse et de direction sur le topic ``drive``."""
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.frame_id = "base_link"
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()

        ackermann_msg.drive.speed = throttle
        ackermann_msg.drive.steering_angle = steering

        self.drive_pub.publish(ackermann_msg)


def main(args=None):
    """Point d'entrée du programme ROS : lance le nœud et le maintient en vie."""
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
