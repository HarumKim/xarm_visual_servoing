#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from xarm_msgs.srv import MoveCartesian
import math
from tf_transformations import euler_from_quaternion

class XArmFollower(Node):
    def __init__(self):
        super().__init__('xarm_follower')
        
        # Cliente al servicio de movimiento XArm
        self.client = self.create_client(MoveCartesian, '/xarm/set_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /xarm/set_position...')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            'aligned_model_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('🔔 Suscrito a aligned_model_pose')
        
        # Parámetros de movimiento
        self.speed = 50.0  # mm/s
        self.acc = 500.0   # mm/s²
        
        # Throttle: ejecutar comando cada N segundos
        self.exec_period = 1.0  # segundos
        
        # Inicializo last_exec_time al pasado para permitir la primera ejecución
        self.last_exec_time = self.get_clock().now() - rclpy.duration.Duration(seconds=self.exec_period)
        
        # Pose inicial: X=291.3, Y=0, Z=444 (mm), orientación inicial
        initial_pos = [214.8, -12.0, 536.6]
        initial_orient = [-3.0892, -0.0611, 0.0332]  # roll, pitch, yaw en radianes (como tu comando)
        self.initial_full = initial_pos + initial_orient
        
        self.prev_pose = None
        self.ref_pose = None  # Guarda la primera pose recibida (como offset)
        
        self.get_logger().info('Ejecutando pose inicial')
        self._execute_pose(self.initial_full)
        self.prev_pose = self.initial_full
        self.last_exec_time = self.get_clock().now()
    
    def pose_callback(self, msg: PoseStamped):
        # Throttle: solo si pasó el periodo
        now = self.get_clock().now()
        elapsed = (now - self.last_exec_time).nanoseconds * 1e-9
        if elapsed < self.exec_period:
            return

        # Extraer datos de posición y orientación
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        
        try:
            roll, pitch, yaw = euler_from_quaternion(quaternion)
        except Exception as e:
            self.get_logger().warn(f'Error en quaternion: {e}')
            roll, pitch, yaw = 0.0, 0.0, 0.0
        
        current_pose = [x, y, z, roll, pitch, yaw]
        
        # Si es la primera pose recibida, solo la guardamos como referencia
        if self.ref_pose is None:
            self.ref_pose = current_pose
            self.get_logger().info('📌 Primera pose recibida guardada como referencia. No se ejecuta movimiento aún.')
            return
        
        self.last_exec_time = now
        
        # Calcular delta entre la referencia y la actual
        dx = current_pose[0] - self.ref_pose[0]
        dy = current_pose[1] - self.ref_pose[1]
        dz = current_pose[2] - self.ref_pose[2]
        droll = current_pose[3] - self.ref_pose[3]
        dpitch = current_pose[4] - self.ref_pose[4]
        dyaw = current_pose[5] - self.ref_pose[5]

        # Sumar delta a la pose inicial
        target_pose = [
            self.initial_full[0] + dx,
            self.initial_full[1] + dy,
            self.initial_full[2] + dz,
            self.initial_full[3] + droll,
            self.initial_full[4] + dpitch,
            self.initial_full[5] + dyaw,
        ]
        
        # Ejecutar pose
        self._execute_pose(target_pose)
        self.prev_pose = target_pose

    def _execute_pose(self, pose):
        """Envía la pose al servicio y muestra delta desde la anterior"""
        x, y, z, roll, pitch, yaw = pose
        
        # Calcular delta si existe prev_pose
        if self.prev_pose is not None:
            dx = x - self.prev_pose[0]
            dy = y - self.prev_pose[1]
            dz = z - self.prev_pose[2]
            droll = roll - self.prev_pose[3]
            dpitch = pitch - self.prev_pose[4]
            dyaw = yaw - self.prev_pose[5]
            
            self.get_logger().info(f'Delta posición: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f} mm')
            self.get_logger().info(f'Delta orientación: droll={droll:.3f}, dpitch={dpitch:.3f}, dyaw={dyaw:.3f} rad')
            self.get_logger().info(f'Delta orientación (°): droll={math.degrees(droll):.1f}, dpitch={math.degrees(dpitch):.1f}, dyaw={math.degrees(dyaw):.1f}')
        
        # Imprimir pose que se va a ejecutar
        self.get_logger().info(f'Ejecutando posición: x={x:.3f}, y={y:.3f}, z={z:.3f} mm')
        self.get_logger().info(f'Ejecutando orientación: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f} rad')
        self.get_logger().info(f'Ejecutando orientación (°): roll={math.degrees(roll):.1f}, pitch={math.degrees(pitch):.1f}, yaw={math.degrees(yaw):.1f}')
        
        # Construir y enviar petición al servicio MoveCartesian
        pose_list = [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
        req = MoveCartesian.Request()
        req.pose = pose_list
        req.speed = self.speed
        req.acc = self.acc
        req.mvtime = 0.0

        # Mostrar comando equivalente en consola (igual que tu comando de ejemplo)
        self.get_logger().info(f'📤 Comando equivalente:\nros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian ' +
                            f'"{{pose: {pose_list}, speed: {self.speed}, acc: {self.acc}, mvtime: {req.mvtime}}}"')

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info('✅ XArm movido correctamente')
        else:
            self.get_logger().error('❌ Falló el movimiento del XArm')

def main(args=None):
    rclpy.init(args=args)
    node = XArmFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()