import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import math
import copy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        # Parámetros de carga y alineación
        self.declare_parameter('reference_ply_file', '')
        self.declare_parameter('voxel_size', 0.005)
        self.declare_parameter('ransac_distance', 0.01)
        self.declare_parameter('ransac_n', 4)
        self.declare_parameter('ransac_iterations', 100000)
        self.declare_parameter('icp_distance', 0.005)
        self.declare_parameter('use_ransac', True)
        # Parámetros para estimación robusta (M-estimador de Huber)
        self.declare_parameter('robust_enable', False)
        self.declare_parameter('robust_threshold', 1.0)
        self.declare_parameter('robust_max_iter', 20)
        self.declare_parameter('robust_tol', 1e-6)
        # Parámetros de calidad (ajustados para ser más permisivos)
        self.declare_parameter('min_fitness', 0.02)  # Reducido de 0.3 a 0.02
        self.declare_parameter('max_rmse', 0.01)     # Reducido de 0.02 a 0.01

        # Leer parámetros
        self.ply_path = self.get_parameter('reference_ply_file').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.ransac_distance = self.get_parameter('ransac_distance').value
        self.ransac_n = int(self.get_parameter('ransac_n').value)
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.icp_distance = self.get_parameter('icp_distance').value
        self.use_ransac = bool(self.get_parameter('use_ransac').value)
        self.robust_enable = bool(self.get_parameter('robust_enable').value)
        self.robust_threshold = float(self.get_parameter('robust_threshold').value)
        self.robust_max_iter = int(self.get_parameter('robust_max_iter').value)
        self.robust_tol = float(self.get_parameter('robust_tol').value)
        self.min_fitness = float(self.get_parameter('min_fitness').value)
        self.max_rmse = float(self.get_parameter('max_rmse').value)

        # Publishers y TF
        self.reference_pub = self.create_publisher(PointCloud2, '/pointcloud/reference_object', 10)
        self.aligned_pub = self.create_publisher(PointCloud2, '/pointcloud/aligned_model', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'aligned_model_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud/segmented_object',
            self.cloud_callback,
            10
        )

        # Cargar y validar CAD de referencia
        if not self.ply_path:
            self.get_logger().error("No se especificó archivo PLY de referencia")
            return
        
        try:
            self.get_logger().info(f"Cargando CAD de referencia: {self.ply_path}")
            self.reference_pcd = o3d.io.read_point_cloud(self.ply_path)
            if len(self.reference_pcd.points) == 0:
                self.get_logger().error("El archivo PLY está vacío")
                return
            self.get_logger().info(f"CAD cargado: {len(self.reference_pcd.points)} puntos")
        except Exception as e:
            self.get_logger().error(f"Error cargando PLY: {e}")
            return

        self.create_timer(1.0, self.publish_reference_cloud)

    def publish_reference_cloud(self):
        """Publica la nube de referencia constantemente para visualización"""
        if hasattr(self, 'reference_pcd') and len(self.reference_pcd.points) > 0:
            msg = self.convert_pcd_to_ros_msg(self.reference_pcd)
            if msg is not None:
                self.reference_pub.publish(msg)
                # Log cada 10 segundos para debug
                if hasattr(self, '_ref_publish_count'):
                    self._ref_publish_count += 1
                    if self._ref_publish_count % 10 == 0:
                        self.get_logger().info(f"Publicando nube de referencia: {len(self.reference_pcd.points)} puntos")
                else:
                    self._ref_publish_count = 1
                    self.get_logger().info(f"Comenzando publicación de nube de referencia: {len(self.reference_pcd.points)} puntos")

    def convert_pcd_to_ros_msg(self, pcd):
        """Convierte Open3D PointCloud a ROS PointCloud2"""
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return None
            
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None
        data = []
        
        for i, pt in enumerate(points):
            x, y, z = pt
            if colors is not None:
                r, g, b = [int(c * 255) for c in colors[i]]
            else:
                r = g = b = 255
            rgb = (r << 16) | (g << 8) | b
            data.append((x, y, z, rgb))
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_color_optical_frame'
        
        return pc2.create_cloud(header, fields, data)

    def robust_pose_estimation(self, source_pcd, target_pcd):
        """Estimación robusta de pose usando M-estimador de Huber"""
        pts_src = np.asarray(source_pcd.points)
        tgt_tree = o3d.geometry.KDTreeFlann(target_pcd)
        
        R_current = np.eye(3)
        t_current = np.zeros(3)
        
        for iteration in range(self.robust_max_iter):
            self.get_logger().info(f"Iteración robusta {iteration+1}/{self.robust_max_iter}")
            
            # Transformar puntos fuente
            src_trans = (R_current @ pts_src.T).T + t_current
            
            # Encontrar correspondencias
            src_corr, tgt_corr = [], []
            for p in src_trans:
                _, idx, _ = tgt_tree.search_knn_vector_3d(p, 1)
                if len(idx) > 0:
                    src_corr.append(p)
                    tgt_corr.append(np.asarray(target_pcd.points)[idx[0]])
            
            if len(src_corr) < 3:
                self.get_logger().warn("Muy pocas correspondencias para estimación robusta")
                break
                
            src_corr = np.array(src_corr)
            tgt_corr = np.array(tgt_corr)
            
            # Calcular residuales y pesos
            residuals = tgt_corr - src_corr
            norms = np.linalg.norm(residuals, axis=1)
            sigma = max(np.median(norms) / 0.6745, 1e-6)
            
            # Pesos de Huber
            threshold = self.robust_threshold * sigma
            weights = np.where(norms <= threshold, 1.0, threshold / (norms + 1e-6))
            
            # Centroides ponderados
            w = weights[:, None]
            total_weight = weights.sum()
            if total_weight < 1e-6:
                break
                
            mean_src = (w * src_corr).sum(axis=0) / total_weight
            mean_tgt = (w * tgt_corr).sum(axis=0) / total_weight
            
            # Alineación ponderada
            src_centered = src_corr - mean_src
            tgt_centered = tgt_corr - mean_tgt
            
            W_matrix = (w * src_centered).T @ tgt_centered
            
            try:
                U, _, VT = np.linalg.svd(W_matrix)
                R_new = VT.T @ U.T
                
                # Asegurar rotación válida
                if np.linalg.det(R_new) < 0:
                    VT[2, :] *= -1
                    R_new = VT.T @ U.T
                    
                t_new = mean_tgt - R_new @ mean_src
                
                # Verificar convergencia
                R_diff = np.linalg.norm(R_new - R_current)
                t_diff = np.linalg.norm(t_new - t_current)
                
                if R_diff < self.robust_tol and t_diff < self.robust_tol:
                    self.get_logger().info(f"Convergencia robusta en iteración {iteration+1}")
                    R_current = R_new
                    t_current = t_new
                    break
                    
                R_current = R_new
                t_current = t_new
                
            except np.linalg.LinAlgError:
                self.get_logger().warn("Error en SVD durante estimación robusta")
                break
        
        # Construir matriz de transformación
        T = np.eye(4)
        T[:3, :3] = R_current
        T[:3, 3] = t_current
        return T

    def preprocess_cloud(self, pcd):
        """Preprocesa nube de puntos: filtro de outliers y downsampling"""
        # Filtrar outliers estadísticos
        pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        # Downsample
        pcd_down = pcd_filtered.voxel_down_sample(self.voxel_size)
        
        # Calcular normales
        if len(pcd_down.points) > 0:
            pcd_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 2, 
                    max_nn=30
                )
            )
        
        return pcd_down

    def validate_registration_quality(self, fitness, rmse):
        """Valida la calidad del registro con logging detallado"""
        is_valid = True
        
        if fitness < self.min_fitness:
            self.get_logger().warn(f"Fitness bajo: {fitness:.3f} < {self.min_fitness} (aún así se procesa)")
            # No invalidamos automáticamente por fitness bajo
        
        if rmse > self.max_rmse:
            self.get_logger().warn(f"RMSE alto: {rmse:.3f} > {self.max_rmse}")
            is_valid = False
            
        # Criterios adicionales más flexibles
        if fitness > 0.01 and rmse < 0.02:  # Criterios más permisivos
            self.get_logger().info(f"Registro aceptable: fitness={fitness:.3f}, rmse={rmse:.3f}")
            is_valid = True
        elif fitness < 0.01:
            self.get_logger().error(f"Fitness demasiado bajo: {fitness:.3f}")
            is_valid = False
            
        return is_valid

    def publish_aligned_pose(self, transformation):
        """Publica la pose alineada como PoseStamped"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_color_optical_frame'

        # Extraer traslación
        pose_msg.pose.position.x = float(transformation[0, 3])
        pose_msg.pose.position.y = float(transformation[1, 3])
        pose_msg.pose.position.z = float(transformation[2, 3])

        # Extraer rotación usando scipy
        try:
            rotation = R.from_matrix(transformation[:3, :3])
            quat = rotation.as_quat()  # [x, y, z, w]
            
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            
            self.pose_pub.publish(pose_msg)
            self.get_logger().info("Pose publicada exitosamente")
            
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo rotación: {e}")

    def cloud_callback(self, msg):
        """Callback principal para procesar nubes de puntos segmentadas"""
        try:
            # Convertir nube segmentada a Open3D
            pts = [tuple(p) for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)]
            if not pts:
                self.get_logger().warn("Nube de puntos vacía recibida")
                return
            
            xyz = np.array(pts, dtype=np.float64)
            live_pcd = o3d.geometry.PointCloud()
            live_pcd.points = o3d.utility.Vector3dVector(xyz)
            
            self.get_logger().info(f"Procesando nube con {len(xyz)} puntos")

            # --- Alineación inicial por plano ---
            ref_transformed = copy.deepcopy(self.reference_pcd)
            
            try:
                # Segmentar plano en nube en vivo
                plane_model, inliers = live_pcd.segment_plane(
                    distance_threshold=self.icp_distance,
                    ransac_n=3,
                    num_iterations=1000
                )
                
                if len(inliers) < 10:
                    raise Exception("Muy pocos puntos en plano detectado")
                
                live_plane = live_pcd.select_by_index(inliers)
                n_live = np.array(plane_model[:3])
                
                # Segmentar plano en CAD de referencia
                ref_plane_model, ref_inliers = ref_transformed.segment_plane(
                    distance_threshold=self.icp_distance,
                    ransac_n=3,
                    num_iterations=1000
                )
                
                if len(ref_inliers) < 10:
                    raise Exception("Muy pocos puntos en plano de referencia")
                
                ref_plane = ref_transformed.select_by_index(ref_inliers)
                n_ref = np.array(ref_plane_model[:3])
                
                # Calcular rotación para alinear normales
                v = np.cross(n_ref, n_live)
                s = np.linalg.norm(v)
                c = np.dot(n_ref, n_live)
                
                if s > 1e-6:  # Vectores no paralelos
                    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                    R_plane = np.eye(3) + K + K @ K * ((1 - c) / (s**2))
                else:
                    R_plane = np.eye(3)
                
                # Traslación de centroides
                cen_ref_plane = np.asarray(ref_plane.points).mean(axis=0)
                cen_live_plane = np.asarray(live_plane.points).mean(axis=0)
                t_plane = cen_live_plane - R_plane @ cen_ref_plane
                
                # Aplicar transformación de plano
                T_plane = np.eye(4)
                T_plane[:3, :3] = R_plane
                T_plane[:3, 3] = t_plane
                ref_transformed.transform(T_plane)
                
                self.get_logger().info("Alineación inicial de plano exitosa")
                
            except Exception as e:
                self.get_logger().warn(f"Alineación de plano falló: {e}, usando modelo original")
                ref_transformed = copy.deepcopy(self.reference_pcd)

            # --- Escalado y centrado ---
            cen_live = xyz.mean(axis=0)
            cen_model = np.asarray(ref_transformed.points).mean(axis=0)
            
            # Calcular escalas basadas en extensión máxima
            extent_live = np.linalg.norm(xyz - cen_live, axis=1).max()
            extent_model = np.linalg.norm(np.asarray(ref_transformed.points) - cen_model, axis=1).max()
            
            scale = extent_live / extent_model if extent_model > 1e-6 else 1.0
            
            # Aplicar escalado y traslación
            model_init = copy.deepcopy(ref_transformed)
            model_init.scale(scale, center=cen_model)
            model_init.translate(cen_live - cen_model)
            
            self.get_logger().info(f"Aplicado escalado: {scale:.3f}")

            # --- Preprocesamiento ---
            src_down = self.preprocess_cloud(model_init)
            tgt_down = self.preprocess_cloud(live_pcd)
            
            if len(src_down.points) < 10 or len(tgt_down.points) < 10:
                self.get_logger().error("Muy pocos puntos después del preprocesamiento")
                return

            # --- Estimación de pose ---
            best_score = -np.inf
            best_trans = np.eye(4)
            chosen_method = ''

            if self.robust_enable:
                self.get_logger().info("Aplicando estimación robusta...")
                try:
                    robust_T = self.robust_pose_estimation(src_down, tgt_down)
                    src_robust = copy.deepcopy(src_down)
                    src_robust.transform(robust_T)
                    
                    # Refinamiento con ICP
                    icp_res = o3d.pipelines.registration.registration_icp(
                        src_robust, tgt_down,
                        self.icp_distance, np.eye(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPlane()
                    )
                    
                    best_trans = icp_res.transformation @ robust_T
                    best_score = icp_res.fitness - icp_res.inlier_rmse
                    chosen_method = 'robust_icp'
                    
                    self.get_logger().info(f"Método robusto: fitness={icp_res.fitness:.3f}, rmse={icp_res.inlier_rmse:.3f}")
                    
                except Exception as e:
                    self.get_logger().error(f"Error en estimación robusta: {e}")

            else:
                # RANSAC + ICP
                if self.use_ransac:
                    try:
                        # Calcular características FPFH
                        f_src = o3d.pipelines.registration.compute_fpfh_feature(
                            src_down, o3d.geometry.KDTreeSearchParamHybrid(
                                radius=self.voxel_size * 5, max_nn=100
                            )
                        )
                        f_tgt = o3d.pipelines.registration.compute_fpfh_feature(
                            tgt_down, o3d.geometry.KDTreeSearchParamHybrid(
                                radius=self.voxel_size * 5, max_nn=100
                            )
                        )
                        
                        # RANSAC
                        ransac_res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                            src_down, tgt_down, f_src, f_tgt, True,
                            self.ransac_distance,
                            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                            self.ransac_n,
                            [
                                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.ransac_distance)
                            ],
                            o3d.pipelines.registration.RANSACConvergenceCriteria(self.ransac_iterations, 0.999)
                        )
                        
                        # Refinamiento con ICP
                        src_ransac = copy.deepcopy(src_down)
                        src_ransac.transform(ransac_res.transformation)
                        
                        icp_res = o3d.pipelines.registration.registration_icp(
                            src_ransac, tgt_down,
                            self.icp_distance, np.eye(4),
                            o3d.pipelines.registration.TransformationEstimationPointToPlane()
                        )
                        
                        trans_ransac = icp_res.transformation @ ransac_res.transformation
                        score_ransac = icp_res.fitness - icp_res.inlier_rmse
                        
                        if score_ransac > best_score:
                            best_score = score_ransac
                            best_trans = trans_ransac
                            chosen_method = 'ransac_icp'
                        
                        self.get_logger().info(f"RANSAC+ICP: fitness={icp_res.fitness:.3f}, rmse={icp_res.inlier_rmse:.3f}")
                        
                    except Exception as e:
                        self.get_logger().error(f"Error en RANSAC: {e}")
                
                # ICP directo
                try:
                    icp_direct = o3d.pipelines.registration.registration_icp(
                        src_down, tgt_down,
                        self.icp_distance, np.eye(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPlane()
                    )
                    
                    score_direct = icp_direct.fitness - icp_direct.inlier_rmse
                    
                    if score_direct > best_score:
                        best_score = score_direct
                        best_trans = icp_direct.transformation
                        chosen_method = 'icp_direct'
                    
                    self.get_logger().info(f"ICP directo: fitness={icp_direct.fitness:.3f}, rmse={icp_direct.inlier_rmse:.3f}")
                    
                except Exception as e:
                    self.get_logger().error(f"Error en ICP directo: {e}")

            # --- Validación y publicación ---
            if chosen_method:
                # Evaluar calidad final
                final_src = copy.deepcopy(src_down)
                final_src.transform(best_trans)
                
                final_eval = o3d.pipelines.registration.evaluate_registration(
                    final_src, tgt_down, self.icp_distance
                )
                
                self.get_logger().info(f"Método elegido: {chosen_method}, score={best_score:.4f}")
                self.get_logger().info(f"Evaluación final: fitness={final_eval.fitness:.3f}, rmse={final_eval.inlier_rmse:.3f}")
                
                if self.validate_registration_quality(final_eval.fitness, final_eval.inlier_rmse):
                    # Transformar modelo completo y publicar
                    aligned_full = copy.deepcopy(model_init)
                    aligned_full.transform(best_trans)
                    
                    aligned_msg = self.convert_pcd_to_ros_msg(aligned_full)
                    if aligned_msg:
                        self.aligned_pub.publish(aligned_msg)
                        self.get_logger().info("Nube alineada publicada")
                    
                    # Publicar pose
                    self.publish_aligned_pose(best_trans)
                    
                    # Publicar TF
                    self.publish_transform(best_trans)
                    
                    self.get_logger().info("✅ Estimación de pose completada exitosamente")
                else:
                    self.get_logger().warn("❌ Calidad de registro insuficiente, no se publica pose")
                    
                    # DEBUG: Publicar de todas formas para visualización
                    aligned_full = copy.deepcopy(model_init)
                    aligned_full.transform(best_trans)
                    aligned_msg = self.convert_pcd_to_ros_msg(aligned_full)
                    if aligned_msg:
                        self.aligned_pub.publish(aligned_msg)
                        self.get_logger().info("🔍 Publicando resultado para debug visual")
            else:
                self.get_logger().error("Ningún método de registro fue exitoso")
                
        except Exception as e:
            self.get_logger().error(f"Error en callback: {e}")

    def publish_transform(self, transformation):
        """Publica la transformación como TF"""
        try:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'camera_color_optical_frame'
            tf_msg.child_frame_id = 'object_frame'
            
            # Traslación
            tf_msg.transform.translation.x = float(transformation[0, 3])
            tf_msg.transform.translation.y = float(transformation[1, 3])
            tf_msg.transform.translation.z = float(transformation[2, 3])
            
            # Rotación usando scipy
            rotation = R.from_matrix(transformation[:3, :3])
            quat = rotation.as_quat()  # [x, y, z, w]
            
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            
            self.tf_broadcaster.sendTransform(tf_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publicando TF: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PoseEstimator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error en main: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()