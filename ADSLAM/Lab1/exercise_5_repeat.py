import copy
import time
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import argparse

# Python program to get average of a list 
def average(lst): 
    return sum(lst) / len(lst) 

class CameraPose:
    def __init__(self, meta, mat):
        self.metadata = meta
        self.pose = mat
    def __str__(self):
        return 'Metadata : ' + ' '.join(map(str, self.metadata)) + '\n' + \
            "Pose : " + "\n" + np.array_str(self.pose)

def read_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        metastr = f.readline();
        while metastr:
            metadata = map(int, metastr.split())
            mat = np.zeros(shape = (4, 4))
            for i in range(4):
                matstr = f.readline();
                mat[i, :] = np.fromstring(matstr, dtype = float, sep=' \t')
            traj.append(CameraPose(metadata, mat))
            metastr = f.readline()
    return traj

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    
    # Get the center of mass of the transformed source cloud
    source_center = source_temp.get_center()
    
    # Define camera parameters
    camera_distance = 1 * source_temp.get_max_bound()[1]
    front = [0, -1, -3]  # Adjusted front direction
    lookat = source_center  # Look at the center of the transformed source cloud
    up = [0, 0, 1]  # Up direction
    
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=camera_distance,
                                      front=front,
                                      lookat=lookat,
                                      up=up)

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh   


def prepare_dataset(voxel_size, source, target):
    print(":: Load two point clouds and disturb initial pose.")

    # demo_icp_pcds = o3d.data.DemoICPPointClouds()
    # source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    # target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def compute_transformation_errors(gt_source, gt_target, That_ts):
    
    T_ts = np.linalg.inv(gt_target) @ gt_source
    # # Example converting a rotation matrix to Euler angles
    Te = np.linalg.inv(That_ts) @ T_ts
    Re = R.from_matrix(Te[:3,:3])
    te = Te[:3,3]
    dett = np.linalg.norm(te)
    detR = np.linalg.norm(Re.as_euler('zxy', degrees=True))

    return detR, dett


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Read source and target frame ids.")
    parser.add_argument("--source_frame", type=int, default = 20, help="Source frame ID")
    parser.add_argument("--target_frame", type=int, default = 21, help="Target frame ID")
    args = parser.parse_args()

    pos_source = args.source_frame
    pos_target = args.target_frame

    print("Source frame:", pos_source)
    print("Target frame:", pos_target)

    gtposes = read_trajectory('./datasets/livingroom1-traj.txt')
    gt_source = gtposes[pos_source].pose
    gt_target = gtposes[pos_target].pose

    #Select source
    source_path_c = "./datasets/livingroom1-color/" + "{:05d}".format(pos_source) + ".jpg"
    source_path_d = "./datasets/livingroom1-depth-clean/" + "{:05d}".format(pos_source) + ".png"

    #Select target
    target_path_c = "./datasets/livingroom1-color/" + "{:05d}".format(pos_target) + ".jpg"
    target_path_d = "./datasets/livingroom1-depth-clean/" + "{:05d}".format(pos_target) + ".png"

    #Read image 1
    print("Read Redwood dataset")
    color_raw = o3d.io.read_image(source_path_c)
    depth_raw = o3d.io.read_image(source_path_d)
    source_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    print(source_rgbd)

    #Read image 2
    print("Read Redwood dataset")
    color_raw = o3d.io.read_image(target_path_c)
    depth_raw = o3d.io.read_image(target_path_d)
    target_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    print(target_rgbd)

    #Convert source to pointcloud
    source = o3d.geometry.PointCloud.create_from_rgbd_image(
        source_rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    #pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd_1])


    #Convert target to pointcloud
    target = o3d.geometry.PointCloud.create_from_rgbd_image(
        target_rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    #pcd_2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd_2])


    #Misaligned point clouds with an identity matrix as transformation.
    voxel_size = 0.05  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, source, target)

    trans_init = np.identity(4)

    detsR_RANSAC = []
    detsT_RANSAC = []

    detsR_refined = []
    detsT_refined = []

    times_ransac = []
    times_refined = []

    fitness_ransac = []
    inlier_rmse_ransac = []
    correspondence_set_ransac = []

    fitness_refined = []
    inlier_rmse_refined = []
    correspondence_set_refined = []

    for i in range(10):
        #RANSAC
        start = time.time()
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    voxel_size)
        time_ransac = time.time() - start
        print("Global registration took %.3f sec.\n" % (time_ransac))
        # print(result_ransac)
        # draw_registration_result(source_down, target_down, result_ransac.transformation)
        
        # print("\nErrors: Initial transformation")
        detR_i, dett_i = compute_transformation_errors(gt_source, gt_target, trans_init)
        # print("rotation error (deg):")
        # print(detR_i)
        # print("translation error (m):")
        # print(dett_i)
        evaluation_i = o3d.pipelines.registration.evaluate_registration(
            source_down, target_down, voxel_size, trans_init)
        #print(evaluation_i)


        print("\nErrors: RANSAC")
        detR_ransac, dett_ransac = compute_transformation_errors(gt_source, gt_target, result_ransac.transformation)
        # print("rotation error (deg):")
        # print(detR_ransac)
        # print("translation error (m):")
        # print(dett_ransac)
        evaluation_ransac = o3d.pipelines.registration.evaluate_registration(
            source_down, target_down, voxel_size, result_ransac.transformation)
        #print(evaluation_ransac)
        
        fitness_ransac.append(evaluation_ransac.fitness)
        inlier_rmse_ransac.append(evaluation_ransac.inlier_rmse)
        correspondence_set_ransac.append(np.array(evaluation_ransac.correspondence_set).shape[0])

        detsR_RANSAC.append(detR_ransac)
        detsT_RANSAC.append(dett_ransac)
        times_ransac.append(time_ransac)

        # #Fast flobal registration
        # start = time.time()
        # result_fast = execute_fast_global_registration(source_down, target_down,
        #                                                source_fpfh, target_fpfh,
        #                                                voxel_size)
        # print("Fast global registration took %.3f sec.\n" % (time.time() - start))
        # print(result_fast)
        # draw_registration_result(source_down, target_down, result_fast.transformation)

        #Local refinement
        start = time.time()
        result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                        voxel_size)
        time_refined = time.time() - start
        #print("\nICP refinement took %.3f sec.\n" % (time_refined))
        #print(result_icp)
        # draw_registration_result(source, target, result_icp.transformation)

        print("Errors: ICP refinement")
        detR_refined, dett_refined = compute_transformation_errors(gt_source, gt_target, result_icp.transformation)
        # print("rotation error (deg):")
        # print(detR_refined)
        # print("translation error (m):")
        # print(dett_refined)
        evaluation_p2l = o3d.pipelines.registration.evaluate_registration(
            source_down, target_down, voxel_size, result_icp.transformation)
        #print(evaluation_p2l)

        fitness_refined.append(evaluation_p2l.fitness)
        inlier_rmse_refined.append(evaluation_p2l.inlier_rmse)
        correspondence_set_refined.append(np.array(evaluation_p2l.correspondence_set).shape[0])        
        
        detsR_refined.append(detR_refined)
        detsT_refined.append(dett_refined)
        times_refined.append(time_refined)


    print("Mean Errors: RANSAC")
    print("rotation error (deg):")
    print(average(detsR_RANSAC))
    print("translation error (m):")
    print(average(detsT_RANSAC))
    print("times (s):")
    print(average(times_ransac))

    print("fitness:")
    print(average(fitness_ransac))
    print("inlier_rmse:")
    print(average(inlier_rmse_ransac))
    print("correspondence_set size:")
    print(average(correspondence_set_ransac))


    print("Mean Errors: ICP refinement")
    print("rotation error (deg):")
    print(average(detsR_refined))
    print("translation error (m):")
    print(average(detsT_refined))
    print("times (s):")
    print(average(times_ransac) + average(times_refined))

    print("fitness:")
    print(average(fitness_refined))
    print("inlier_rmse:")
    print(average(inlier_rmse_refined))
    print("correspondence_set size:")
    print(average(correspondence_set_refined))