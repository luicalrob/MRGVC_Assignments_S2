import copy
import time
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import argparse

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
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

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


    # plt.subplot(1, 2, 1)
    # plt.title('Redwood grayscale image')
    # plt.imshow(rgbd_image_1.color)
    # plt.subplot(1, 2, 2)
    # plt.title('Redwood depth image')
    # plt.imshow(rgbd_image_1.depth)
    # plt.show()

    #Read image 2
    print("Read Redwood dataset")
    color_raw = o3d.io.read_image(target_path_c)
    depth_raw = o3d.io.read_image(target_path_d)
    target_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    print(target_rgbd)


    # plt.subplot(1, 2, 1)
    # plt.title('Redwood grayscale image')
    # plt.imshow(rgbd_image_2.color)
    # plt.subplot(1, 2, 2)
    # plt.title('Redwood depth image')
    # plt.imshow(rgbd_image_2.depth)
    # plt.show()


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

    #RANSAC
    start = time.time()
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print("Global registration took %.3f sec.\n" % (time.time() - start))
    print(result_ransac)
    draw_registration_result(source_down, target_down, result_ransac.transformation)

    print("Errors: RANSAC")
    detR_ransac, dett_ransac = compute_transformation_errors(gt_source, gt_target, result_ransac.transformation)
    print("rotation error (deg):")
    print(detR_ransac)
    print("translation error (m):")
    print(dett_ransac)

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
    print("ICP refinement took %.3f sec.\n" % (time.time() - start))
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)

    print("Errors: ICP refinement")
    detR_refined, dett_refined = compute_transformation_errors(gt_source, gt_target, result_icp.transformation)
    print("rotation error (deg):")
    print(detR_refined)
    print("translation error (m):")
    print(dett_refined)