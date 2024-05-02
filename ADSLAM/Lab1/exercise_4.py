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
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    


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


    #Select source
    source_path_c = "./datasets/livingroom1-color/" + "{:05d}".format(pos_source) + ".jpg"
    source_path_d = "./datasets/livingroom1-depth-clean/" + "{:05d}".format(pos_source) + ".png"

    #Select target
    target_path_c = "./datasets/livingroom1-color/" + "{:05d}".format(pos_target) + ".jpg"
    target_path_d = "./datasets/livingroom1-depth-clean/" + "{:05d}".format(pos_target) + ".png"
    gtposes = read_trajectory('./datasets/livingroom1-traj.txt')
    gt_source = gtposes[pos_source].pose
    gt_target = gtposes[pos_target].pose

    #1

    #Read image 1
    print("Read Redwood dataset")
    color_raw = o3d.io.read_image(source_path_c)
    depth_raw = o3d.io.read_image(source_path_d)
    rgbd_image_1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    print(rgbd_image_1)


    plt.subplot(1, 2, 1)
    plt.title('Redwood grayscale image')
    plt.imshow(rgbd_image_1.color)
    plt.subplot(1, 2, 2)
    plt.title('Redwood depth image')
    plt.imshow(rgbd_image_1.depth)
    plt.show()

    #Read image 2
    print("Read Redwood dataset")
    color_raw = o3d.io.read_image(target_path_c)
    depth_raw = o3d.io.read_image(target_path_d)
    rgbd_image_2 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)
    print(rgbd_image_2)


    # plt.subplot(1, 2, 1)
    # plt.title('Redwood grayscale image')
    # plt.imshow(rgbd_image_2.color)
    # plt.subplot(1, 2, 2)
    # plt.title('Redwood depth image')
    # plt.imshow(rgbd_image_2.depth)
    # plt.show()


    #Convert image 1 to pointcloud
    pcd_1 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image_1,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    #pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd_1])


    #Convert image 2 to pointcloud
    pcd_2 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image_2,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    #pcd_2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd_2])

    #Run ICP with identity as initial seed
    threshold = 0.02
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0], 
                            [0.0, 0.0, 0.0, 1.0]])
    draw_registration_result(pcd_1, pcd_2, trans_init)

    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        pcd_1, pcd_2, threshold, trans_init)
    print(evaluation)

    #ICP 2000 iterations
    print("Apply point-to-point ICP")
    start = time.time()
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd_1, pcd_2, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print("Point-to-point ICP took %.3f sec.\n" % (time.time() - start))
    print("Transformation That_ts is:")
    print(reg_p2p.transformation)
    draw_registration_result(pcd_1, pcd_2, reg_p2p.transformation)

    pcd_1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd_2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # #Point-to-plane ICP
    print("Apply point-to-plane ICP")
    start = time.time()
    reg_p2l = o3d.pipelines.registration.registration_icp(
        pcd_1, pcd_2, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print("Point-to-plane ICP took %.3f sec.\n" % (time.time() - start))
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(pcd_1, pcd_2, reg_p2l.transformation)


    That_ts_point_to_plane = reg_p2l.transformation
    That_ts_point_to_point = reg_p2p.transformation

    print("Errors: Te point to point")
    detR_p2p, dett_p2p = compute_transformation_errors(gt_source, gt_target, That_ts_point_to_point)
    print("rotation error (deg):")
    print(detR_p2p)
    print("translation error (m):")
    print(dett_p2p)

    print("Errors: Te point to plane")
    detR_p2l, dett_p2l = compute_transformation_errors(gt_source, gt_target, That_ts_point_to_plane)
    print("rotation error (deg):")
    print(detR_p2l)
    print("translation error (m):")
    print(dett_p2l)