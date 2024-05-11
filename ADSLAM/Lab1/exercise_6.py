import copy
import time
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import argparse
import matplotlib.pyplot as plt

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
    


def compute_transformation_errors(gt_source, gt_target, That_ts):
    
    T_ts = np.linalg.inv(gt_target) @ gt_source
    # # Example converting a rotation matrix to Euler angles
    Te = np.linalg.inv(That_ts) @ T_ts
    Re = R.from_matrix(Te[:3,:3])
    te = Te[:3,3]
    dett = np.linalg.norm(te)
    detR = np.linalg.norm(Re.as_euler('zxy', degrees=True))

    return detR, dett

def compute_odometry_errors(gt_target, That_wt):
    
    T_wt = gt_target
    # # Example converting a rotation matrix to Euler angles
    Te = np.linalg.inv(That_wt) @ T_wt
    Re = R.from_matrix(Te[:3,:3])
    te = Te[:3,3]
    dett = np.linalg.norm(te)
    detR = np.linalg.norm(Re.as_euler('zxy', degrees=True))

    return detR, dett


if __name__ == "__main__":

    
    # Read first frame and point cloud
    source_path_c = "./datasets/livingroom1-color/00000.jpg"
    source_path_d = "./datasets/livingroom1-depth-clean/00000.png"

    parser = argparse.ArgumentParser(description="Read source and target frame ids.")
    parser.add_argument("--point_to_plane", type=str, default="True", help="Source frame ID")
    parser.add_argument("--source_frame", type=int, default = 0, help="Source frame ID")
    parser.add_argument("--final_frame", type=int, default = 2870, help="Final frame ID")
    args = parser.parse_args()
    pos_source = args.source_frame
    pos_final = args.final_frame
    is_point_to_plane = args.point_to_plane.lower() in ['true', '1', 'yes', 'y']

    print("Source frame:", pos_source)
    print("Final frame:", pos_final)
    print("Point to plane:", is_point_to_plane)

    gtposes = read_trajectory('./datasets/livingroom1-traj.txt')
    
    gt_source = gtposes[pos_source].pose

    trajectory_length = (pos_final-pos_source)
    print(trajectory_length)

    #Read first
    color_raw = o3d.io.read_image(source_path_c)
    depth_raw = o3d.io.read_image(source_path_d)
    rgbd_src = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw)

    #Convert image 1 to pointcloud
    pcd_src = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_src,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    
    That_wt = gtposes[pos_source].pose

    trajectory = []
    trajectory_gt = []

    odometry_error_transl = []
    odometry_error_rot = []

    start = time.time()

    iteration = 0
    for i in range(pos_source+1, pos_source+trajectory_length):
        iteration = iteration + 1
        print(f"iteration {iteration}/{trajectory_length}: source frame={pos_source}, target frame={i}")
        #Select target
        gt_target = gtposes[i].pose
        target_path_c = "./datasets/livingroom1-color/" + "{:05d}".format(i) + ".jpg"
        target_path_d = "./datasets/livingroom1-depth-clean/" + "{:05d}".format(i) + ".png"
        #Read target
        color_raw = o3d.io.read_image(target_path_c)
        depth_raw = o3d.io.read_image(target_path_d)
        rgbd_image_target = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_raw, depth_raw)

        #Convert image 2 to pointcloud
        pcd_target = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image_target,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

        #Run ICP with identity as initial seed
        threshold = 0.02
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0], 
                                [0.0, 0.0, 0.0, 1.0]])
        #draw_registration_result(pcd_src, pcd_target, trans_init)

        # print("Initial alignment")
        evaluation = o3d.pipelines.registration.evaluate_registration(
            pcd_src, pcd_target, threshold, trans_init)
        # print(evaluation)


        if(is_point_to_plane):
            pcd_src.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            pcd_target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            # #Point-to-plane ICP
            reg_p2p = o3d.pipelines.registration.registration_icp(
                pcd_src, pcd_target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPlane())
            #print("Point-to-plane ICP took %.3f sec.\n" % (time.time() - start))
            #print("Transformation is:")
            #print(reg_p2l.transformation)
            #draw_registration_result(pcd_src, pcd_target, reg_p2l.transformation)

        else:
            #Point-to-point ICP 2000 iterations
            reg_p2p = o3d.pipelines.registration.registration_icp(
                pcd_src, pcd_target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            # print("Point-to-point ICP took %.3f sec.\n" % (time.time() - start))
            # print("Transformation That_ts is:")
            # print(reg_p2p.transformation)
            #draw_registration_result(pcd_src, pcd_target, reg_p2p.transformation)


        That_ts = reg_p2p.transformation
        That_wt = That_wt @ np.linalg.inv(That_ts)

        trajectory.append(That_wt[:3,3])
        trajectory_gt.append(gt_target[:3,3])

        # print("Errors: Te")
        detR_p2p, dett_p2p = compute_transformation_errors(gt_source, gt_target, That_ts)
        # print("rotation error (deg):")
        # print(detR_p2p)
        # print("translation error (m):")
        # print(dett_p2p)

        # print("Errors: Global Te")
        odomR_p2p, odomt_p2p = compute_odometry_errors(gt_target, That_wt)
        # print("rotation error (deg):")
        # print(odomR_p2p)
        # print("translation error (m):")
        # print(odomt_p2p)

        odometry_error_transl.append(odomt_p2p)
        odometry_error_rot.append(odomR_p2p)

        pos_source = i
        pcd_src = pcd_target
        gt_source = gt_target

    print("\n Trajectory took %.3f sec.\n" % (time.time() - start))

    print("\nErrors: Global Te")
    detR_p2p, dett_p2p = compute_odometry_errors(gt_target, That_wt)
    print("rotation error (deg):")
    print(detR_p2p)
    print("translation error (m):")
    print(dett_p2p)
    # pcd_ini = o3d.geometry.PointCloud.create_from_rgbd_image(
    #     rgbd_src,
    #     o3d.camera.PinholeCameraIntrinsic(
    #         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # evaluation_p2p = o3d.pipelines.registration.evaluate_registration(
    #     pcd_ini, pcd_target, threshold, That_wt)
    # print(evaluation_p2p)


    # Convert the trajectories to numpy arrays for easier manipulation
    trajectory = np.array(trajectory)
    trajectory_gt = np.array(trajectory_gt)
    odometry_error_transl = np.array(odometry_error_transl)
    odometry_error_rot = np.array(odometry_error_rot)

    # Save trajectories as text files
    np.savetxt('trajectory.txt', trajectory)
    np.savetxt('trajectory_gt.txt', trajectory_gt)
    np.savetxt('odometry_error_transl.txt', odometry_error_transl)
    np.savetxt('odometry_error_rot.txt', odometry_error_rot)

    # Create figure and subplots
    fig, (ax1, ax2) = plt.subplots(2, 1)

    # Plot odometry_error_transl on top subplot
    ax1.plot(odometry_error_transl)
    ax1.set_title('Odometry Error - Translation(m)')

    # Plot odometry_error_rot on bottom subplot
    ax2.plot(odometry_error_rot)
    ax2.set_title('Odometry Error- Rotation(º)')

    # Adjust layout
    plt.tight_layout()

    # Show plot
    plt.show()


   # Plot the trajectories
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, and z coordinates from the trajectories arrays
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]

    x_gt = trajectory_gt[:, 0]
    y_gt = trajectory_gt[:, 1]
    z_gt = trajectory_gt[:, 2]

    # Plot the trajectories
    ax.plot(x, y, z, color='blue', label='Trajectory')
    ax.plot(x_gt, y_gt, z_gt, color='red', label='Ground Truth Trajectory')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectories (m)')
    ax.legend()

    # Show plot
    plt.show()