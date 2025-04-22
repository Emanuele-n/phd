import argparse
import os
import pyvista as pv
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.interpolate import interp1d
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

"""
This script reads a centerline .vtp file and computes the Frenet-Serret frame at each point.
Setting the transformation matrix w_T_o from the Frenet-Serret (w) frame to the orignal frame (o), set in the CAD 
"""


def smooth_vectors(vectors, window_size=10, passes=1):
    """Apply a simple smoothing filter to vectors multiple times."""
    smoothed_vectors = vectors.copy()
    for _ in range(passes):
        # Pad the vectors at the start and end
        pad_width = window_size // 2
        padded_vectors = np.pad(
            smoothed_vectors, ((pad_width, pad_width), (0, 0)), mode="edge"
        )

        # Apply a uniform filter
        for i in range(len(smoothed_vectors)):
            smoothed_vectors[i] = np.mean(padded_vectors[i : i + window_size], axis=0)

        # Normalize the smoothed vectors
        norms = np.linalg.norm(smoothed_vectors, axis=1)
        smoothed_vectors = smoothed_vectors / norms[:, np.newaxis]

    return smoothed_vectors


def preprocess_points(points, epsilon=1e-5):
    """
    Preprocess the points by removing duplicates or points that are too close to each other.
    :param points: np.array of points
    :param epsilon: Minimum distance between points
    :return: np.array of preprocessed points
    """
    # Check if points array is at least one-dimensional
    if points.ndim < 1:
        print("Error: Points data must be at least one-dimensional.")
        exit()

    # Check for a minimum number of points
    if len(points) < 2:
        print("Error: Not enough points to compute differences.")
        exit()

    unique_points = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - unique_points[-1]) > epsilon:
            unique_points.append(point)
    return np.array(unique_points)


def interpolate_fs_frames(fs1, fs2, num_points=10):
    """
    Interpolates between two FS frames by linearly interpolating the translations and spherically interpolating the rotations.
    """
    R1, t1 = fs1[:3, :3], fs1[:3, 3]
    R2, t2 = fs2[:3, :3], fs2[:3, 3]

    # Linear interpolation of translations:
    translations = np.linspace(t1, t2, num_points)

    # Spherical linear interpolation (slerp) for rotations:
    key_times = [0, 1]
    rotations = Rotation.from_matrix([R1, R2])
    slerp = Slerp(key_times, rotations)
    interp_times = np.linspace(0, 1, num_points)
    interp_rotations = slerp(interp_times).as_matrix()

    fs_frames = []
    for i in range(num_points):
        fs = np.eye(4)
        fs[:3, :3] = interp_rotations[i]
        fs[:3, 3] = translations[i]
        fs_frames.append(fs)
    return fs_frames


def interpolate_line(points, num_points=None):
    """
    Robustly interpolate a line through 3D points, handling duplicates and non-monotonic data.
    :param points: np.array of 3D points
    :param num_points: Number of points in the interpolated line
    :return: np.array of interpolated points
    """
    try:
        points = preprocess_points(np.array(points))
        if num_points is None:
            num_points = len(points) * 3  # Default to 3x the number of input points

        # Compute the cumulative distance along the line
        distances = np.cumsum(np.r_[0, np.linalg.norm(np.diff(points, axis=0), axis=1)])

        # Handle duplicates in distances
        distances, unique_indices = np.unique(distances, return_index=True)
        points = points[unique_indices]

        # Ensure that distances are strictly increasing
        if len(distances) < 2 or np.any(np.diff(distances) <= 0):
            print("Error: Not enough unique points to interpolate.")
            return None

        # Normalize the distances
        distance_normalized = distances / distances[-1]

        # Interpolate for each dimension
        interpolated_points = np.zeros((num_points, 3))
        for i in range(3):
            interpolator = interp1d(
                distance_normalized,
                points[:, i],
                kind="cubic",
                bounds_error=False,
                fill_value="extrapolate",
            )
            interpolated_points[:, i] = interpolator(np.linspace(0, 1, num_points))

        return interpolated_points

    except Exception as e:
        print(f"An error occurred during interpolation: {e}")
        return None


def compute_tangent_vectors(interpolated_points):
    # Compute tangent vectors using finite differences
    tangents = np.diff(interpolated_points, axis=0)
    tangents = np.vstack(
        (tangents, tangents[-1])
    )  # Ensure same number of tangents as points
    # Normalize the tangent vectors
    norms = np.linalg.norm(tangents, axis=1)
    tangents = tangents / norms[:, np.newaxis]
    tangents = smooth_vectors(tangents)  # Smooth the tangent vectors
    return tangents


def compute_MRF(tangents):
    tangents = np.array(tangents)
    normals = np.zeros_like(tangents)
    binormals = np.zeros_like(tangents)

    # Initialize the normal for the first point
    # Choose an arbitrary vector that is not parallel to the tangent
    arbitrary = (
        np.array([0, 0, 1])
        if not np.allclose(tangents[0], [0, 0, 1])
        else np.array([0, 1, 0])
    )
    normals[0] = np.cross(tangents[0], arbitrary)
    if np.linalg.norm(normals[0]) == 0:
        raise ValueError(
            "Cannot define a normal vector; tangent is parallel to the arbitrary vector."
        )
    normals[0] /= np.linalg.norm(normals[0])

    # Compute the binormal for the first point
    binormals[0] = np.cross(tangents[0], normals[0])

    # Propagate the frame along the curve
    for i in range(1, len(tangents)):
        # Project the previous normal onto the plane orthogonal to the current tangent
        proj = normals[i - 1] - np.dot(normals[i - 1], tangents[i]) * tangents[i]
        norm_proj = np.linalg.norm(proj)
        if norm_proj < 1e-6:
            # Handle the case where the previous normal is parallel to the current tangent
            arbitrary = (
                np.array([0, 0, 1])
                if not np.allclose(tangents[i], [0, 0, 1])
                else np.array([0, 1, 0])
            )
            normals[i] = np.cross(tangents[i], arbitrary)
        else:
            normals[i] = proj / norm_proj

        # Compute the binormal as the cross product of tangent and normal
        binormals[i] = np.cross(tangents[i], normals[i])

    # Smooth the normals and binormals if needed
    normals = smooth_vectors(normals)
    binormals = smooth_vectors(binormals)

    # Re-orthogonalize the frame after smoothing to correct any deviations
    for i in range(len(tangents)):
        # Ensure tangent is normalized
        tangents[i] /= np.linalg.norm(tangents[i])

        # Recompute normal: make it orthogonal to tangent
        normals[i] = normals[i] - np.dot(normals[i], tangents[i]) * tangents[i]
        normals[i] /= np.linalg.norm(normals[i])

        # Recompute binormal to ensure right-handedness
        binormals[i] = np.cross(tangents[i], normals[i])
        binormals[i] /= np.linalg.norm(binormals[i])

    # Build rotation matrix
    R = np.stack((tangents, normals, binormals), axis=1)

    # Check if the matrix is orthogonal
    # Verify orthogonality
    for i in range(len(R)):
        RtR = R[i].T @ R[i]
        I = np.eye(3)
        error = RtR - I
        max_error = np.abs(error).max()
        if max_error > 1e-6:
            print(
                f"Rotation matrix at index {i} is not orthogonal enough. Max error: {max_error}"
            )
            raise ValueError("Frame is not orthogonal.")

    # Check if the matrix is right-handed
    if not np.allclose(np.linalg.det(R), 1):
        raise ValueError("Frame is not right-handed.")

    return normals, binormals


def compute_normal_vectors(tangents):
    # Use finite differences to approximate normal vectors
    normals = np.diff(tangents, axis=0)
    normals = np.vstack(
        (normals, normals[-1])
    )  # Ensure same number of normals as tangents

    # Replace any zero vectors with NaN to avoid division by zero
    norms = np.linalg.norm(normals, axis=1)
    zeros = norms < 1e-6
    normals[zeros] = np.nan

    # Normalize the normal vectors, skipping any NaN entries
    normals = normals / norms[:, None]

    # Handle NaNs if there were any zero vectors
    normals[zeros] = [0, 0, 0]  # or any other placeholder for zero vectors

    normals = smooth_vectors(normals)  # Smooth the normal vectors
    return normals


def compute_binormal_vectors(tangents, normals):
    # Compute binormal vectors as the cross product of tangent and normal vectors
    binormals = np.cross(tangents, normals)
    # Normalize the binormal vectors
    norms = np.linalg.norm(binormals, axis=1)
    zeros = norms < 1e-6
    binormals[zeros] = np.nan
    binormals = binormals / norms[:, np.newaxis]
    binormals[zeros] = [0, 0, 0]  # or any other placeholder for zero vectors
    binormals = smooth_vectors(binormals)  # Smooth the binormal vectors
    return binormals


def draw_FS_frames(
    num_points=100, draw_tangent=True, draw_normal=True, draw_binormal=True, path=None
):
    # Load the .vtp file
    # line_model = pv.read("data/mesh/vascularmodel/0023_H_AO_MFS/sim/path.vtp")
    # line_model = pv.read("data/mesh/vascularmodel/0063_H_PULMGLN_SVD/sim/path.vtp")
    # line_model = pv.read("data/mesh/easier_slam_test/centerline.vtp")
    line_model = pv.read(path)
    # line_model = pv.read("data/mesh/easier_slam_test/path.vtp")
    n_cells = line_model.n_cells
    print(f"points: {line_model.points.shape}")
    print(f"Number of branches (cells): {n_cells}")

    # Create a plotter
    plotter = pv.Plotter()

    # Draw the origin of the world frame
    plotter.add_arrows(np.zeros((1, 3)), np.array([[10, 0, 0]]), color="red", mag=1)
    plotter.add_arrows(np.zeros((1, 3)), np.array([[0, 10, 0]]), color="green", mag=1)
    plotter.add_arrows(np.zeros((1, 3)), np.array([[0, 0, 10]]), color="blue", mag=1)

    for i in range(n_cells):
        # Extract the i-th cell (branch)
        single_line = line_model.extract_cells(i)
        points = single_line.points
        print(f"Processing branch {i} with {len(points)} points")

        if len(points) < 2:
            print(f"Skipping branch {i} due to insufficient points.")
            continue

        # Interpolate the line for smoothing
        interpolated_points = interpolate_line(points)

        if interpolated_points is None or len(interpolated_points) == 0:
            print(f"Skipping branch {i} due to interpolation failure.")
            continue

        # Compute tangent vectors for the interpolated points
        tangents = compute_tangent_vectors(interpolated_points)

        # Compute the Frenet-Serret frame using the MRF algorithm
        normals, binormals = compute_MRF(tangents)

        # Define a list of distinct colors for branches
        branch_colors = ["red", "green", "blue", "orange", "purple", "cyan", "yellow"]
        branch_color = branch_colors[i % len(branch_colors)]  # Cycle through colors

        # Draw the very first point of the branch
        plotter.add_points(interpolated_points[0], color=branch_color, point_size=10)

        # Add the branch to the plotter
        plotter.add_mesh(single_line, color="blue", line_width=2)

        # Select random points from the interpolated line to display frames
        random_indices = np.random.choice(
            len(interpolated_points),
            min(num_points, len(interpolated_points)),
            replace=False,
        )

        # Add Frenet-Serret frames to the plotter for the random points
        for idx in random_indices:
            point = interpolated_points[idx]
            tangent = tangents[idx]
            normal = normals[idx]
            binormal = binormals[idx]

            mag = 1.0
            # Draw the tangent vector
            if draw_tangent:
                plotter.add_arrows(
                    point[np.newaxis], tangent[np.newaxis], color="red", mag=mag
                )
            # Draw the normal vector
            if draw_normal:
                plotter.add_arrows(
                    point[np.newaxis], normal[np.newaxis], color="green", mag=mag
                )
            # Draw the binormal vector
            if draw_binormal:
                plotter.add_arrows(
                    point[np.newaxis], binormal[np.newaxis], color="blue", mag=mag
                )

    # Show the plotter
    plotter.show()
    return


def save_frames_single_branch(input_path):

    # Load the .vtp file and define output path based on input path
    input_filename = os.path.basename(input_path)
    output_filename = input_filename.replace(".vtp", "_fs.txt")
    output_path = os.path.join(os.path.dirname(input_path), output_filename)

    line_model = pv.read(input_path)
    n_cells = line_model.n_cells
    print(f"Number of branches (cells): {n_cells}")

    # Open the output file once
    with open(output_path, "w") as file:
        for i in range(n_cells):
            # Extract the i-th cell (branch)
            single_line = line_model.extract_cells(i)
            points = single_line.points
            print(f"Processing branch {i} with {len(points)} points")

            # Convert from mm to m
            # points = points / 1000

            if len(points) < 2:
                print(f"Skipping branch {i} due to insufficient points.")
                continue

            # Interpolate the line for smoothing
            interpolated_points = interpolate_line(points)

            if interpolated_points is None or len(interpolated_points) == 0:
                print(f"Skipping branch {i} due to interpolation failure.")
                continue

            # Compute tangent vectors for the interpolated points
            tangents = compute_tangent_vectors(interpolated_points)

            # Compute the Frenet-Serret frame using the MRF algorithm
            normals, binormals = compute_MRF(tangents)

            # For each point, save the coordinates and the respective FS frame
            for idx in range(len(interpolated_points)):
                point = interpolated_points[idx]
                tangent = tangents[idx]
                normal = normals[idx]
                binormal = binormals[idx]

                # Write to file
                file.write(
                    f"{point[0]}, {point[1]}, {point[2]}, "
                    f"{tangent[0]}, {tangent[1]}, {tangent[2]}, "
                    f"{normal[0]}, {normal[1]}, {normal[2]}, "
                    f"{binormal[0]}, {binormal[1]}, {binormal[2]}\n"
                )

    return output_path


def save_frames_all_branches(input_paths):

    for path in input_paths:
        output_fs = save_frames_single_branch(path)
        output_tum = output_fs.replace("_fs.txt", "_tum.txt")
        convert_fs_to_tum(output_fs, output_tum, convention="wTc")


def convert_fs_to_tum(input_file, output_file, convention="wTc"):
    """
    Convert Frenet-Serret frames (px, py, pz, Tx, Ty, Tz, Nx, Ny, Nz, Bx, By, Bz)
    into TUM format:

        timestamp px py pz qx qy qz qw

    To get the series of w_T_ci matrices of the ground truth trajectory

    o is the origin frame (CAD origin)
    w is the world frame (SLAM origin)
    The w frame has to correspond to the first FS frame in the centerline

    The matrix saved in the FS frame file is built as
    o_T_fsi = [Tx_i, Nx_i, Bx_i, px_i]
                [Ty_i, Ny_i, By_i, py_i]
                [Tz_i, Nz_i, Bz_i, pz_i]
                [0, 0, 0, 1]
    so is the transformation from the i-th fs frame to the origin frame

    To align the fs frame to the camera convention, rotate the frame 90 degrees around the n axis
    (t forward, n down, b left) -> (z forward, x right, y down)
    R_n(90) =   [0, 0, 1]
                [0, 1, 0]
                [-1, 0, 0]

    fsi_T_ci =  [0, 0, 1, 0]
                [0, 1, 0, 0]
                [-1, 0, 0, 0]
                [0, 0, 0, 1]

    Then
    o_T_ci = o_T_fsi * fsi_T_ci

    The first point of the centerline (i = 0) corresponds to the transformation from the world frame to the origin frame
    o_T_w = o_T_fs0 * fs0_T_c0

    The camera trajectory of the SLAM is saved as a series of Twc (w_T_ci) transformations,
    so to get the ground truth data from here I have to obtain all the Twc_i (w_T_ci) matrices

    w_T_ci = o_T_w^-1 * o_T_ci


    """

    with open(input_file, "r") as fin, open(output_file, "w") as fout:
        lines = fin.readlines()
        first = True
        o_T_w = np.eye(4)
        w_T_o = np.eye(4)

        for i, line in enumerate(lines):
            # Each line has 12 floats: px, py, pz, Tx, Ty, Tz, Nx, Ny, Nz, Bx, By, Bz
            vals = line.strip().split(",")
            if len(vals) != 12:
                continue

            # Parse floats
            px, py, pz = map(float, vals[0:3])
            # x = x / 1000.0
            # y = y / 1000.0
            # z = z / 1000.0
            tx, ty, tz = map(float, vals[3:6])
            nx, ny, nz = map(float, vals[6:9])
            bx, by, bz = map(float, vals[9:12])

            # Build rotation matrix as in file: t forward, n down, b left
            o_R_fsi = np.array([[tx, nx, bx], [ty, ny, by], [tz, nz, bz]])

            # Check determinant and adjust if necessary
            if np.linalg.det(o_R_fsi) < 0:
                print("Determinant is negative. Adjusting rotation matrix.")
                continue

            # Verify orthogonality
            RtR = o_R_fsi.T @ o_R_fsi
            I = np.eye(3)
            error = RtR - I
            max_error = np.abs(error).max()
            if max_error > 1e-6:
                print(f"Rotation matrix not orthogonal enough. Max error: {max_error}")
                continue

            # Build o_T_fsi (FS frame as originally set)
            o_T_fsi = np.eye(4)
            o_T_fsi[:3, :3] = o_R_fsi
            o_T_fsi[:3, 3] = [px, py, pz]

            # Apply rotation to match camera convention build fsi_T_ci (from FS frame to camera frame)
            # Rotate 90 degrees around n axis
            Rn = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            fsi_T_ci = np.eye(4)
            fsi_T_ci[:3, :3] = Rn
            fsi_T_ci[:3, 3] = [0, 0, 0]

            # Build o_T_ci = o_T_fsi * fsi_T_ci (from origin to camera frame)
            o_T_ci = o_T_fsi @ fsi_T_ci

            # Handle first frame
            if first:
                o_T_w = o_T_ci
                w_T_o = np.linalg.inv(o_T_w)
                first = False

            # Compute w_T_ci = (o_T_w)^-1 * o_T_ci
            w_T_ci = w_T_o @ o_T_ci
            w_R_ci = w_T_ci[:3, :3]

            if convention == "wTc":
                final = w_T_ci
                final_rot = w_R_ci
            elif convention == "cTw":
                final = np.linalg.inv(w_T_ci)
                final_rot = final[:3, :3]

            # Convert to quaternion
            rot = Rotation.from_matrix(final_rot.astype(np.float64))
            qx, qy, qz, qw = rot.as_quat()

            # Write TUM format: timestamp px py pz qx qy qz qw
            timestamp = float(i)
            fout.write(
                f"{timestamp:.6f} {final[0, 3]:.6f} {final[1, 3]:.6f} {final[2, 3]:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
            )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Process centerline file and compute Frenet-Serret frames."
    )
    parser.add_argument(
        "i", type=str, help="Path to the input centerline .vtp file or folder"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()

    draw_only = False

    if draw_only:
        draw_FS_frames(path=args.i)

    else:
        # Check if input is a file or directory
        if os.path.isfile(args.i) and args.i.endswith(".vtp"):
            # Process single .vtp file
            save_frames_single_branch(args.i)
        elif os.path.isdir(args.i):
            # Process all centerline_b*.vtp files in directory
            vtp_files = [
                os.path.join(args.i, f)
                for f in os.listdir(args.i)
                if f.startswith("b") and f.endswith(".vtp")
            ]

            save_frames_all_branches(vtp_files)

        else:
            print(
                "Error: Input must be either a .vtp file or a directory containing .vtp files"
            )
