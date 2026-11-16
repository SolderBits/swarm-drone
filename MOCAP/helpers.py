import numpy as np
from scipy import linalg, optimize, signal
import cv2 as cv
from scipy.spatial.transform import Rotation
import copy
import json
import os
import time
from KalmanFilter import KalmanFilter
from Singleton import Singleton
from mocap_config import CONFIG
from camera_backend import open_cameras
from drone_locator import DroneLocator
from sfm_compat import sfm, USING_OPENCV_SFM

EPIPOLAR_MAX_PX = CONFIG["epipolar_threshold_px"] or 5.0 * CONFIG["process_width"] / 320.0

SHARPEN_KERNEL = np.array([[-2,-1,-1,-1,-2],
                           [-1,1,3,1,-1],
                           [-1,3,4,3,-1],
                           [-1,1,3,1,-1],
                           [-2,-1,-1,-1,-2]])

@Singleton
class Cameras:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "camera-params.json")
        f = open(filename)
        self.camera_params = json.load(f)

        if CONFIG["camera_backend"].lower() == "ps3eye":
            from pseyepy import Camera
            self.cameras = Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)
        else:
            self.cameras = open_cameras(CONFIG)
        self.num_cameras = len(self.cameras.exposure)
        print(f"[mocap] {self.num_cameras} cameras, backend {CONFIG['camera_backend']}, "
              f"{'OpenCV' if USING_OPENCV_SFM else 'numpy'} SFM functions")
        if len(self.camera_params) < self.num_cameras:
            raise RuntimeError(f"camera-params.json has {len(self.camera_params)} entries but there are "
                               f"{self.num_cameras} cameras; add one entry per camera")
        self._check_intrinsics_size()
        self._undistort_maps = {}
        self._last_stats_print = time.time()
        self._last_heading_warning = 0
        self.locator = DroneLocator(CONFIG["num_drones"], CONFIG["drone_markers"],
                                    CONFIG["marker_tolerance"], CONFIG["tracking_gate"])

        self.is_capturing_points = False

        self.is_triangulating_points = False
        self.camera_poses = None

        self.is_locating_objects = False

        self.to_world_coords_matrix = None

        self.drone_armed = []

        self.num_objects = None

        self.kalman_filter = None

        self.socketio = None
        self.ser = None

        self.serialLock = None

        global cameras_init
        cameras_init = True

    def set_socketio(self, socketio):
        self.socketio = socketio

    def set_ser(self, ser):
        self.ser = ser

    def set_serialLock(self, serialLock):
        self.serialLock = serialLock

    def set_num_objects(self, num_objects):
        if self.num_objects == num_objects:
            return
        self.num_objects = num_objects
        self.drone_armed = [False for i in range(0, self.num_objects)]
        markers = list(CONFIG["drone_markers"])
        markers += [markers[-1]] * max(0, num_objects - len(markers))
        self.locator = DroneLocator(num_objects, markers, CONFIG["marker_tolerance"], CONFIG["tracking_gate"])

    def edit_settings(self, exposure, gain):
        self.cameras.exposure = [exposure] * self.num_cameras
        self.cameras.gain = [gain] * self.num_cameras

    def _check_intrinsics_size(self):
        if CONFIG["camera_backend"].lower() == "ps3eye":
            return
        size = max(CONFIG["process_width"], CONFIG["process_height"])
        for i in range(self.num_cameras):
            K = np.array(self.camera_params[i]["intrinsic_matrix"], dtype=float)
            if abs(K[0, 2] - size / 2) > 0.25 * size or abs(K[1, 2] - size / 2) > 0.25 * size:
                print(f"[mocap] WARNING: camera {i} intrinsics have their centre at ({K[0,2]:.0f}, {K[1,2]:.0f}) but frames "
                      f"are {size}x{size}: camera-params.json was made for another size. "
                      f"Run tools/calibrate_intrinsics.py --camera {i}")

    def _undistort(self, i, frame):

        h, w = frame.shape[:2]
        key = (i, w, h)
        maps = self._undistort_maps.get(key)
        if maps is None:
            K = self.get_camera_params(i)["intrinsic_matrix"].astype(np.float64)
            D = self.get_camera_params(i)["distortion_coef"].astype(np.float64)
            maps = cv.initUndistortRectifyMap(K, D, None, K, (w, h), cv.CV_16SC2)
            self._undistort_maps[key] = maps
        return cv.remap(frame, maps[0], maps[1], cv.INTER_LINEAR)

    def _print_stats(self):
        if time.time() - self._last_stats_print < 5 or not hasattr(self.cameras, "stats"):
            return
        self._last_stats_print = time.time()
        print(f"[cameras] {self.cameras.stats()}")

    def _camera_read(self):
        frames, _ = self.cameras.read()
        self._print_stats()

        for i in range(0, self.num_cameras):
            frames[i] = np.rot90(frames[i], k=self.camera_params[i]["rotation"])
            frames[i] = make_square(frames[i])
            frames[i] = self._undistort(i, frames[i])
            frames[i] = cv.GaussianBlur(frames[i],(9,9),0)
            frames[i] = cv.filter2D(frames[i], -1, SHARPEN_KERNEL)
            frames[i] = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)

        if (self.is_capturing_points):
            image_points = []
            for i in range(0, self.num_cameras):
                frames[i], single_camera_image_points = self._find_dot(frames[i])
                image_points.append(single_camera_image_points)

            if (any(np.all(point[0] != [None,None]) for point in image_points)):
                if self.is_capturing_points and not self.is_triangulating_points:
                    self.socketio.emit("image-points", [x[0] for x in image_points])
                elif self.is_triangulating_points:
                    errors, object_points, frames = find_point_correspondance_and_object_points(image_points, self.camera_poses, frames)

                    for i, object_point in enumerate(object_points):
                        new_object_point = np.array([[-1,0,0],[0,-1,0],[0,0,1]]) @ object_point
                        new_object_point = np.concatenate((new_object_point, [1]))
                        new_object_point = np.array(self.to_world_coords_matrix) @ new_object_point
                        new_object_point = new_object_point[:3] / new_object_point[3]
                        new_object_point[1], new_object_point[2] = new_object_point[2], new_object_point[1]
                        object_points[i] = new_object_point

                    objects = []
                    filtered_objects = []
                    if self.is_locating_objects:
                        objects = self.locator.locate(object_points, errors)
                        filtered_objects = self.kalman_filter.predict_location(objects)

                        if len(filtered_objects) != 0:
                            for filtered_object in filtered_objects:
                                if self.drone_armed[filtered_object['droneIndex']]:
                                    filtered_object["heading"] = round(float(filtered_object["heading"]), 4)
                                    if abs(filtered_object["heading"]) > np.pi / 4 and time.time() - self._last_heading_warning > 2:
                                        self._last_heading_warning = time.time()
                                        print(f"[mocap] WARNING: drone {filtered_object['droneIndex']} heading is "
                                              f"{np.degrees(filtered_object['heading']):.0f} deg; its front LED should point along +x")

                                    serial_data = {
                                        "pos": [round(x, 4) for x in filtered_object["pos"].tolist()] + [filtered_object["heading"]],
                                        "vel": [round(x, 4) for x in filtered_object["vel"].tolist()]
                                    }
                                    with self.serialLock:
                                        self.ser.write(f"{filtered_object['droneIndex']}{json.dumps(serial_data)}".encode('utf-8'))
                                        time.sleep(0.001)

                        for filtered_object in filtered_objects:
                            filtered_object["vel"] = filtered_object["vel"].tolist()
                            filtered_object["pos"] = filtered_object["pos"].tolist()

                        for filtered_object in filtered_objects:
                            filtered_object["heading"] = float(filtered_object["heading"])

                    self.socketio.emit("object-points", {
                        "object_points": object_points.tolist(),
                        "errors": errors.tolist(),
                        "objects": [{k:(v.tolist() if isinstance(v, np.ndarray) else v) for (k,v) in object.items()} for object in objects],
                        "filtered_objects": filtered_objects
                    })

        return frames

    def get_frames(self):
        frames = self._camera_read()

        stacked = np.hstack(frames)
        display_h = int(CONFIG.get("display_height") or 0)
        if display_h and stacked.shape[0] > display_h:
            scale = display_h / stacked.shape[0]
            stacked = cv.resize(stacked, (int(round(stacked.shape[1] * scale)), display_h), interpolation=cv.INTER_AREA)
        return stacked

    def _find_dot(self, img):

        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255*CONFIG["dot_threshold"], 255, cv.THRESH_BINARY)[1]
        contours,_ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        img = cv.drawContours(img, contours, -1, (0,255,0), 1)

        image_points = []
        for contour in contours:
            moments = cv.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                cv.putText(img, f'({center_x}, {center_y})', (center_x,center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (100,255,100), 1)
                cv.circle(img, (center_x,center_y), 1, (100,255,100), -1)
                image_points.append([center_x, center_y])

        if len(image_points) == 0:
            image_points = [[None, None]]

        return img, image_points

    def start_capturing_points(self):
        self.is_capturing_points = True

    def stop_capturing_points(self):
        self.is_capturing_points = False

    def start_trangulating_points(self, camera_poses):
        self.is_capturing_points = True
        self.is_triangulating_points = True
        self.camera_poses = camera_poses
        self.kalman_filter = KalmanFilter(self.num_objects)
        self.locator.reset()

    def stop_trangulating_points(self):
        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.camera_poses = None

    def start_locating_objects(self):
        self.is_locating_objects = True

    def stop_locating_objects(self):
        self.is_locating_objects = False

    def get_camera_params(self, camera_num):
        return {
            "intrinsic_matrix": np.array(self.camera_params[camera_num]["intrinsic_matrix"]),
            "distortion_coef": np.array(self.camera_params[camera_num]["distortion_coef"]),
            "rotation": self.camera_params[camera_num]["rotation"]
        }

    def set_camera_params(self, camera_num, intrinsic_matrix=None, distortion_coef=None):
        if intrinsic_matrix is not None:
            self.camera_params[camera_num]["intrinsic_matrix"] = intrinsic_matrix

        if distortion_coef is not None:
            self.camera_params[camera_num]["distortion_coef"] = distortion_coef
        self._undistort_maps = {}

def calculate_reprojection_errors(image_points, object_points, camera_poses):
    errors = np.array([])
    for image_points_i, object_point in zip(image_points, object_points):
        error = calculate_reprojection_error(image_points_i, object_point, camera_poses)
        if error is None:
            continue
        errors = np.concatenate([errors, [error]])

    return errors

def calculate_reprojection_error(image_points, object_point, camera_poses):
    cameras = Cameras.instance()

    image_points = np.array(image_points)
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return None

    image_points_t = image_points.transpose((0,1))

    errors = np.array([])
    for i, camera_pose in enumerate(camera_poses):
        if np.all(image_points[i] == None, axis=0):
            continue
        projected_img_points, _ = cv.projectPoints(
            np.expand_dims(object_point, axis=0).astype(np.float32),
            np.array(camera_pose["R"], dtype=np.float64),
            np.array(camera_pose["t"], dtype=np.float64),
            cameras.get_camera_params(i)["intrinsic_matrix"],
            np.array([])
        )
        projected_img_point = projected_img_points[:,0,:][0]
        errors = np.concatenate([errors, (image_points_t[i]-projected_img_point).flatten() ** 2])

    return errors.mean()

def bundle_adjustment(image_points, camera_poses, socketio):
    cameras = Cameras.instance()

    def params_to_camera_poses(params):
        focal_distances = []
        num_cameras = int((params.size-1)/7)+1
        camera_poses = [{
            "R": np.eye(3),
            "t": np.array([0,0,0], dtype=np.float32)
        }]
        focal_distances.append(params[0])
        for i in range(0, num_cameras-1):
            focal_distances.append(params[i*7+1])
            camera_poses.append({
                "R": Rotation.as_matrix(Rotation.from_rotvec(params[i*7 + 2 : i*7 + 3 + 2])),
                "t": params[i*7 + 3 + 2 : i*7 + 6 + 2]
            })

        return camera_poses, focal_distances

    def residual_function(params):
        camera_poses, focal_distances = params_to_camera_poses(params)
        for i in range(0, len(camera_poses)):
            intrinsic = cameras.get_camera_params(i)["intrinsic_matrix"]
            intrinsic[0, 0] = focal_distances[i]
            intrinsic[1, 1] = focal_distances[i]

        object_points = triangulate_points(image_points, camera_poses)
        errors = calculate_reprojection_errors(image_points, object_points, camera_poses)
        errors = errors.astype(np.float32)
        socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})

        return errors

    focal_distance = cameras.get_camera_params(0)["intrinsic_matrix"][0,0]
    init_params = np.array([focal_distance])
    for i, camera_pose in enumerate(camera_poses[1:]):
        rot_vec = Rotation.as_rotvec(Rotation.from_matrix(camera_pose["R"])).flatten()
        focal_distance = cameras.get_camera_params(i)["intrinsic_matrix"][0,0]
        init_params = np.concatenate([init_params, [focal_distance]])
        init_params = np.concatenate([init_params, rot_vec])
        init_params = np.concatenate([init_params, camera_pose["t"].flatten()])

    res = optimize.least_squares(
        residual_function, init_params, verbose=2, loss="cauchy", ftol=1E-2
    )
    return params_to_camera_poses(res.x)[0]

def triangulate_point(image_points, camera_poses):
    image_points = np.array(image_points)
    cameras = Cameras.instance()
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return [None, None, None]

    Ps = []

    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    def DLT(Ps, image_points):
        A = []

        for P, image_point in zip(Ps, image_points):
            A.append(image_point[1]*P[2,:] - P[1,:])
            A.append(P[0,:] - image_point[0]*P[2,:])

        A = np.array(A).reshape((len(Ps)*2,4))
        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)
        object_point = Vh[3,0:3]/Vh[3,3]

        return object_point

    object_point = DLT(Ps, image_points)

    return object_point

def triangulate_points(image_points, camera_poses):
    object_points = []
    for image_points_i in image_points:
        object_point = triangulate_point(image_points_i, camera_poses)
        object_points.append(object_point)

    return np.array(object_points)

def find_point_correspondance_and_object_points(image_points, camera_poses, frames):
    cameras = Cameras.instance()

    for image_points_i in image_points:
        try:
            image_points_i.remove([None, None])
        except:
            pass

    correspondances = [[[i]] for i in image_points[0]]

    Ps = []
    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    root_image_points = [{"camera": 0, "point": point} for point in image_points[0]]

    for i in range(1, len(camera_poses)):
        epipolar_lines = []
        for root_image_point in root_image_points:
            F = sfm.fundamentalFromProjections(Ps[root_image_point["camera"]], Ps[i])
            line = cv.computeCorrespondEpilines(np.array([root_image_point["point"]], dtype=np.float32), 1, F)
            epipolar_lines.append(line[0,0].tolist())
            frames[i] = drawlines(frames[i], line[0])

        not_closest_match_image_points = np.array(image_points[i])
        points = np.array(image_points[i])

        for j, [a, b, c] in enumerate(epipolar_lines):
            distances_to_line = np.array([])
            if len(points) != 0:
                distances_to_line = np.abs(a*points[:,0] + b*points[:,1] + c) / np.sqrt(a**2 + b**2)

            possible_matches = points[distances_to_line < EPIPOLAR_MAX_PX].copy()

            distances_to_line = distances_to_line[distances_to_line < EPIPOLAR_MAX_PX]
            possible_matches_sorter = distances_to_line.argsort()
            possible_matches = possible_matches[possible_matches_sorter]

            if len(possible_matches) == 0:
                for possible_group in correspondances[j]:
                    possible_group.append([None, None])
            else:
                not_closest_match_image_points = [row for row in not_closest_match_image_points.tolist() if row != possible_matches.tolist()[0]]
                not_closest_match_image_points = np.array(not_closest_match_image_points)

                new_correspondances_j = []
                for possible_match in possible_matches:
                    temp = copy.deepcopy(correspondances[j])
                    for possible_group in temp:
                        possible_group.append(possible_match.tolist())
                    new_correspondances_j += temp
                correspondances[j] = new_correspondances_j

        for not_closest_match_image_point in not_closest_match_image_points:
            root_image_points.append({"camera": i, "point": not_closest_match_image_point})
            temp = [[[None, None]] * i]
            temp[0].append(not_closest_match_image_point.tolist())
            correspondances.append(temp)

    object_points = []
    errors = []
    for image_points in correspondances:
        object_points_i = triangulate_points(image_points, camera_poses)

        if np.all(object_points_i == None):
            continue

        errors_i = calculate_reprojection_errors(image_points, object_points_i, camera_poses)

        object_points.append(object_points_i[np.argmin(errors_i)])
        errors.append(np.min(errors_i))

    return np.array(errors), np.array(object_points), frames

def numpy_fillna(data):
    data = np.array(data, dtype=object)

    lens = np.array([len(i) for i in data])

    mask = np.arange(lens.max()) < lens[:,None]

    out = np.full((mask.shape[0], mask.shape[1], 2), [None, None])
    out[mask] = np.concatenate(data)
    return out

def drawlines(img1,lines):
    height,width,_ = img1.shape
    def clamp(p):
        return (int(np.clip(p[0], -1e7, 1e7)), int(np.clip(p[1], -1e7, 1e7)))
    for a, b, c in lines:
        color = tuple(np.random.randint(0,255,3).tolist())

        if abs(b) > 1e-9:
            p0, p1 = (0, -c/b), (width, -(c+a*width)/b)
        elif abs(a) > 1e-9:
            p0, p1 = (-c/a, 0), (-c/a, height)
        else:
            continue
        inside, q0, q1 = cv.clipLine((0, 0, width, height), clamp(p0), clamp(p1))
        if inside:
            img1 = cv.line(img1, q0, q1, color,1)
    return img1

def make_square(img):
    x, y, _ = img.shape
    size = max(x, y)
    new_img = np.zeros((size, size, 3), dtype=np.uint8)
    ax,ay = (size - img.shape[1])//2,(size - img.shape[0])//2
    new_img[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img

    feather_pixels = min(8, ay, ax) if ax and ay else min(8, max(ax, ay))
    for i in range(feather_pixels):
        alpha = (i + 1) / feather_pixels
        if ay:
            new_img[ay - i - 1, ax:ax+img.shape[1]] = img[0, :] * (1 - alpha)
            new_img[ay + img.shape[0] + i, ax:ax+img.shape[1]] = img[-1, :] * (1 - alpha)
        if ax:
            new_img[ay:ay+img.shape[0], ax - i - 1] = img[:, 0] * (1 - alpha)
            new_img[ay:ay+img.shape[0], ax + img.shape[1] + i] = img[:, -1] * (1 - alpha)

    return new_img

def camera_pose_to_serializable(camera_poses):
    for i in range(0, len(camera_poses)):
        camera_poses[i] = {k: v.tolist() for (k, v) in camera_poses[i].items()}

    return camera_poses

def cartesian_product(x, y):
    return np.array([[x0, y0] for x0 in x for y0 in y])

def add_white_border(image, border_size):
    height, width = image.shape[:2]
    bordered_image = cv.copyMakeBorder(image, border_size, border_size, border_size, border_size, cv.BORDER_CONSTANT, value=[255, 255, 255])
    return bordered_image
