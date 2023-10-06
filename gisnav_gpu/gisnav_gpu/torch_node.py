import importlib

import numpy as np
import torch
from kornia.feature import LoFTR
from rclpy.node import Node

model = LoFTR(pretrained="outdoor")
torch.save(model.state_dict(), "loftr.pth")


def bgr2gray(bgr):
    """Converts BGR format image to grayscale."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


class TorchNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _load_pickled_model(self, _, model_file, model_pt_path):
        """Loads the :term:`LoFTR` model"""
        module = importlib.import_module(model_file.split(".")[0])
        model_cls = getattr(module, "LoFTR")
        model = model_cls()
        state_dict = torch.load(model_pt_path, map_location=self.device)
        model.load_state_dict(state_dict)
        return model

    def preprocess(self, data):
        """Converts incoming images to torch tensors"""
        data = data[0]
        query_img, reference_img, k_matrix, elevation = self._deserialize_data(data)
        # self._display_images("Query", query_img, "Reference", reference_img)

        qry_tensor, ref_tensor = self._convert_images_to_tensors(
            query_img, reference_img
        )

        return (
            {"image0": qry_tensor, "image1": ref_tensor},
            query_img,
            reference_img,
            k_matrix,
            elevation,
        )

    @staticmethod
    def _deserialize_data(data):
        """Deserializes pickled data"""
        return (
            pickle.loads(data[key]) for key in ["query", "reference", "k", "elevation"]
        )

    # @staticmethod
    # def _display_images(*args):
    #    """Displays images using OpenCV"""
    #    for i in range(0, len(args), 2):
    #        cv2.imshow(args[i], args[i + 1])
    #    cv2.waitKey(1)

    @staticmethod
    def _convert_images_to_tensors(qry, ref):
        """Converts grayscale images to torch tensors"""
        qry_tensor = torch.Tensor(bgr2gray(qry)[None, None]).cuda() / 255.0
        ref_tensor = torch.Tensor(bgr2gray(ref)[None, None]).cuda() / 255.0
        return qry_tensor, ref_tensor

    def inference(self, preprocessed_data):
        """Do keypoint matching."""
        with torch.no_grad():
            results = self.model(preprocessed_data[0])
        return results, *preprocessed_data[1:]

    def postprocess(self, inferred_data):
        """Filters matches based on confidence threshold and calculates :term:`pose`"""
        results, query_img, reference_img, k_matrix, elevation = inferred_data
        mkp_qry, mkp_ref = self._filter_matches_based_on_confidence(results)

        if mkp_qry is None or len(mkp_qry) < self.MIN_MATCHES:
            return [{}]

        mkp2_3d = self._compute_3d_points(mkp_ref, elevation)
        pose = self._compute_pose(mkp2_3d, mkp_qry, k_matrix)
        self._visualize_matches_and_pose(
            query_img, reference_img, mkp_qry, mkp_ref, k_matrix, *pose
        )

        return [{"r": pose[0].tolist(), "t": pose[1].tolist()}]

    def _filter_matches_based_on_confidence(self, results):
        """Filters matches based on confidence threshold"""
        conf = results["confidence"].cpu().numpy()
        valid = conf > self.CONFIDENCE_THRESHOLD
        return (
            results["keypoints0"].cpu().numpy()[valid, :],
            results["keypoints1"].cpu().numpy()[valid, :],
        )

    @staticmethod
    def _compute_3d_points(mkp_ref, elevation):
        """Computes 3D points from matches"""
        if elevation is None:
            return np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))

        x, y = np.transpose(np.floor(mkp_ref).astype(int))
        z_values = elevation[y, x].reshape(-1, 1)
        return np.hstack((mkp_ref, z_values))

    @staticmethod
    def _compute_pose(mkp2_3d, mkp_qry, k_matrix):
        """Computes :term:`pose` using :func:`cv2.solvePnPRansac`"""
        dist_coeffs = np.zeros((4, 1))
        _, r, t, _ = cv2.solvePnPRansac(
            mkp2_3d,
            mkp_qry,
            k_matrix,
            dist_coeffs,
            useExtrinsicGuess=False,
            iterationsCount=10,
        )
        r_matrix, _ = cv2.Rodrigues(r)
        return r_matrix, t

    def _visualize_matches_and_pose(self, qry, ref, mkp_qry, mkp_ref, k, r, t):
        """Visualizes matches and projected :term:`FOV`"""
        h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
        projected_fov = self._project_fov(qry, h_matrix)
        img_with_fov = cv2.polylines(
            ref, [np.int32(projected_fov)], True, 255, 3, cv2.LINE_AA
        )

        mkp_qry = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_qry]
        mkp_ref = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_ref]
        matches = [cv2.DMatch(i, i, 0) for i in range(len(mkp_qry))]

        match_img = cv2.drawMatches(
            img_with_fov,
            mkp_ref,
            qry,
            mkp_qry,
            matches,
            None,
            matchColor=(0, 255, 0),
            flags=2,
        )

        cv2.imshow("Matches and field of view", match_img)
        cv2.waitKey(1)

    @staticmethod
    def _project_fov(img, h_matrix):
        """Projects :term:`FOV` on :term:`reference` image"""
        height, width = img.shape[0:2]
        src_pts = np.float32(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]
        ).reshape(-1, 1, 2)
        try:
            return cv2.perspectiveTransform(src_pts, np.linalg.inv(h_matrix))
        except np.linalg.LinAlgError:
            return src_pts
