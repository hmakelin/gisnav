import importlib
import pickle

import cv2
import numpy as np
import torch
from ts.torch_handler.base_handler import BaseHandler


def bgr2gray(bgr):
    """Convert (OpenCV) BGR format to grayscale"""
    b, g, r = bgr[:, :, 0], bgr[:, :, 1], bgr[:, :, 2]
    gray = 0.114 * b + 0.587 * g + 0.299 * r
    return gray


class LoFTRHandler(BaseHandler):
    MIN_MATCHES = 15
    """Minimum amount of keypoint matches for a good match"""

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def _load_pickled_model(self, _, model_file, model_pt_path):
        """Recognize LoFTR model definition inside loftr.py and pass config"""
        module = importlib.import_module(model_file.split(".")[0])
        model_ = getattr(module, "LoFTR")
        model = model_()
        state_dict = torch.load(model_pt_path, map_location=self.device)
        model.load_state_dict(state_dict)
        return model

    def preprocess(self, data):
        """Convert incoming images to torch tensors"""
        data = data[0]
        qry = pickle.loads(data["query"])
        ref = pickle.loads(data["reference"])
        k = pickle.loads(data["k"])
        elevation = pickle.loads(data["elevation"])
        qry_grayscale = bgr2gray(qry)
        ref_grayscale = bgr2gray(ref)
        qry_tensor = torch.Tensor(qry_grayscale)[None][None].cuda() / 255.0
        ref_tensor = torch.Tensor(ref_grayscale)[None][None].cuda() / 255.0
        batch = {"image0": qry_tensor, "image1": ref_tensor}
        return batch, qry, ref, k, elevation

    def inference(self, preprocessed_data):
        """Do keypoint matching"""
        batch, qry, ref, k, elevation = preprocessed_data
        with torch.no_grad():
            results = self.model(batch)
        return results, qry, ref, k, elevation

    def postprocess(self, inferred_data):
        """Filter based on confidence threshold"""
        results, qry, ref, k, elevation = inferred_data
        mkp_qry = results["keypoints0"].cpu().numpy()
        mkp_ref = results["keypoints1"].cpu().numpy()
        conf = results["confidence"].cpu().numpy()
        valid = conf > self.CONFIDENCE_THRESHOLD

        mkp_qry = mkp_qry[valid, :]
        mkp_ref = mkp_ref[valid, :]

        print(f"{np.amax(mkp_ref, axis=0)} {np.amax(mkp_ref, axis=1)}")

        if mkp_qry is None or len(mkp_qry) < self.MIN_MATCHES:
            return [{}]  # Not enough matching keypoints found

        if elevation is None:
            # Set world z values to zero
            # assume planar ground surface and estimate homography
            z_values = np.array([[0]] * len(mkp_qry))
        else:
            # Z values assumed to be in camera native coordinates (not meters)
            x, y = np.transpose(
                np.floor(mkp_ref).astype(int)
            )  # do floor first before converting to int
            z_values = elevation[y, x].reshape(-1, 1)  # y axis first  # TODO: correct?

        mkp2_3d = np.hstack((mkp_ref, z_values))

        dist_coeffs = np.zeros((4, 1))
        _, r, t, __ = cv2.solvePnPRansac(
            mkp2_3d,
            mkp_qry,
            k,
            dist_coeffs,
            None,
            None,
            useExtrinsicGuess=False,
            iterationsCount=10,
        )
        r, _ = cv2.Rodrigues(r)

        self.visualize(qry, ref, mkp_qry, mkp_ref, k, r, t)

        return [{"r": r.tolist(), "t": t.tolist()}]

    @staticmethod
    def _make_keypoint(pt, sz=1.0):
        """Converts tuple to a cv2.KeyPoint."""
        return cv2.KeyPoint(pt[0], pt[1], sz)

    def visualize(self, qry, ref, mkp_qry, mkp_ref, k, r, t):
        print(mkp_qry.shape)

        # Visualize matches and projected FOV
        h = k @ np.delete(np.hstack((r, t)), 2, 1)
        height, width = qry.shape[0:2][::-1]  # cv2 flips axis order
        src_pts = np.float32(
            [[0, 0], [height - 1, 0], [height - 1, width - 1], [0, width - 1]]
        ).reshape(-1, 1, 2)
        try:
            fov_pix = cv2.perspectiveTransform(src_pts, np.linalg.inv(h))
            map_with_fov = cv2.polylines(
                ref,
                [np.int32(fov_pix)],
                True,
                255,
                3,
                cv2.LINE_AA,
            )
        except np.linalg.LinAlgError:
            # cannot project FOV
            map_with_fov = ref

        draw_params = dict(
            matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2
        )

        # Need cv2.KeyPoints and cv2.DMatches to make cv2.drawMatches work
        mkp_ref = np.apply_along_axis(self._make_keypoint, 1, mkp_ref)
        mkp_qry = np.apply_along_axis(self._make_keypoint, 1, mkp_qry)
        matches = list(map(lambda i: cv2.DMatch(i, i, 0), range(0, len(mkp_qry))))

        img = cv2.drawMatches(
            map_with_fov, mkp_ref, qry, mkp_qry, matches, None, **draw_params
        )

        cv2.imshow("Matches and field of view", img)
        cv2.waitKey(1)
