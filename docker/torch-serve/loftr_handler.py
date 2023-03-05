import importlib
import pickle

import torch
import cv2
import numpy as np
from ts.torch_handler.base_handler import BaseHandler

def bgr2gray(bgr):
    """Convert (OpenCV) BGR format to grayscale"""
    b, g, r = bgr[:,:,0], bgr[:,:,1], bgr[:,:,2]
    gray = 0.114 * b + 0.587 * g + 0.299 * r
    return gray

class LoFTRHandler(BaseHandler):

    MIN_MATCHES = 15
    """Minimum amount of keypoint matches for a good match"""

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def _load_pickled_model(self, _, model_file, model_pt_path):
        """Modify parent method to recognize LoFTR model definition inside loftr.py and to pass default config"""
        module = importlib.import_module(model_file.split(".")[0])
        model_ = getattr(module, 'LoFTR')
        model = model_()
        state_dict = torch.load(model_pt_path, map_location=self.device)
        model.load_state_dict(state_dict)
        return model

    def preprocess(self, data):
        """Convert incoming images to torch tensors"""
        data = data[0]
        qry = pickle.loads(data['query'])
        ref = pickle.loads(data['reference'])
        k = pickle.loads(data['k'])
        elevation = pickle.loads(data['elevation'])
        qry_grayscale = bgr2gray(qry)
        ref_grayscale = bgr2gray(ref)
        qry_tensor = torch.Tensor(qry_grayscale)[None][None].cuda() / 255.
        ref_tensor = torch.Tensor(ref_grayscale)[None][None].cuda() / 255.
        batch = {'image0': qry_tensor, 'image1': ref_tensor}
        return batch, k, elevation

    def inference(self, batch):
        """Do keypoint matching"""
        batch, k, elevation = batch
        with torch.no_grad():
            results = self.model(batch)
        return results, k, elevation

    def postprocess(self, data):
        """Filter based on confidence threshold"""
        batch, k, elevation = data
        mkp_qry = batch['keypoints0'].cpu().numpy()
        mkp_ref = batch['keypoints1'].cpu().numpy()
        conf = batch['confidence'].cpu().numpy()
        valid = conf > self.CONFIDENCE_THRESHOLD

        mkp_qry = mkp_qry[valid, :]
        mkp_ref = mkp_ref[valid, :]

        if mkp_qry is None or len(mkp_qry) < self.MIN_MATCHES:
            return [{}]  # Not enough matching keypoints found

        if elevation is None:
            # Set world z values to zero (assume planar ground surface and estimate homography)
            z_values = np.array([[0]] * len(mkp_qry))
        else:
            # Z values are assumed to be in camera native coordinates (not in meters)
            x, y = np.transpose(np.floor(mkp_ref).astype(int))  # do floor first before converting to int
            z_values = elevation[y, x].reshape(-1, 1)  # y axis first

        mkp2_3d = np.hstack((mkp_ref, z_values))

        dist_coeffs = np.zeros((4, 1))
        _, r, t, __ = cv2.solvePnPRansac(mkp2_3d, mkp_qry, k, dist_coeffs, None, None, useExtrinsicGuess=False,
                                         iterationsCount=10)
        r, _ = cv2.Rodrigues(r)

        return [{'r': r.tolist(), 't': t.tolist()}]
