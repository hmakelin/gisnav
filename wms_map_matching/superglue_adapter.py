"""This module adapts the SuperGlue match_pairs.py demo code for this app."""
from models.matching import Matching
from models.utils import estimate_pose, make_matching_plot


class SuperGlue():
    """Matches img to map, adapts code from match_pairs.py so that do not have to write files to disk."""

    def __init__(self, logger=None):
        """Init the SuperGlue matcher.

        Args:
            logger - ROS2 node logger for logging messages."""
        self._device = 'cuda' if torch.cuda.is_available() and not opt.force_cpu else 'cpu'
        self._logger = logger
        if self._logger is not None:
            self._logger.debug('SuperGlue using device {}'.format(device))

        # Recommended outdoor config from SuperGlue repo's README.md
        self._config = {
            'superpoint': {
                'nms_radius': 3,
                'keypoint_threshold': 0.005,
                'max_keypoints': 2048
            },
            'superglue': {
                'weights': 'outdoor',
                'sinkhorn_iterations': 20,
                'match_threshold': 0.2
            }
        }
        if self._logger is not None:
            self._logger.debug('SuperGlue using config {}'.format(self._config))
        self._matching = Matching(config).eval().to(device)


    def match(self, img, map):
        """Match img to map."""
        pred = self._matching({'image0': img, 'image1': map})  # TODO: check that img and map are formatted correctly
        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}
        kp_img, kp_map = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        # Matching keypoints
        valid = matches > -1
        mkp_img = kp_img[valid]
        mkp_map= kp_map[matches[valid]]
        mconf = conf[valid]

        # Visualize
        color = cm.jet(mconf)
        text = [
            'SuperGlue',
            'Keypoints: {}:{}'.format(len(kp_img), len(kp_map)),
            'Matches: {}'.format(len(mkp_img)),
        ]

        # Display extra parameter info.
        k_thresh = matching.superpoint.config['keypoint_threshold']
        m_thresh = matching.superglue.config['match_threshold']
        small_text = [
            'Keypoint Threshold: {:.4f}'.format(k_thresh),
            'Match Threshold: {:.2f}'.format(m_thresh),
            'Image Pair: {}:{}'.format('img', 'map'),
        ]

        make_matching_plot(
            img, map, kp_img, kp_map, mkp_img, mkp_map, color, text, viz_path, True, True, True, 'Matches', small_text
        )