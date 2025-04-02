# Minimal vendored version of RigidTransform from autolab_core
# Only includes what you need for ArUco localization

import numpy as np
from scipy.spatial.transform import Rotation as R

class RigidTransform:
    def __init__(self, rotation, translation, from_frame='world', to_frame='camera'):
        self._rotation = np.asarray(rotation).reshape((3, 3))
        self._translation = np.asarray(translation).reshape((3,))
        self._from_frame = from_frame
        self._to_frame = to_frame

    @property
    def rotation(self):
        return self._rotation

    @property
    def translation(self):
        return self._translation

    @property
    def quaternion(self):
        # Convert rotation matrix to quaternion (x, y, z, w)
        return R.from_matrix(self._rotation).as_quat()  # (x, y, z, w)

    def inverse(self):
        inv_rotation = self._rotation.T
        inv_translation = -inv_rotation @ self._translation
        return RigidTransform(inv_rotation, inv_translation, from_frame=self._to_frame, to_frame=self._from_frame)

    def __mul__(self, other):
        if not isinstance(other, RigidTransform):
            raise TypeError("Can only multiply with another RigidTransform")
        new_rotation = self._rotation @ other.rotation
        new_translation = self._rotation @ other.translation + self._translation
        return RigidTransform(new_rotation, new_translation, from_frame=self._from_frame, to_frame=other.to_frame)

    def __str__(self):
        return f"RigidTransform from '{self._from_frame}' to '{self._to_frame}':\n" \
               f"Translation: {self._translation}\nRotation:\n{self._rotation}"

    def save(self, filename):
        data = {
            'rotation': self._rotation.tolist(),
            'translation': self._translation.tolist(),
            'from_frame': self._from_frame,
            'to_frame': self._to_frame
        }
        import json
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
