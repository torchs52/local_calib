import cv2
import numpy as np


def random_split_time_series(data_length, num_splits=5):
    # Initialize the labels array
    labels = np.zeros(data_length, dtype=int)

    # Iterate over the data in chunks of 5
    for i in range(0, data_length, num_splits):
        # Generate random labels for the current chunk
        num_splits_batch = min(num_splits, data_length - i)
        chunk_labels = np.random.permutation(num_splits_batch)

        # Assign the labels to the corresponding positions in the labels array
        labels[i : i + num_splits_batch] = chunk_labels

    return labels


def calc_accuracy_proj3dto2d(rvec, tvec, pt2d, pt3d, ncm1, distCoeffs):
    point3dto2d = cv2.projectPoints(
        objectPoints=np.array(pt3d[:, :3]),
        rvec=rvec,
        tvec=tvec,
        cameraMatrix=ncm1,
        distCoeffs=distCoeffs,
    )[0].reshape(-1, 2)
    diff2d = np.zeros((0, 2))
    diff2d_sum = np.inf
    if len(point3dto2d) == len(pt2d):
        diff2d = point3dto2d - pt2d[:, 0:2]
        diff2d_sum = np.average(np.sqrt(np.sum(diff2d * diff2d, axis=1)))

    return diff2d_sum
