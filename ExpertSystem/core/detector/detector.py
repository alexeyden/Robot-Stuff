import numpy as np
import cv2

from .classifier import BagOfWordsClassifier


class ScanningWindowDetector:
    def __init__(self, classifier: BagOfWordsClassifier):
        self.classifier = classifier
        self.win_size = np.array((200, 200))
        self.win_step_size = np.array((30, 30))
        self.tol = 0.8

        # 0.8 200

    def detect(self, image_path, interactive=False):
        image = cv2.imread(image_path)

        img_h, img_w, _ = image.shape

        matrix = self._detect_scan(image) if not interactive else self._detect_interactive(image)
        matches = self._build_matches_from_matrix(matrix, self.tol)

        return matches

    def _build_matches_from_matrix(self, matrix: np.ndarray, tol: float):
        def flood_fill(m, ij):
            deltas = [(di, dj) for di in (-1, 0, +1) for dj in (-1, 0, +1) if di != 0 or dj != 0]
            q = {ij}
            min_i, min_j = ij
            max_i, max_j = ij
            while q:
                i, j = q.pop()
                min_i, min_j = min(min_i, i), min(min_j, j)
                max_i, max_j = max(max_i, i), max(max_j, j)
                m[i, j] = False
                for di, dj in deltas:
                    in_bounds = 0 <= i + di < m.shape[0] and 0 <= j + dj < m.shape[1]
                    if in_bounds and m[i + di, j + dj]:
                        q.add((i + di, j + dj))
            return np.array((min_j, min_i)), np.array((max_j, max_i))

        match_matrix = matrix > tol

        matches = []
        it = np.nditer(match_matrix, ('multi_index',))
        while not it.finished:
            if it[0]:
                bound_min, bound_max = flood_fill(match_matrix, it.multi_index)
                v0 = bound_min * self.win_step_size[::-1]
                v1 = bound_max * self.win_step_size[::-1] + self.win_size[::-1]
                #vc = (v0 + v1)//2
                prob = matrix[it.multi_index]
                matches.append((v0, v1, prob))
            it.iternext()

        return matches, matrix

    def _make_matrix(self, image):
        matrix_shape = np.floor((np.array(image.shape[:2]) - self.win_size) / self.win_step_size).astype(np.int32)
        matrix = np.zeros(shape=matrix_shape, dtype=np.float32)

        return matrix

    def _detect_scan(self, image):
        img_h, img_w, _ = image.shape
        win_h, win_w = self.win_size
        win_dy, win_dx = self.win_step_size

        matrix = self._make_matrix(image)

        for win_y in range(0, (img_h - win_h)//win_dy):
            for win_x in range(0, (img_w - win_w)//win_dx):
                prob = self._score_window(win_x * win_dx, win_y * win_dy, image)

                if not prob:
                    continue

                matrix[win_y, win_x] = prob

        return matrix

    def _detect_interactive(self, image):
        img_h, img_w, _ = image.shape
        win_w, win_h = self.win_size
        win_dx, win_dy = self.win_step_size

        matrix = self._make_matrix(image)

        ch = 0

        win_x, win_y = 0, 0
        while ch not in (-1, 27):
            if ch == ord('d'):  # right
                win_x += win_dx
            elif ch == ord('a'):  # left
                win_x -= win_dx
            elif ch == ord('w'):  # up
                win_y -= win_dy
            elif ch == ord('s'):  # down
                win_y += win_dy

            match = self._score_window(win_x, win_y, image)

            if match:
                prob = match
                view = image.copy()

                if ch == ord('q'):
                    sub_img = image[win_y:win_y + win_h, win_x:win_x + win_w].copy()
                    kp, _ = self.classifier.surf_detector.detectAndCompute(sub_img, None)
                    cv2.drawKeypoints(sub_img, kp, sub_img, (0, 255, 0), 4)
                    view[win_y:win_y + win_h, win_x:win_x + win_w] = sub_img

                cv2.rectangle(view, (win_x, win_y), (win_x + self.win_size[0], win_y + self.win_size[1]), (255, 0, 0), 1)

                tsize, _ = cv2.getTextSize('{:.3f}'.format(prob), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)
                cv2.rectangle(view,  (win_x, win_y + win_h - tsize[1]),  (win_x + tsize[0], win_y + win_h ), (255, 0, 0), -1)
                cv2.putText(view, '{:.3f}'.format(prob), (win_x, win_y + win_h), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255))

                matrix[win_y // win_dy, win_x // win_dx] = prob
                cv2.imshow('detect', view)

            ch = cv2.waitKey(0)

        return matrix

    def _score_window(self, win_x, win_y, image):
        win_w, win_h = self.win_size
        sub_img = image[win_y:win_y + win_h, win_x:win_x + win_w]
        prob = self.classifier.predict(sub_img)

        if prob is None:
            return None

        return prob
