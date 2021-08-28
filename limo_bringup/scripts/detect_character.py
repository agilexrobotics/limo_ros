
import cv2
import sys
import numpy as np

characters = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
              'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
              '0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
keys = [65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 97, 98, 99, 100, 101, 102,
            103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57]

class DetectCharacter():
    def __init__(self):
        self.image_ = None

    def SetImage(self, path):
        tmp_image = cv2.imread(path)
        padding = 30
        self.image_ = tmp_image[120 + padding:480 -
                                padding, 160+padding:640-padding]

    def GenerateDatabase(self):
        if self.image_ is None:
            print("image is empty")
            return None

        gray = cv2.cvtColor(self.image_, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)

        # thresh = cv2.adaptiveThreshold(gray, 200, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        samples = np.empty((0, 900))
        responses = []

        count = 0

        for cnt in contours:
            if cv2.contourArea(cnt) > 80:
                [x, y, w, h] = cv2.boundingRect(cnt)
                if h > 60 and h < 90:
                    cv2.rectangle(self.image_, (x, y),
                                  (x+w, y+h), (0, 0, 255), 2)
                    roi = thresh[y:y+h, x:x+w]
                    roismall = cv2.resize(roi, (30, 30))
                    cv2.imshow("norm", self.image_)
                    cv2.imshow("small", roismall)

                    key = cv2.waitKey(0)
                    if key == 27:  # ESC
                        return None
                    elif key in keys:
                        # TODO: read uppercase key
                        key = key-32

                        responses.append(key)
                        sample = roismall.reshape((1, 900))
                        samples = np.append(samples, sample, 0)
                        print("the key: ", chr(key))
                    
                    if count > 100:
                        break
                    
                    count+=1

        responses = np.array(responses, np.float32)
        responses = responses.reshape((responses.size,1))

        np.savetxt('generalresponses_chars.data', responses)
        np.savetxt('generalsamples_chars.data', samples)

    def Detect(self):
        responses = np.loadtxt('generalresponses_chars.data', dtype=np.float32)
        samples = np.loadtxt('generalsamples_chars.data', dtype=np.float32)
        responses = responses.reshape((responses.size,1))

        model = cv2.ml.KNearest_create()
        model.train(samples, cv2.ml.ROW_SAMPLE, responses)

        gray = cv2.cvtColor(self.image_, cv2.COLOR_BGR2GRAY)
        out = np.zeros(self.image_.shape, np.uint8)
        ret, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        count = 0
        chars = []

        for cnt in contours:
            if cv2.contourArea(cnt) > 80:
                [x, y, w, h] = cv2.boundingRect(cnt)
                if h > 60 and h < 90:
                    cv2.rectangle(self.image_, (x, y),
                                  (x+w, y+h), (0, 0, 255), 2)
                    roi = thresh[y:y+h, x:x+w]
                    roismall = cv2.resize(roi, (30, 30))
                    roismall = roismall.reshape((1,900))
                    roismall = np.float32(roismall)

                    cv2.imshow("norm", self.image_)
                    cv2.imshow("small", roismall)

                    retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                    string = str((chr(results[0][0])))
                    chars.append(chr((results[0][0])))
                    print("result: ", results[0][0])
                    cv2.putText(out, string, (x, y+h), 0, 1, (0, 255,0))

                    cv2.imshow('out', out)
                    key = cv2.waitKey(0)


if __name__ == "__main__":
    dc = DetectCharacter()
    dc.SetImage(
        '/opt/ros_ws/src/drivers/limo_ros/limo_bringup/scripts/realsense.png')
    # dc.GenerateDatabase()
    dc.Detect()
