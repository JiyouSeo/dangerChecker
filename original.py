import numpy as np
import cv2


config = {
    "NUMBER_OF_CENTER_POINT": 15,
    "HIGHEST_ORDER_TERM": 1, # 2이상값은 결과가 제대로 도출이 안되는 문제가 있음
    "FPS_OF_VIDEO": 25, # 프레임 단위
    "PREDICT_SECOND": 3, # 초 단위
    "DANGEROUS_DISTANCE": 40, # 픽셀 단위
    "TIME_YOU_INDICATE_BOX": 2 # 초 단위
}


class DangerousZone:
    def __init__(self):
        # id 마다 Data를 저장
        self.center_point = {}
        self.parameter = {}
        self.future_point = {}

        # fcnt (frame count) 마다 Data를 저장
        self.warning_id = {}

        # dangerous zone을 시각화 하기 위한 Timer
        self.warning_flag = {}


    # 객체별 Center좌표를 num 만큼 저장
    def save_point(self, id, center, fcnt, num=config["NUMBER_OF_CENTER_POINT"]):
        # 객체가 처음 식별 된 경우
        if id not in self.center_point:
            # [center의 x좌표, center의 y좌표, frame_count]
            self.center_point[id] = [[center[0], center[1], fcnt]]
            self.parameter[id] = [0] * (config["HIGHEST_ORDER_TERM"] + 1)
            self.future_point[id] = [0] * (config["PREDICT_SECOND"] + 1)
        # 이미 식별된 객체의 center 좌표 추적
        else:
            # 객체별 center좌표를 누락없이 전부 축적
            if num == 1:
                self.center_point[id].append([center[0], center[1], fcnt])
            # 객체별 center좌표를 num크기만큼 축적
            elif num > 1:
                if len(self.center_point[id]) == num:
                    self.center_point[id] = [i for i in self.center_point[id][1:]]
                self.center_point[id].append([center[0], center[1], fcnt])


    # 최소자승법 원리를 이용하여 center좌표들을 기반으로 차량의 이동동선과 가장 비슷한 형태의 함수를 도출
    def least_square(self, id):
        ncp = config["NUMBER_OF_CENTER_POINT"]
        hot = config["HIGHEST_ORDER_TERM"]

        # 필요한 행렬의 생성 및 초기화
        array_F = np.zeros((ncp, hot + 1), dtype=int)
        array_X = np.zeros((ncp, 1), dtype=int)
        array_Y = np.zeros((ncp, 1), dtype=int)

        """ x = f(t) & y = f(t), t = frame_count """
        # array_X = [Xi], array_Y = [Yi]
        for i in range(ncp):
            array_X[i] = self.center_point[id][i][0]
            array_Y[i] = self.center_point[id][i][1]

        # array_F = [ ... (Fi * Fi) , (Fi) , 1]
        for i in range(ncp):
            Fi = self.center_point[id][i][2]
            for j in range(hot + 1):
                array_F[i][-j - 1] = Fi ** j

        # 최소자승법 계산 : (A^(T) * A)^(-1) * A^(T) * Y
        self.parameter[id][0] = np.dot(np.dot(np.linalg.inv(np.dot(array_F.T, array_F)), array_F.T), array_X)
        self.parameter[id][1] = np.dot(np.dot(np.linalg.inv(np.dot(array_F.T, array_F)), array_F.T), array_Y)


    def predict_point(self, id):
        # 영상의 fps값
        fps = config["FPS_OF_VIDEO"]
        hot = config["HIGHEST_ORDER_TERM"]
        # 몇 초의 미래까지 예측할 것인지
        predict_second = config["PREDICT_SECOND"]

        cur_frame = self.center_point[id][-1][2]
        self.future_point[id][0] = [self.center_point[id][-1][0], self.center_point[id][-1][1]]
        for i in range(predict_second):
            fur_frame = cur_frame + (fps * (i + 1))
            fur_center_point_x = 0
            fur_center_point_y = 0
            for j in range(hot + 1):
                fur_center_point_x += int(self.parameter[id][0][-j-1][0] * (fur_frame ** j))
                fur_center_point_y += int(self.parameter[id][1][-j-1][0] * (fur_frame ** j))
            self.future_point[id][i+1] = [fur_center_point_x, fur_center_point_y]


    def calculate_distance(self, fid, sid, fcnt):
        stime = config["PREDICT_SECOND"]
        for t in range(stime + 1):
            x1 = self.future_point[fid][t][0]
            y1 = self.future_point[fid][t][1]

            x2 = self.future_point[sid][t][0]
            y2 = self.future_point[sid][t][1]

            distance = ((x1 - x2) ** 2) + ((y1 - y2) ** 2)
            dangerous_distance = config["DANGEROUS_DISTANCE"] ** 2
            if distance < dangerous_distance:
                if fcnt not in self.warning_id:
                    self.warning_id[fcnt] = [[(fid, sid), t]]
                else:
                    self.warning_id[fcnt].append([(fid, sid), t])


    def is_dangerous(self, id_list, fcnt):
        # center좌표가 충분치 않아 예측좌표가 없는 id를 id_list에서 제거
        for id in id_list[:]:
            if len(self.center_point[id]) < config["NUMBER_OF_CENTER_POINT"]:
                id_list.remove(id)

        for fid in id_list:
            for sid in id_list:
                if fid < sid:
                    self.calculate_distance(fid, sid, fcnt)


    def draw_warning_zone(self, fcnt, frame):
        overlap_flag = 0
        overlap = []
        timer = config["FPS_OF_VIDEO"] * config["TIME_YOU_INDICATE_BOX"]
        for i in range(fcnt, (fcnt - timer), -1):
            if i not in self.warning_flag:
                continue

            for j in range(len(self.warning_flag[i])):
                for k in range(len(overlap)):
                    overlap_flag = 0
                    if self.warning_flag[i][j][0] is not overlap[k][0]:
                        continue
                    if self.warning_flag[i][j][1] is not overlap[k][1]:
                        continue
                    if self.warning_flag[i][j][2] is not overlap[k][2]:
                        continue
                    overlap_flag = 1
                    break

                if overlap_flag == 1:
                    continue

                overlap.append([self.warning_flag[i][j][0], self.warning_flag[i][j][1], self.warning_flag[i][j][2]])

                rect = self.warning_flag[i][j][3]
                top_left = self.warning_flag[i][j][4]
                cv2.polylines(frame, [rect], True, (0, 0, 255), 3)
                cv2.putText(frame, "warning", (top_left[0], top_left[1]), 0, 5e-3 * 300, (0, 0, 255), 2)

        if fcnt not in self.warning_id:
            return

        index = len(self.warning_id[fcnt])
        for i in range(index):
            if fcnt not in self.warning_flag:
                self.warning_flag[fcnt] = []

            fid = self.warning_id[fcnt][i][0][0]
            sid = self.warning_id[fcnt][i][0][1]
            time = self.warning_id[fcnt][i][1]

            fid_point = self.future_point[fid][time]
            sid_point = self.future_point[sid][time]
            middle_x = (fid_point[0] + sid_point[0]) // 2
            middle_y = (fid_point[1] + sid_point[1]) // 2

            top_left     = [(middle_x - 75), (middle_y - 60)]
            top_right    = [(middle_x + 75), (middle_y - 60)]
            bottom_right = [(middle_x + 75), (middle_y + 60)]
            bottom_left  = [(middle_x - 75), (middle_y + 60)]

            rect = np.array([top_left, top_right, bottom_right, bottom_left], np.int32)
            rect = [fid, sid, time, rect.reshape((-1, 1, 2)), top_left]
            self.warning_flag[fcnt].append(rect)


    def point_check(self, id, center, fcnt, frame, color):
        # center 좌표와 그 때의 frame count를 축적
        self.save_point(id, center, fcnt)

        # 충분한 data
        if len(self.center_point[id]) >= config["NUMBER_OF_CENTER_POINT"]:
            # least_square를 이용해 함수의 prameter 계산
            self.least_square(id)

            # 위치좌표 예측
            self.predict_point(id)
            cv2.line(frame, (self.center_point[id][-1][0], self.center_point[id][-1][1]),
                     (self.future_point[id][-1][0], self.future_point[id][-1][1]), (0, 255, 0), 2)