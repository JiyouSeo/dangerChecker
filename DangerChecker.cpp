#include <map>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include "ObjectLog.h"
#include "LinearRegression.cpp"
#define PREDICT_SECOND 3
#define NUMBER_OF_CENTER 15
#define LOWER_BOUND_CENTER 10
#define HIGHEST_ORDER_TERM 1
#define FPS 25
#define DANGEROUS_DISTANCE 40 // pixel
using namespace std;



/*frame ID X Y objectID */
class DangerChecker
{
private:
    /* data */
    map<int,deque<ObjectLog>> centerPointEachID;
    map<int,deque<CenterPoint>> futurePointEachID;
    map<int,double [2]> parametersEachIDWithX;
    map<int,double [2]> parametersEachIDWithY;
public:
    // DangerChecker(/* args */);

    /*def point_check(self, id, center, fcnt, frame, color)*/
    int CheckDangerByID(long frame,long id, double X,double Y,long objectID); 
    /*def save_point(self, id, center, fcnt, num=config["NUMBER_OF_CENTER_POINT"])*/
    void SavePointEachID(long frame,long id,double X,double Y,long objectID);
    /*def least_square(self, id)*/
    void GetParamsByLSM(long id);
    int PredictFutureCoordinate(long id);
    pair<long,long> CalculateDistance(long fid,long sid);
    int PredictDangerByDistance();
    void addCoefficients(long id,pair<double,double> result,int whatCoordinate);
    // ~DangerChecker();
};

// DangerChecker::DangerChecker(/* args */)
// {
// }

// DangerChecker::~DangerChecker()
// {
// }


// 최초 진입 함수 및 최종 경고 결과 전달
int DangerChecker::CheckDangerByID(long frame,long id,double X,double Y,long objectID) {
    SavePointEachID(frame,id,X,Y,objectID);
    if (centerPointEachID[id].size() >= LOWER_BOUND_CENTER) {
        GetParamsByLSM(id);
        int result = PredictFutureCoordinate(id);
        return result;
    } 
    return 0;
}

// 센터 좌표 축적 함수
void DangerChecker::SavePointEachID(long frame,long id,double X,double Y,long objectID) {
    ObjectLog ol(frame,X,Y,objectID);
    if (centerPointEachID.find(id) == centerPointEachID.end()) {
        // 존재하지 않음
        ObjectLog ol(frame,X,Y,objectID);
        centerPointEachID[id].push_back(ol);
        // parameter, future point 의 initialize
        for(int i=0; i<PREDICT_SECOND; i++) {
            futurePointEachID[id].push_back({0.0,0.0});
        }
        parametersEachIDWithX[id][0] = 0.0;
        parametersEachIDWithX[id][1] = 0.0;
        parametersEachIDWithY[id][0] = 0.0;
        parametersEachIDWithY[id][1] = 0.0;


    }
    // 존재
    else {
        // 허용량 이상을 저장하게 되면 가장 옛날의 좌표를 제거
        if (centerPointEachID[id].size() > NUMBER_OF_CENTER) {
            centerPointEachID[id].pop_front();
            centerPointEachID[id].push_back(ol);
        } // 허용량 이내면 그대로 추가
        else {
            centerPointEachID[id].push_back(ol);
        }
    }
    
}


void DangerChecker::addCoefficients(long id,pair<double,double> result,int whatCoordinate) {
// coord == 0 -> X, 1-> Y
    if (whatCoordinate==0) {
        // A + BX, 0 is A, 1 is B
        parametersEachIDWithX[id][0] = result.first;
        parametersEachIDWithX[id][1] = result.second;
    } else {
        parametersEachIDWithY[id][0] = result.first;
        parametersEachIDWithY[id][1] = result.second;
    }   
}    


// 최소 자승법 함수
void DangerChecker::GetParamsByLSM(long id) {
    deque<double> X;
    deque<double> Y;
    deque<double> T;
    pair<double,double> result;
    int numberOfCenterPoint = centerPointEachID[id].size();
    for (int i=0; i < numberOfCenterPoint; i++) {
        T.push_back(centerPointEachID[id].at(i).frame);
        X.push_back(centerPointEachID[id].at(i).X);
        Y.push_back(centerPointEachID[id].at(i).Y);
    }
    cout << "X=f(T)" << "\n";
    result = leastRegLine(T,X,numberOfCenterPoint,0);
    addCoefficients(id,result,0);
    cout << "Y=f(T)" << "\n";
    result = leastRegLine(T,Y,numberOfCenterPoint,1);
    addCoefficients(id,result,1);
}


// 좌표 예측 함수
int DangerChecker::PredictFutureCoordinate(long id) {
    deque<pair<long,long>> res;
    long futureFrame = 0;
    double futureX = 0;
    double futureY = 0;
    long currentFrame = centerPointEachID[id].back().frame;
    futurePointEachID[id].front().X,futurePointEachID[id].front().Y = 
    centerPointEachID[id].back().X,centerPointEachID[id].back().Y;
    for (int i=0; i<PREDICT_SECOND; i++) {
        futureFrame = currentFrame + (FPS *(i+1));
        // for j in range(hot + 1):
        //         fur_center_point_x += int(self.parameter[id][0][-j-1][0] * (fur_frame ** j))
        //         fur_center_point_y += int(self.parameter[id][1][-j-1][0] * (fur_frame ** j))
        //     self.future_point[id][i+1] = [fur_center_point_x, fur_center_point_y]
        // for (int term=0; term < HIGHEST_ORDER_TERM+1; term++) {
        //     futurePointEachID[id]
        // }

        // B+AX
        futureX += (parametersEachIDWithX[id][0] + parametersEachIDWithX[id][1] * futureFrame);
        futureY += (parametersEachIDWithY[id][0] + parametersEachIDWithY[id][1] * futureFrame);
        futurePointEachID[id].at(i).X = futureX;
        futurePointEachID[id].at(i).Y = futureY;
        cout << id << "future Point in " << futureFrame << " frame : " << futurePointEachID[id].at(i).X << " " << futurePointEachID[id].at(i).Y << "\n";
    }
    return PredictDangerByDistance();
}

int DangerChecker::PredictDangerByDistance() {
    vector<long> idList;
    vector<long> ind;
    for (auto it = futurePointEachID.begin(); it != futurePointEachID.end(); it++) {
        idList.push_back(it->first);
    }
    for(int i=0; i<2; i++){
		ind.push_back(1);
	}
	// size-2만큼 0을 추가
	for(int i=0; i<idList.size()-2; i++){
		ind.push_back(0);
	}
    sort(ind.begin(),ind.end());
    do{
		vector<int> params;
		for(int i=0; i<ind.size(); i++){
			if(ind[i] == 1){
				params.push_back(idList[i]);
			}
		}
        CalculateDistance(params[0],params[1]);
        
	}while(next_permutation(ind.begin(), ind.end()));

    return 1;
}

pair<long,long> DangerChecker::CalculateDistance(long fid,long sid) {
        // stime = config["PREDICT_SECOND"]
        // for t in range(stime + 1):
        //     x1 = self.future_point[fid][t][0]
        //     y1 = self.future_point[fid][t][1]

        //     x2 = self.future_point[sid][t][0]
        //     y2 = self.future_point[sid][t][1]

        //     distance = ((x1 - x2) ** 2) + ((y1 - y2) ** 2)
        //     dangerous_distance = config["DANGEROUS_DISTANCE"] ** 2
        //     if distance < dangerous_distance:
        //         if fcnt not in self.warning_id:
        //             self.warning_id[fcnt] = [[(fid, sid), t]]
        //         else:
        //             self.warning_id[fcnt].append([(fid, sid), t])
    long dangerousDistance = pow(DANGEROUS_DISTANCE,2);
    pair<long,long> result(0,0);
    for (int t=0; t< PREDICT_SECOND; t++) {
        int x1 = futurePointEachID[fid].at(t).X;
        int y1 = futurePointEachID[fid].at(t).Y;
        int x2 = futurePointEachID[sid].at(t).X;
        int y2 = futurePointEachID[sid].at(t).Y;

        long distance = pow((x1-x2),2) + pow((y1-y2),2);
        if (distance < dangerousDistance) {
           result.first = fid;
           result.second = sid;
        } else {

        }

    }
    return result;
}


