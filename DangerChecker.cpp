#include <map>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include "ObjectLog.h"
#include "LinearRegression.cpp"
#define HUMAN_OBJECT_ID 17
#define PREDICT_SECOND 8
#define NUMBER_OF_CENTER 25
#define LOWER_BOUND_CENTER 15
#define HIGHEST_ORDER_TERM 1
#define FPS 30
#define DANGEROUS_DISTANCE 15 // pixel
#define FLUSH_TIME 600

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
    map<int,int> alreadyWarned;
    map<int,int> DangerousDistance;
    int predictionSecond[4];

    int timer = 0;
public:
    DangerChecker(int cam1,int cam2,int cam3);
    /*def point_check(self, id, center, fcnt, frame, color)*/
    int CheckDangerByID(long frame,long id, double X,double Y,long objectID); 
    /*def save_point(self, id, center, fcnt, num=config["NUMBER_OF_CENTER_POINT"])*/
    void SavePointEachID(long frame,long id,double X,double Y,long objectID);
    /*def least_square(self, id)*/
    void GetParamsByLSM(long id);
    pair<long,long> PredictFutureCoordinate(long id);
    pair<long,long> CalculateDistance(long fid,long sid);
    pair<long,long> PredictDangerByDistance();
    void addCoefficients(long id,pair<double,double> result,int whatCoordinate);
    void Flush();
    int getPredictSecByCameraID(long fid,long sid);
    int getDangerDistByCameraID(long fid,long sid);
};

DangerChecker::DangerChecker(int cam1,int cam2,int cam3) {
    predictionSecond[0] = 0;
    predictionSecond[1] = cam1;
    predictionSecond[2] = cam2;
    predictionSecond[3] = cam3;
    DangerousDistance[3] = 75; // 1 & 2 
    DangerousDistance[5] = 25; // 2 & 3 
    DangerousDistance[4] = 25; // 1 & 3
}


void DangerChecker::Flush() {
    if (timer < FLUSH_TIME) {
        timer++;
    }
    else {
        timer = 0;
        centerPointEachID.clear();
        futurePointEachID.clear();
        alreadyWarned.clear();
        parametersEachIDWithX.clear();
        parametersEachIDWithY.clear();
    }
}

// 최초 진입 함수 및 최종 경고 결과 전달
// result 0 -> no warning, 1 -> warning
int DangerChecker::CheckDangerByID(long frame,long id,double X,double Y,long objectID) {
    SavePointEachID(frame,id,X,Y,objectID);
    if (centerPointEachID[id].size() >= LOWER_BOUND_CENTER) {
        GetParamsByLSM(id);
        pair<long,long> result = PredictFutureCoordinate(id);
        if (result.first != -1) {
            string res = "warning: ";
            res.append(to_string(result.first));
            res.append(" and ");
            res.append(to_string(result.second));
            res.append("\n");
            cout << res;
            return 1;
        }
    } 
    return 0;
}

// 센터 좌표 축적 함수
void DangerChecker::SavePointEachID(long frame,long id,double X,double Y,long objectID) {
    ObjectLog ol(frame,X,Y,objectID);
    if (centerPointEachID.find(id) == centerPointEachID.end()) {
        // 존재하지 않음

        centerPointEachID[id].push_back(ol);
        // parameter, future point 의 initialize


        for(int i=0; i<=PREDICT_SECOND; i++) {
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
    // cout << "X=f(T)" << "\n";
    result = leastRegLine(T,X,numberOfCenterPoint,0);
    addCoefficients(id,result,0);
    // cout << "Y=f(T)" << "\n";
    result = leastRegLine(T,Y,numberOfCenterPoint,1);
    addCoefficients(id,result,1);
}


// 좌표 예측 함수
pair<long,long> DangerChecker::PredictFutureCoordinate(long id) {
    deque<pair<long,long>> res;
    long futureFrame = 0;
    double futureX = 0;
    double futureY = 0;
    long currentFrame = centerPointEachID[id].back().frame;
    futurePointEachID[id].front().X,futurePointEachID[id].front().Y = 
    centerPointEachID[id].back().X,centerPointEachID[id].back().Y;
    for (int i=1; i<=PREDICT_SECOND; i++) {
        futureFrame = currentFrame + (FPS *(i));

        // Y = B+AX
        futureX += (parametersEachIDWithX[id][0] + parametersEachIDWithX[id][1] * futureFrame);
        futureY += (parametersEachIDWithY[id][0] + parametersEachIDWithY[id][1] * futureFrame);
        futurePointEachID[id].at(i).X = futureX;
        futurePointEachID[id].at(i).Y = futureY;


    }
    return PredictDangerByDistance();
}

pair<long,long> DangerChecker::PredictDangerByDistance() {
    vector<long> idList;
    vector<long> ind;
    pair<long,long> res(-1,-1);
    for (auto it = futurePointEachID.begin(); it != futurePointEachID.end(); ++it) {
        if (centerPointEachID[it->first].size() >= LOWER_BOUND_CENTER) { 
            idList.push_back(it->first);
        }
    }
    if (idList.size() < 2) {
        return res;
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
        if (alreadyWarned.count(params[0]) != 0 ) {
            continue; // || params[0] == params[1] - 1
        }
        res = CalculateDistance(params[0],params[1]);
        if (res.first != -1) {
            alreadyWarned[params[0]] = params[1];
            alreadyWarned[params[1]] = params[0];
            break;
        }
	}while(next_permutation(ind.begin(), ind.end()));

    return res;
}

pair<long,long> DangerChecker::CalculateDistance(long fid,long sid) {
    pair<long,long> result(-1,-1);
    if (centerPointEachID[fid].begin()->objectId == HUMAN_OBJECT_ID && 
    centerPointEachID[sid].begin()->objectId == HUMAN_OBJECT_ID) {
        return result;
    }
    if (abs(fid - sid) < 10000) {
        return result;
    }
    // long dangerousDistance = pow(DANGEROUS_DISTANCE,2);
    long dangerousDistance = pow(getDangerDistByCameraID(fid,sid),2);
    int maxPredictionSecond = getPredictSecByCameraID(fid,sid);

    for (int t=maxPredictionSecond; t >= 0; --t) {
        double x1 = futurePointEachID[fid].at(t).X;
        double y1 = futurePointEachID[fid].at(t).Y;
        double x2 = futurePointEachID[sid].at(t).X;
        double y2 = futurePointEachID[sid].at(t).Y;

        double distance = pow((x1-x2),2) + pow((y1-y2),2);
        if (distance < dangerousDistance) {
           printf("time = %d\n",t);
           result.first = fid;
           result.second = sid;
           break;
        }

    }
    return result;
}


int DangerChecker::getPredictSecByCameraID(long fid,long sid) {
    int digitOfChannel = 10000;
    int firstCamera = fid / digitOfChannel;
    int secondCamera = sid / digitOfChannel;
    
    return max(predictionSecond[firstCamera],predictionSecond[secondCamera]);

}

int DangerChecker::getDangerDistByCameraID(long fid,long sid) {
    int digitOfChannel = 10000;
    int firstCamera = fid / digitOfChannel;
    int secondCamera = sid / digitOfChannel;
    

    return DangerousDistance[firstCamera+secondCamera];
}