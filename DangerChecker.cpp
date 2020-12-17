#include <map>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include "ObjectLog.h"
#include "LinearRegression.cpp"
#include "WarningResult.h"
#define DIGIT_CH 10000
#define HUMAN_OBJECT_ID 17
#define PREDICT_SECOND 8
#define NUMBER_OF_CENTER 25
#define LOWER_BOUND_CENTER 15
#define HIGHEST_ORDER_TERM 1
#define FPS 30
#define FLUSH_TIME 600
#define DANGEROUS_DIST_IN_CCTV 15

using namespace std;

/*frame ID X Y objectID */
class DangerChecker
{
private:
    /* 
    map<int,deque<ObjectLog>> centerPointEachID : 각 ID별 중심좌표 및 프레임, 오브젝트종류를 Queue 형태로 저장합니다.
    map<int,deque<CenterPoint>> futurePointEachID : 각 ID별 가까운 미래의 중심좌표류를 Queue 형태로 저장합니다.
    map<int,double [2]> parametersEachIDWithX : 최소자승법에 필요한, 각 ID별 객체의 X좌표 방정식 계수입니다.
    map<int,double [2]> parametersEachIDWithY : 최소자승법에 필요한, 각 ID별 객체의 Y좌표 방정식 계수입니다.
    map<int,int> alreadyWarned : 이미 경고한 객체를 저장해 중복 경고가 발생하지 않기 위함입니다.
    map<int,int> DangerousDistance : 각 카메라 암전구역 별 경고를 줘야하는 거리 경계를 설정합니다.
    int predictionSecond[4] : 각 객체 별 몇초 미래의 좌표까지를 계산해 예측할지를 정합니다.
     */
    map<int,deque<ObjectLog>> centerPointEachID;
    map<int,deque<CenterPoint>> futurePointEachID;
    map<int,double [2]> parametersEachIDWithX;
    map<int,double [2]> parametersEachIDWithY;
    map<int,int> alreadyWarned;
    map<int,int> DangerousDistance;
    int predictionSecond[4];

    int timer = 0;
public:
    /*
    WarningResult CheckDangerByID(long frame,long id, double X,double Y,long objectID) : 외부에서 호출해야할 진입 함수입니다. 최종적으로 2개의 ID와 objectID를 담은 구조체를 리턴.
    void SavePointEachID : 각 id 별로 센터 좌표를 Queue 에 push 합니다. 각 ID별로 일정량이 차는 경우 가장오래된 센터좌표기록을 지웁니다.
    GetParamsByLSM : 최소 자승법을 이용해 각 ID 의 미래좌표 예상에 필요한 방정식 계수를 계산하여 parameter Queue 에 저장합니다.
    PredictFutureCoordinate : 각 프레임 별 객체들 간의 미래 좌표 예상 및 거리 연산을 통한 위험예측을 시작하는 함수입니다. 최종적으로 2개의 id를 리턴합니다.
    CalculateDistance : 2개 객체 간의 미래좌표 간 거리를 계산하는 함수입니다. 위험거리 안에 드는 경우 바로 리턴합니다.
    PredictDangerByDistance : 거리 계산 함수를 호출하는 함수입니다. next_permutation 을 통해 미래좌표가 예측되어있는 객체 들을 2개씩 조합하여 위 함수를 호출합니다.
    */
    DangerChecker(int cam1,int cam2,int cam3,int camOneAndTwo, int camTwoAndThree,int camOneAndThree);
    /*def point_check(self, id, center, fcnt, frame, color)*/
    WarningResult CheckDangerByID(long frame,long id, double X,double Y,long objectID); 
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

DangerChecker::DangerChecker(int cam1,int cam2,int cam3,int camOneAndTwo, int camTwoAndThree,int camOneAndThree) {
    predictionSecond[0] = 0;
    predictionSecond[1] = cam1;
    predictionSecond[2] = cam2;
    predictionSecond[3] = cam3;
    DangerousDistance[3] = camOneAndTwo; // 1 & 2 
    DangerousDistance[5] = camTwoAndThree; // 2 & 3 
    DangerousDistance[4] = camOneAndThree; // 1 & 3
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
WarningResult DangerChecker::CheckDangerByID(long frame,long id,double X,double Y,long objectID) {
    SavePointEachID(frame,id,X,Y,objectID);
    WarningResult res;
    if (centerPointEachID[id].size() >= LOWER_BOUND_CENTER) {
        GetParamsByLSM(id);
        pair<long,long> twoResultId = PredictFutureCoordinate(id);
        if(twoResultId.first != -1) {
            // 경고 결과가 리턴되는 경우
            res.fid = twoResultId.first;
            res.sid = twoResultId.second;
            res.firstObjectId =  centerPointEachID[twoResultId.first].begin()->objectId;
            res.secondObjectId = centerPointEachID[twoResultId.second].begin()->objectId;
            res.isDanger = true;
            printf("warning : %ld(#%ld) and %ld(#%ld)\n",res.fid,res.firstObjectId,res.sid,res.secondObjectId);
        }
        
    } 
    return res;
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
    // 이하 3라인은 오브젝트ID 17번인 보행자끼리의 충돌예측을 하지 않기위한 조건문입니다.
    // if (centerPointEachID[fid].begin()->objectId == HUMAN_OBJECT_ID && 
    // centerPointEachID[sid].begin()->objectId == HUMAN_OBJECT_ID) {
    //     return result;
    // }
    // 동일 CCTV 내 객체에 대한 예측을 하지 않기 위한 조건문입니다.
    // if (abs(fid - sid) < 10000) {
    //     return result;
    // }
    long dangerousDistance = pow(getDangerDistByCameraID(fid,sid),2);
    int maxPredictionSecond = getPredictSecByCameraID(fid,sid);

    for (int t=maxPredictionSecond; t >= 0; --t) {
        double x1 = futurePointEachID[fid].at(t).X;
        double y1 = futurePointEachID[fid].at(t).Y;
        double x2 = futurePointEachID[sid].at(t).X;
        double y2 = futurePointEachID[sid].at(t).Y;

        double distance = pow((x1-x2),2) + pow((y1-y2),2);
        if (distance < dangerousDistance) {
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
    int digitOfChannel = DIGIT_CH;
    int firstCamera = fid / digitOfChannel;
    int secondCamera = sid / digitOfChannel;
    int context;
    if (firstCamera==secondCamera) {
        context = firstCamera;
    } else {
        context = firstCamera+secondCamera;
    }

    return DangerousDistance[context];
}