#include <map>
#include <string>
#include <vector>
#include <deque>
#include "ObjectLog.h"
#define PREDICT_SECOND 3
#define NUMBER_OF_CENTER 15
#define LOWER_BOUND_CENTER 10
using namespace std;



/*frame ID X Y objectID */
class DangerChecker
{
private:
    /* data */
    map<int,deque<ObjectLog>> centerPointEachID;
    map<int,deque<CenterPoint>> futurePointEachID;
    map<int,double [2]> parametersEachID;
public:
    DangerChecker(/* args */);
    /*def point_check(self, id, center, fcnt, frame, color)*/
    int CheckDangerByID(long frame,long id,long double X,double Y,long objectID); 
    /*def save_point(self, id, center, fcnt, num=config["NUMBER_OF_CENTER_POINT"])*/
    void SavePointEachID(long frame,long id,long double X,double Y,long objectID);
    /*def least_square(self, id)*/
    void GetParamsByLSM(long id);
    int PredictFutureCoordinate(long id);
    ~DangerChecker();
};

DangerChecker::DangerChecker(/* args */)
{
}

DangerChecker::~DangerChecker()
{
}


// 최초 진입 함수 및 최종 경고 결과 전달
int DangerChecker::CheckDangerByID(long frame,long id,long double X,double Y,long objectID) {
    SavePointEachID(frame,id,X,Y,objectID);
    if (centerPointEachID[id].size() >= LOWER_BOUND_CENTER) {
        GetParamsByLSM(id);
        int result = PredictFutureCoordinate(id);
        return result;
    } 
    return 0;
}

// 센터 좌표 축적 함수
void DangerChecker::SavePointEachID(long frame,long id,long double X,double Y,long objectID) {
    ObjectLog ol(frame,X,Y,objectID);
    if (centerPointEachID.find(id) == centerPointEachID.end()) {
        // 존재하지 않음
        ObjectLog ol(frame,X,Y,objectID);
        centerPointEachID[id].push_back(ol);
        // parameter, future point 의 initialize
        for(int i=0; i<PREDICT_SECOND; i++) {
            futurePointEachID[id].push_back({0.0,0.0});
        }
        parametersEachID[id][0] = 0.0;
        parametersEachID[id][1] = 0.0;



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

// 최소 자승법 함수
void DangerChecker::GetParamsByLSM(long id) {

}


// 좌표 예측 함수
int DangerChecker::PredictFutureCoordinate(long id) {
    return 1;
}